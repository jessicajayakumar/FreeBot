

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/drivers/gpio.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>


#include <zephyr/logging/log.h>

/* Include header files to use built-in temperature sensor */
#include <zephyr/device.h>
/* Include header files to use power management */
#include <zephyr/pm/pm.h>
/* Include header files to use thread analyzer */
#include <zephyr/debug/thread_analyzer.h>

#include <zephyr/sys/sys_heap.h>

#include "freebot.h"

#include "my_fbs.h"



#define LOG_MODULE_NAME freebot_voltage
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE
#define PRIORITY 7

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

#define RUN_STATUS_LED DK_LED1
#define RUN_LED_BLINK_INTERVAL 1000

#define CON_STATUS_LED DK_LED2

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK
#define KEY_PASSKEY_REJECT DK_BTN2_MSK

#define FB_DEMO_DELAY K_MSEC(500)

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;


static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};


/* Code to test the ble notify service of the freebot*/
/* Define UUIDs for the service and characteristic */
#define BT_UUID_CUSTOM_SERVICE BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890AB))
#define BT_UUID_CUSTOM_CHAR BT_UUID_DECLARE_128(BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x1234567890AC))

#define CCCD_NOTIFY_ENABLE 0x0001


static ssize_t read_custom_char(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr, void *buf,
                                uint16_t len, uint16_t offset)
{
    const char *value = "Hello, World!";
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

static ssize_t write_custom_char(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr, const void *buf,
                                 uint16_t len, uint16_t offset, uint8_t flags)
{
    // Handle the data written to the characteristic
    LOG_INF("Data written to characteristic");

    // Send an acknowledgment back to the central device
    const char *ack = "Data received";
    bt_gatt_notify(conn, attr, ack, strlen(ack));

    return len;
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    bool notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notifications %s\n", notifications_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(freebot_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_CHAR,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_custom_char, write_custom_char, NULL),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);



// ****************************************
//  BLE Connection Callbacks
// ****************************************

static void connected(struct bt_conn *conn, uint8_t err) // Define a static function named 'connected' that is called upon a Bluetooth connection event, taking a connection object and an error code as parameters.
{
	char addr[BT_ADDR_LE_STR_LEN]; // Declare a character array to store the Bluetooth address in string format.

	if (err) { // Check if there was an error during the connection attempt.
		LOG_ERR("Connection failed (err %u)", err); // Log the connection failure and the error code.
		return; // Exit the function early if there was an error.
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address of the connected device to a string and store it in 'addr'.
	LOG_INF("Connected %s", addr); // Log a message indicating a successful connection, along with the device's Bluetooth address.

	current_conn = bt_conn_ref(conn); // Store a reference to the current connection in a global/static variable 'current_conn'.

	dk_set_led_on(CON_STATUS_LED); // Turn on an LED to indicate that the connection has been established.
}

static void disconnected(struct bt_conn *conn, uint8_t reason) // Define a static function named 'disconnected' that is called upon a Bluetooth disconnection event, taking a connection object and a reason code as parameters.
{
	char addr[BT_ADDR_LE_STR_LEN]; // Declare a character array to store the Bluetooth address in string format.

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address of the disconnected device to a string and store it in 'addr'.
	LOG_INF("Disconnected: %s (reason %u)", addr, reason); // Log a message indicating the disconnection, along with the device's Bluetooth address and the reason code.

	if (auth_conn) { // Check if there is an authenticated connection stored in 'auth_conn'.
		bt_conn_unref(auth_conn); // Release the reference to the authenticated connection, allowing its resources to be freed.
		auth_conn = NULL; // Set 'auth_conn' to NULL, indicating there's no longer an authenticated connection.
	}

	if (current_conn) { // Check if there is a current connection stored in 'current_conn'.
		bt_conn_unref(current_conn); // Release the reference to the current connection, allowing its resources to be freed.
		current_conn = NULL; // Set 'current_conn' to NULL, indicating there's no longer a current connection.
		dk_set_led_off(CON_STATUS_LED); // Turn off an LED to indicate that the device is no longer connected.
	}
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
			err);
	}

}

// Define a static function to handle the cancellation of Bluetooth pairing.
static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN]; // Declare a character array to store the Bluetooth address in string format.

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address of the device to a string and store it in 'addr'.

	LOG_INF("Pairing cancelled: %s", addr); // Log a message indicating that the pairing process has been cancelled, along with the device's Bluetooth address.
}

// Define a static function to handle the completion of Bluetooth pairing.
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	char addr[BT_ADDR_LE_STR_LEN]; // Declare a character array to store the Bluetooth address in string format.

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address of the device to a string and store it in 'addr'.

	LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded); // Log a message indicating that the pairing process is complete, along with the device's Bluetooth address and the bonding status.
} 

// Define a static function to handle the failure of Bluetooth pairing.
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	char addr[BT_ADDR_LE_STR_LEN]; // Declare a character array to store the Bluetooth address in string format.

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address of the device to a string and store it in 'addr'.

	LOG_INF("Pairing failed conn: %s, reason %d", addr, reason); // Log a message indicating that the pairing process has failed, along with the device's Bluetooth address and the reason for failure.
} 


// ***************************************
//  BLE Callbacks
// ***************************************

static struct bt_conn_auth_cb conn_auth_callbacks = {
	.cancel = auth_cancel,
};

// Define a structure for Bluetooth connection authentication information callbacks.
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete, // Callback for handling completion of pairing.
    .pairing_failed = pairing_failed // Callback for handling failure of pairing.
};

// Define a struct to register callbacks
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
	.security_changed = security_changed,
    // For data received, you might need to set up GATT services and characteristics
};

// Define a function to enter an error state, turning off all LEDs and entering an infinite loop.
void error(void)
{
    dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK); // Turn off all LEDs.

    while (true) { // Enter an infinite loop.
        k_sleep(K_MSEC(1000)); // Sleep for 1000 milliseconds (1 second) in each iteration of the loop.
    }
}


// ***************************************
// Handling received data
// ***************************************

void le_data_received(struct bt_conn *conn, const uint8_t *data, size_t len) {
    // Check for invalid data
    if (data == NULL || len == 0) {
        LOG_ERR("Invalid data received");
        return;
    }

    // Print received data in hexadecimal format
    for (size_t i = 0; i < len; i++) {
        printk("%02x ", data[i]);
    }
    printk("\n");

    // Send acknowledgment to the central device
    const char *ack = "Data received";
    bt_gatt_notify(conn, &freebot_svc.attrs[1], ack, strlen(ack));
}


int main(void)
{
	int blink_status = 0;
	int err = 0;

	LOG_INF("Starting BLE FreeBot\n");

	fb_init();

	err = bt_conn_auth_cb_register(&conn_auth_callbacks);
	if (err) {
		LOG_ERR("Failed to register authorization callbacks.");
		return 0;
	}

	err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
	if (err) {
		printk("Failed to register authorization info callbacks.\n");
		return 0;
	}

	bt_conn_cb_register(&conn_callbacks);
	
	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd,
			      ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return 0;
	}

	for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}
