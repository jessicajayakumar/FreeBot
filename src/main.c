/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */

//********************************************************************************
// Including header files
//********************************************************************************

#include <zephyr/types.h> // Include Zephyr type definitions
#include <zephyr/kernel.h> // Include Zephyr kernel functions and macros
#include <zephyr/usb/usb_device.h> // Include USB device support

#include <zephyr/device.h> // Include device model API
#include <zephyr/devicetree.h> // Include device tree macros and functions
#include <soc.h> // Include SoC-specific definitions

#include <zephyr/drivers/gpio.h> // Include GPIO driver APIs

#include <zephyr/bluetooth/bluetooth.h> // Include Bluetooth core APIs
#include <zephyr/bluetooth/uuid.h> // Include Bluetooth UUID definitions
#include <zephyr/bluetooth/gatt.h> // Include Bluetooth GATT (Generic Attribute Profile) APIs
#include <zephyr/bluetooth/hci.h> // Include Bluetooth HCI (Host Controller Interface) APIs

#include <bluetooth/services/nus.h> // Include Nordic UART Service (NUS) definitions

// #include <dk_buttons_and_leds.h> // Include DK buttons and LEDs support

#include <zephyr/settings/settings.h> // Include settings storage APIs

#include <stdio.h> // Include standard I/O functions
#include <string.h> // Include string manipulation functions

#include <zephyr/logging/log.h> // Include Zephyr logging APIs

#include "freebot.h" // Include FreeBot header file that includes the various FreeBot modules

//********************************************************************************
// Defining macros
//********************************************************************************

#define LOG_MODULE_NAME freebot_uart // Define the log module name
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG); // Register the log module with debug level

#define STACKSIZE CONFIG_BT_NUS_THREAD_STACK_SIZE // Define the stack size for the thread
#define PRIORITY 7 // Define the thread priority

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME // Define the Bluetooth device name
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1) // Calculate the length of the device name

#define RUN_LED_BLINK_INTERVAL 1000 // Define the blink interval for the run status LED in milliseconds

//********************************************************************************
// Defining FB controls
//********************************************************************************

#define LED1 D15 // Define the pin for LED1
#define LED2 D16 // Define the pin for LED2

#define SW2_NODE DT_NODELABEL(sw2) 
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
static struct gpio_callback button_cb_data;


#define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE // Define the UART buffer size
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50) // Define the delay for waiting for UART buffer in milliseconds
#define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME // Define the wait time for UART RX

static K_SEM_DEFINE(ble_init_ok, 0, 1); // Define and initialize a semaphore for BLE initialization

//********************************************************************************
// Function variable declarations
//********************************************************************************

struct k_timer voltage_timer; // Declare a timer for voltage measurement

static struct bt_conn *current_conn; // Pointer to the current Bluetooth connection
static struct bt_conn *auth_conn; // Pointer to the Bluetooth connection for authentication

struct nus_data {
    uint8_t data[256]; // Buffer to hold data
    size_t len; // Length of the data
};

struct msg_data{
    uint8_t data[256]; // Buffer to hold data
    size_t len; // Length of the data
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), // Advertising data: general discoverable and no BR/EDR
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN), // Advertising data: complete device name
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL), // Scan response data: 128-bit UUID for NUS service
};


// Maximum value returned by rand() function
#define RAND_MAX K_MSEC(1600)

static bool data_received = false; // Flag to indicate if data has been received
static bool voltage_send = true; // Flag to indicate if voltage should be sent

static uint32_t volt_delay = 1000; // Voltage send delay

int auth_btn_sw2_clicked;

int handle_msg(struct bt_conn *conn, const uint8_t *const data,
              uint16_t len);
    
int change_param( struct msg_data msg_received);

// ***************************************************************************************************
// FreeBot ID - Change this to the ID of your FreeBot, and change the name in the prj.conf file
// ***************************************************************************************************
uint8_t FB_ID = 0x31; // hex for decimal 49

// **********************************************************
// BLE Connection and configuration
// **********************************************************


static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the Bluetooth address as a string

    if (err) { // Check if there was an error during connection
        LOG_ERR("Connection failed (err %u)", err); // Log the error
        return; // Exit the function if there was an error
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string
    LOG_INF("Connected %s", addr); // Log the successful connection with the address

    current_conn = bt_conn_ref(conn); // Reference the current connection to keep it active

    fb_set_led(LED1); // Turn on the connection status LED
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the Bluetooth address as a string

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string

    LOG_INF("Disconnected: %s (reason %u)", addr, reason); // Log the disconnection with the address and reason

    if (auth_conn) { // Check if there is an authenticated connection
        bt_conn_unref(auth_conn); // Unreference the authenticated connection
        auth_conn = NULL; // Set the authenticated connection pointer to NULL
    }

    if (current_conn) { // Check if there is a current connection
        bt_conn_unref(current_conn); // Unreference the current connection
        current_conn = NULL; // Set the current connection pointer to NULL
        fb_clear_led(LED1); // Turn off the connection status LED
    }
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void security_changed(struct bt_conn *conn, bt_security_t level,
                 enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the Bluetooth address as a string

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string

    if (!err) { // Check if there was no error in security change
        LOG_INF("Security changed: %s level %u", addr, level); // Log the security level change with the address
    } else {
        LOG_WRN("Security failed: %s level %u err %d", addr,
            level, err); // Log the security failure with the address, level, and error
    }
}
#endif

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected, // Callback for when a connection is established
    .disconnected = disconnected, // Callback for when a connection is disconnected
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
    .security_changed = security_changed, // Callback for when the security level changes (only if security is enabled)
#endif
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED)
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the Bluetooth address as a string

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string

	LOG_INF("Passkey for %s: %06u", addr, passkey); // Log the passkey for the device
}

static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the Bluetooth address as a string

    auth_conn = bt_conn_ref(conn); // Reference the connection to keep it active

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string

    LOG_INF("Passkey for %s: %06u", addr, passkey); // Log the passkey for the device
    LOG_INF("Press Button 1 to confirm, Button 2 to reject."); // Log instructions for confirming or rejecting the passkey
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the Bluetooth address as a string

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string

    LOG_INF("Pairing cancelled: %s", addr); // Log that pairing was cancelled for the device
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the Bluetooth address as a string

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string

    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded); // Log that pairing was completed and whether the device is bonded
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the Bluetooth address as a string

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string

    LOG_INF("Pairing failed conn: %s, reason %d", addr, reason); // Log that pairing failed for the device and the reason
}


static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display, // Callback for displaying the passkey
    .passkey_confirm = auth_passkey_confirm, // Callback for confirming the passkey
    .cancel = auth_cancel, // Callback for cancelling the authentication
};

static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete, // Callback for when pairing is complete
    .pairing_failed = pairing_failed // Callback for when pairing fails
};
#else
static struct bt_conn_auth_cb conn_auth_callbacks; // Define an empty structure for authentication callbacks
static struct bt_conn_auth_info_cb conn_auth_info_callbacks; // Define an empty structure for authentication info callbacks
#endif

// **********************************************************
// BLE data receive
// **********************************************************

static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
              uint16_t len)
{
    // int err;
    char addr[BT_ADDR_LE_STR_LEN] = {0}; // Buffer to hold the Bluetooth address as a string

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr)); // Convert the Bluetooth address to a string

    // LOG_INF("Received data from: %s", addr); // Log the address of the device that sent the data
    // LOG_INF("data received: %d", len); // Log the length of the received data
    // LOG_INF("Data received:");
    // for (uint16_t i = 0; i < len; i++) {
    //     LOG_INF("0x%02x ", data[i]); // Log each byte of the received data in hexadecimal format
    // }

    data_received = true; // Set the flag indicating that data has been received
    handle_msg(conn, data, len); // Call the function to handle the received message
}

// Function to process messages received over BLE
int handle_msg(struct bt_conn *conn, const uint8_t *const data,
              uint16_t len){
    
    struct msg_data msg_received;
    msg_received.len = len;
    memcpy(msg_received.data, (int*) data, len);

    LOG_INF("Data received");
    // print the received data
    for (uint16_t i = 0; i < len; i++) {
        LOG_INF("0x%02x ", msg_received.data[i]); // Log each byte of the received data in hexadecimal format
    }

    // check the first byte of the message
    switch (msg_received.data[0])
    {
        case 0x01:
            LOG_INF("Change param message received");
            change_param(msg_received);
            break;
        
        case 0x02:
            LOG_INF("Message from peer received");
            break;
        
    }
    return 0;
}

// Function to change the parameters of the FreeBot based on message received over BLE
int change_param( struct msg_data msg_received){
    // check the second byte of the message
    switch (msg_received.data[1])
    {
        case 0x01:
            LOG_INF("STOP received");
            break;
        
        case 0x02:
            LOG_INF("START received");
            break;

        case 0x03:
            LOG_INF("Voltage send off received");
            voltage_send = false;
            break;
        case 0x04:
            LOG_INF("Voltage send on received");
            voltage_send = true;
            break;
        
        case 0x05:
            LOG_INF("Change voltage send delay received");
            // Copy the delay value from the message, starting from the third byte
            memcpy(&volt_delay, &msg_received.data[2], sizeof(volt_delay));
            LOG_INF("Voltage send delay: %d", volt_delay);
            volt_delay = volt_delay * 1000; // Convert the delay to milliseconds
            break;
            
        case 0x06:
            LOG_INF("Message from peer received");
            break;
    }
    return 0;

}


static struct bt_nus_cb nus_cb = {
    .received = bt_receive_cb, // Set the callback for receiving data
};

void error(void)
{
    // dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK); // Turn off all LEDs

    while (true) {
        /* Spin forever */
        k_sleep(K_MSEC(1000)); // Sleep for 1000 milliseconds in an infinite loop
    }
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
static void num_comp_reply(bool accept)
{
    if (accept) {
        bt_conn_auth_passkey_confirm(auth_conn); // Confirm the passkey if accepted
        LOG_INF("Numeric Match, conn %p", (void *)auth_conn); // Log that the numeric comparison was accepted
    } else {
        bt_conn_auth_cancel(auth_conn); // Cancel the authentication if rejected
        LOG_INF("Numeric Reject, conn %p", (void *)auth_conn); // Log that the numeric comparison was rejected
    }

    bt_conn_unref(auth_conn); // Unreference the connection
    auth_conn = NULL; // Set the connection reference to NULL
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    if (auth_conn) {
		auth_btn_sw2_clicked++;
	}
}

// **********************************************************
// Main function
// **********************************************************

int main(void)
{
    int blink_status = 0; // Variable to keep track of LED blink status
    int err = 0; // Variable to store error codes

    // configure_gpio(); // Configure GPIOs (buttons and LEDs)
    fb_init(); // Initialize the FB API
    fb_v_measure_select(V_CAP); // Select the voltage measurement

    err = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (err) {
		LOG_ERR("Button configuration failed (err %d)\n", err);
		return 0;
	}

	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin)); 
	gpio_add_callback(button.port, &button_cb_data);

    if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
        err = bt_conn_auth_cb_register(&conn_auth_callbacks); // Register authentication callbacks if security is enabled
        if (err) {
            printk("Failed to register authorization callbacks.\n"); // Print an error message if registration fails
            return 0; // Exit the program
        }

        err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks); // Register authentication info callbacks if security is enabled
        if (err) {
            printk("Failed to register authorization info callbacks.\n"); // Print an error message if registration fails
            return 0; // Exit the program
        }
    }

    err = bt_enable(NULL); // Enable Bluetooth
    if (err) {
        error(); // Call the error function if Bluetooth initialization fails
    }

    LOG_INF("Bluetooth initialized"); // Log that Bluetooth has been initialized

    k_sem_give(&ble_init_ok); // Give the semaphore indicating that BLE initialization is complete

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load(); // Load settings if the CONFIG_SETTINGS option is enabled
    }

    err = bt_nus_init(&nus_cb); // Initialize the Nordic UART Service (NUS)
    if (err) {
        LOG_ERR("Failed to initialize UART service (err: %d)", err); // Log an error if NUS initialization fails
        return 0; // Exit the program
    }

    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd)); // Start Bluetooth advertising
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err); // Log an error if advertising fails to start
        return 0; // Exit the program
    }

    for (;;) {
        fb_set_led(LED2); // Toggle the status LED
        k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL)); // Sleep for the blink interval
    }
    return 0; // Return 0 (though this line is never reached due to the infinite loop)
}

// **********************************************************
// BLE voltage measurement thread
// **********************************************************

void ble_write_thread(void)
{
	/* Don't go any further until BLE is initialized */
	k_sem_take(&ble_init_ok, K_FOREVER);
	struct nus_data data={
		.len = 0,
	};

	uint32_t last_voltage_send_time = k_uptime_get_32();
    k_msleep(8000);

	
    for (;;) {

		if (voltage_send){
            uint32_t current_time = k_uptime_get_32();

            if ((current_time - last_voltage_send_time) >= volt_delay) {
                int v_cap = fb_v_measure();
                //LOG_INF("Voltage: %d", v_cap);
                uint8_t volt_val=(v_cap*100)/3000;

                uint8_t msg[] = {FB_ID, volt_val}; // Voltage
                size_t msg_len = sizeof(msg);

                int loc = 0;

                int plen = MIN(sizeof(data.data) - data.len, msg_len);

                if (plen>0){

                    for (int i = 0; i < plen; i++) {
                    //LOG_INF("Data to be sent over BLE: %d", msg[loc]);
                    }

                    memcpy(&data.data[data.len], &msg[loc], plen);
                    data.len += plen;
                    loc += plen;

                    for (int i = 0; i < data.len; i++) {
                        //LOG_INF("Data to be sent over BLE: %d", data.data[i]);
                    }
                    //LOG_INF("Data length: %d", data.len);

                    if (data.len >= sizeof(data.data) || loc >= msg_len) {
                        if (bt_nus_send(NULL, data.data, data.len)) {
                            LOG_WRN("Failed to send data over BLE connection");
                        }
                        //LOG_INF("data sent over BLE");
                        data.len = 0;
                    }

                    plen = MIN(sizeof(data.data), msg_len - loc);
                    last_voltage_send_time = current_time;
                        
                }
            
            }

            // // Short sleep to prevent busy-waiting and allow other threads to run
            k_msleep(10);
        }

	}
	
}


K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
		NULL, PRIORITY, 0, 0);

// **********************************************************
// BLE motor control thread
// **********************************************************

void ble_motor_control(void){

}

                          