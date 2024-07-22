/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Bridge Service (NUS) sample
 */
// #include "uart_async_adapter.h"  //We don't need this header file, UART async adapter is not used in this example

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/settings/settings.h>

#include <stdio.h>
#include <string.h>

#include <zephyr/logging/log.h>

#include "freebot.h"

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

// #define UART_BUF_SIZE CONFIG_BT_NUS_UART_BUFFER_SIZE
// #define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
// #define UART_WAIT_FOR_RX CONFIG_BT_NUS_UART_RX_WAIT_TIME

#define FB_DEMO_DELAY K_MSEC(500)

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static struct bt_conn *current_conn;
static struct bt_conn *auth_conn;

// static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
// static struct k_work_delayable uart_work;

// struct uart_data_t {
// 	void *fifo_reserved;
// 	uint8_t data[UART_BUF_SIZE];
// 	uint16_t len;
// };

// static K_FIFO_DEFINE(fifo_uart_tx_data);
// static K_FIFO_DEFINE(fifo_uart_rx_data);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

// #ifdef CONFIG_UART_ASYNC_ADAPTER
// UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
// #else
// #define async_adapter NULL
// #endif

int pwr_measure_demo(void)
{
    fb_v_measure_select(V_CAP);
    k_sleep(FB_DEMO_DELAY);
    int v_cap = fb_v_measure();
    LOG_DBG("Vcap = %dmV", v_cap);
	return v_cap;
}

void ble_stack_init(void) {
    ret_code_t err_code;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using settings for the application.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}

/* UART is not required for sending the voltage back to the DK board as it is not receiving the voltage readings from the serial terminal
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev); // Macro to suppress unused parameter warning for 'dev'

    static size_t aborted_len; // Static variable to keep track of length of data not sent if a transmission was aborted
    struct uart_data_t *buf; // Pointer to hold the address of the UART data structure
    static uint8_t *aborted_buf; // Static variable to keep track of the buffer that was being sent if a transmission was aborted
    static bool disable_req; // Static variable to indicate if a disable request has been made (unused in this snippet)

    switch (evt->type) { // Switch on the type of UART event
    case UART_TX_DONE: // Case when UART transmission is completed
        LOG_DBG("UART_TX_DONE"); // Log debug message indicating transmission is done
        if ((evt->data.tx.len == 0) || // If the length of transmitted data is 0 OR
            (!evt->data.tx.buf)) { // the buffer pointer is NULL, then exit the function
            return;
        }

        if (aborted_buf) { // If there was an aborted transmission (aborted_buf is not NULL)
            buf = CONTAINER_OF(aborted_buf, struct uart_data_t, // Retrieve the uart_data_t structure containing the aborted buffer
                       data[0]);
            aborted_buf = NULL; // Reset aborted_buf to NULL
            aborted_len = 0; // Reset aborted_len to 0
        } else { // If there was no aborted transmission
            buf = CONTAINER_OF(evt->data.tx.buf, struct uart_data_t, // Retrieve the uart_data_t structure for the buffer that was just transmitted
                       data[0]);
        }

        k_free(buf); // Free the memory allocated for the uart_data_t structure

        buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT); // Attempt to get the next buffer from the FIFO queue without waiting
        if (!buf) { // If there is no next buffer, exit the function
            return;
        }

        if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) { // Attempt to transmit the next buffer. If it fails,
            LOG_WRN("Failed to send data over UART"); // log a warning message
        }

        break; // Break out of the switch-case

	case UART_RX_RDY: // Case when UART data is ready to be read
		LOG_DBG("UART_RX_RDY"); // Log debug message indicating data is ready for reading
		buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]); // Retrieve the uart_data_t structure for the received buffer
		buf->len += evt->data.rx.len; // Update the length of data in the buffer

		if (disable_req) { // If a disable request has been made,
			return; // exit the function
		}

		if ((evt->data.rx.buf[buf->len - 1] == '\n') || // If the last character received is a newline or carriage return,
			(evt->data.rx.buf[buf->len - 1] == '\r')) {
			disable_req = true; // set the disable request flag
			uart_rx_disable(uart); // and disable UART reception
		}

		break;

	case UART_RX_DISABLED: // Case when UART reception has been disabled
		LOG_DBG("UART_RX_DISABLED"); // Log debug message indicating reception is disabled
		disable_req = false; // Reset the disable request flag

		buf = k_malloc(sizeof(*buf)); // Allocate memory for a new uart_data_t structure
		if (buf) { // If allocation was successful,
			buf->len = 0; // initialize the length of data in the buffer to 0
		} else { // If allocation failed,
			LOG_WRN("Not able to allocate UART receive buffer"); // log a warning message
			k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY); // and reschedule UART work with a delay
			return; // then exit the function
		}

		uart_rx_enable(uart, buf->data, sizeof(buf->data), // Enable UART reception, providing the new buffer
				UART_WAIT_FOR_RX);

		break;

	case UART_RX_BUF_REQUEST: // Case when UART requests a new buffer for reception
		LOG_DBG("UART_RX_BUF_REQUEST"); // Log debug message indicating a new buffer is requested
		buf = k_malloc(sizeof(*buf)); // Allocate memory for a new uart_data_t structure
		if (buf) { // If allocation was successful,
			buf->len = 0; // initialize the length of data in the buffer to 0
			uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data)); // Respond with the new buffer
		} else { // If allocation failed,
			LOG_WRN("Not able to allocate UART receive buffer"); // log a warning message
		}

		break;

	case UART_RX_BUF_RELEASED: // Case when a receive buffer is released
		LOG_DBG("UART_RX_BUF_RELEASED"); // Log debug message indicating a buffer has been released
		buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, // Retrieve the uart_data_t structure for the released buffer
				data[0]);

		if (buf->len > 0) { // If the buffer contains data,
			k_fifo_put(&fifo_uart_rx_data, buf); // put it in the receive FIFO queue
		} else { // If the buffer is empty,
			k_free(buf); // free the buffer
		}

		break;

	case UART_TX_ABORTED: // Case when UART transmission has been aborted
		LOG_DBG("UART_TX_ABORTED"); // Log debug message indicating transmission was aborted
		if (!aborted_buf) { // If there is no previously aborted buffer,
			aborted_buf = (uint8_t *)evt->data.tx.buf; // set aborted_buf to the current buffer
		}

		aborted_len += evt->data.tx.len; // Update the length of data not sent
		buf = CONTAINER_OF((void *)aborted_buf, struct uart_data_t, // Retrieve the uart_data_t structure for the aborted buffer
				data);

		uart_tx(uart, &buf->data[aborted_len], // Attempt to resend the remaining data
			buf->len - aborted_len, SYS_FOREVER_MS);

		break;

	default: // Default case for unhandled events
		break;
	}
} */



// static void uart_work_handler(struct k_work *item)
// {
//     /* Declaration of a pointer to struct uart_data_t */
//     struct uart_data_t *buf;

//     /* Allocate memory for buf to hold UART data */
//     buf = k_malloc(sizeof(*buf));
//     if (buf) {
//         /* Memory allocation successful, initialize buf->len to 0 */
//         buf->len = 0;
//     } else {
//         /* Memory allocation failed, log a warning */
//         LOG_WRN("Not able to allocate UART receive buffer");
        
//         /* Reschedule the work item for later execution */
//         k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY);
        
//         /* Exit the function early since no memory was allocated */
//         return;
//     }

// 	uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_WAIT_FOR_RX); // Enable UART reception: starts receiving data into `buf->data`, up to `sizeof(buf->data)` bytes, with a defined wait timeout `UART_WAIT_FOR_RX`.
// }


/**
 * @brief Checks if the UART driver supports asynchronous API.
 *
 * This function checks if the provided UART device supports the asynchronous API by
 * verifying if the callback_set function pointer is not NULL.
 *
 * @param dev Pointer to the UART device structure.
 * @return true if the UART driver supports asynchronous API, false otherwise.
 */

/*
static bool uart_test_async_api(const struct device *dev)
{
	const struct uart_driver_api *api =
		(const struct uart_driver_api *)dev->api;

	return (api->callback_set != NULL);
}

static int uart_init(void)
{
	int err;
	int pos;
	struct uart_data_t *rx;
	struct uart_data_t *tx;

	// Check if the UART device is ready
	if (!device_is_ready(uart)) {
		return -ENODEV;
	}

	// Enable USB if USB device stack is enabled
	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_ERR("Failed to enable USB");
			return err;
		}
	}

	// Allocate memory for the receive buffer
	rx = k_malloc(sizeof(*rx));
	if (rx) {
		rx->len = 0;
	} else {
		return -ENOMEM;
	}

	// Initialize the UART work item
	k_work_init_delayable(&uart_work, uart_work_handler);

	// If UART async adapter is enabled and the UART driver does not support async API, use the adapter
	if (IS_ENABLED(CONFIG_UART_ASYNC_ADAPTER) && !uart_test_async_api(uart)) {
		//Implement API adapter 
		uart_async_adapter_init(async_adapter, uart);
		uart = async_adapter;
	}

	// Set the UART callback function
	err = uart_callback_set(uart, uart_cb, NULL);
	if (err) {
		k_free(rx);
		LOG_ERR("Cannot initialize UART callback");
		return err;
	}

	// If UART line control is enabled, wait for DTR signal
	if (IS_ENABLED(CONFIG_UART_LINE_CTRL)) {
		LOG_INF("Wait for DTR");
		while (true) {
			uint32_t dtr = 0;

			uart_line_ctrl_get(uart, UART_LINE_CTRL_DTR, &dtr);
			if (dtr) {
				break;
			}
			// Give CPU resources to low priority threads. 
			k_sleep(K_MSEC(100));
		}
		LOG_INF("DTR set");
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DCD, 1);
		if (err) {
			LOG_WRN("Failed to set DCD, ret code %d", err);
		}
		err = uart_line_ctrl_set(uart, UART_LINE_CTRL_DSR, 1);
		if (err) {
			LOG_WRN("Failed to set DSR, ret code %d", err);
		}
	}

	// Allocate memory for the transmit buffer and send a welcome message
	tx = k_malloc(sizeof(*tx));
	if (tx) {
		pos = snprintf(tx->data, sizeof(tx->data),
				   "Starting Nordic UART service example\r\n");

		if ((pos < 0) || (pos >= sizeof(tx->data))) {
			k_free(rx);
			k_free(tx);
			LOG_ERR("snprintf returned %d", pos);
			return -ENOMEM;
		}

		tx->len = pos;
	} else {
		k_free(rx);
		return -ENOMEM;
	}

	err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS);
	if (err) {
		k_free(rx);
		k_free(tx);
		LOG_ERR("Cannot display welcome message (err: %d)", err);
		return err;
	}

	// Enable UART reception with the receive buffer
	err = uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_WAIT_FOR_RX);
	if (err) {
		LOG_ERR("Cannot enable uart reception (err: %d)", err);
		// Free the rx buffer only because the tx buffer will be handled in the callback 
		k_free(rx);
	}

	return err;
}
*/

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

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED // Check if Bluetooth NUS (Nordic UART Service) security is enabled through configuration.
static void security_changed(struct bt_conn *conn, bt_security_t level,
				 enum bt_security_err err) // Define a static function named 'security_changed' to handle security level changes for a Bluetooth connection.
{
	char addr[BT_ADDR_LE_STR_LEN]; // Declare a character array to store the Bluetooth address in string format.

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address of the device to a string and store it in 'addr'.

	if (!err) { // Check if there was no error in changing the security level.
		LOG_INF("Security changed: %s level %u", addr, level); // Log a message indicating the security level change, along with the device's Bluetooth address and the new security level.
	} else { // If there was an error in changing the security level.
		LOG_WRN("Security failed: %s level %u err %d", addr, level, err); // Log a warning message indicating the security level change failure, along with the device's Bluetooth address, the intended security level, and the error code.
	}
}
#endif // End of conditional compilation block for CONFIG_BT_NUS_SECURITY_ENABLED.

BT_CONN_CB_DEFINE(conn_callbacks) = { // Define a structure of Bluetooth connection callbacks.
	.connected    = connected, // Assign the 'connected' function to handle Bluetooth connection events.
	.disconnected = disconnected, // Assign the 'disconnected' function to handle Bluetooth disconnection events.
#ifdef CONFIG_BT_NUS_SECURITY_ENABLED // Check if Bluetooth NUS security is enabled through configuration.
	.security_changed = security_changed, // Assign the 'security_changed' function to handle Bluetooth security level change events.
#endif 
// End of conditional compilation block for CONFIG_BT_NUS_SECURITY_ENABLED.
};

#if defined(CONFIG_BT_NUS_SECURITY_ENABLED) // Check if Bluetooth NUS security is enabled through configuration.
static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey) // Define a static function named 'auth_passkey_display' to display the passkey for Bluetooth pairing.
{
	char addr[BT_ADDR_LE_STR_LEN]; // Declare a character array to store the Bluetooth address in string format.

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address of the device to a string and store it in 'addr'.

	LOG_INF("Passkey for %s: %06u", addr, passkey); // Log a message displaying the passkey required for pairing, along with the device's Bluetooth address.
}


// Define a static function to handle the passkey confirmation during Bluetooth pairing.
static void auth_passkey_confirm(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN]; // Declare a character array to store the Bluetooth address in string format.

	auth_conn = bt_conn_ref(conn); // Store a reference to the connection object for later use, increasing its reference count.

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address of the device to a string and store it in 'addr'.

	LOG_INF("Passkey for %s: %06u", addr, passkey); // Log the passkey for the user to confirm, along with the device's Bluetooth address.
	LOG_INF("Press Button 1 to confirm, Button 2 to reject."); // Instruct the user to press a button to confirm or reject the passkey.
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


// Define a structure for Bluetooth connection authentication callbacks.
static struct bt_conn_auth_cb conn_auth_callbacks = {
    .passkey_display = auth_passkey_display, // Callback for displaying passkey.
    .passkey_confirm = auth_passkey_confirm, // Callback for confirming passkey.
    .cancel = auth_cancel, // Callback for handling cancellation of pairing.
};

// Define a structure for Bluetooth connection authentication information callbacks.
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete, // Callback for handling completion of pairing.
    .pairing_failed = pairing_failed // Callback for handling failure of pairing.
};
#else
// Define empty structures for Bluetooth connection callbacks if the condition is not met.
static struct bt_conn_auth_cb conn_auth_callbacks;
static struct bt_conn_auth_info_cb conn_auth_info_callbacks;
#endif

// Define a function to handle receiving data over Bluetooth.
static void bt_receive_cb(struct bt_conn *conn, const uint8_t *const data,
              uint16_t len)
{
    int err; // Variable to store error codes.
    char addr[BT_ADDR_LE_STR_LEN] = {0}; // Array to store the Bluetooth address as a string.

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, ARRAY_SIZE(addr)); // Convert the Bluetooth address to a string.

    LOG_INF("Received data from: %s", addr); // Log the address of the device from which data was received.

    // Loop through the received data and process it in chunks.
    for (uint16_t pos = 0; pos != len;) {
        struct uart_data_t *tx = k_malloc(sizeof(*tx)); // Dynamically allocate memory for a UART data structure.

        if (!tx) { // Check if memory allocation failed.
            LOG_WRN("Not able to allocate UART send data buffer"); // Log a warning if memory allocation failed.
            return; // Exit the function if memory allocation failed.
        }

        // Calculate the size of data to be sent, leaving space for a potential LF character.
        size_t tx_data_size = sizeof(tx->data) - 1;

        // Determine the length of data to be copied to the UART buffer.
        if ((len - pos) > tx_data_size) {
            tx->len = tx_data_size;
        } else {
            tx->len = (len - pos);
        }

        memcpy(tx->data, &data[pos], tx->len); // Copy the data to the UART buffer.

        pos += tx->len; // Update the position in the received data buffer.

        // Append a LF character if the last received character was a CR and we're at the end of the data.
        if ((pos == len) && (data[len - 1] == '\r')) {
            tx->data[tx->len] = '\n';
            tx->len++;
        }

        err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS); // Attempt to send the data over UART.
        if (err) {
            k_fifo_put(&fifo_uart_tx_data, tx); // If sending fails, put the data in a FIFO for later transmission.
        }
    }
}
static struct bt_nus_cb nus_cb = {
	.received = bt_receive_cb,
};

// Define a function to enter an error state, turning off all LEDs and entering an infinite loop.
void error(void)
{
    dk_set_leds_state(DK_ALL_LEDS_MSK, DK_NO_LEDS_MSK); // Turn off all LEDs.

    while (true) { // Enter an infinite loop.
        k_sleep(K_MSEC(1000)); // Sleep for 1000 milliseconds (1 second) in each iteration of the loop.
    }
}

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED // Check if Bluetooth NUS security is enabled through configuration.
// Define a static function to reply to numeric comparison authentication.
static void num_comp_reply(bool accept)
{
    if (accept) { // If the numeric comparison is accepted.
        bt_conn_auth_passkey_confirm(auth_conn); // Confirm the passkey for the Bluetooth connection.
        LOG_INF("Numeric Match, conn %p", (void *)auth_conn); // Log that the numeric comparison was accepted.
    } else { // If the numeric comparison is rejected.
        bt_conn_auth_cancel(auth_conn); // Cancel the authentication for the Bluetooth connection.
        LOG_INF("Numeric Reject, conn %p", (void *)auth_conn); // Log that the numeric comparison was rejected.
    }

    bt_conn_unref(auth_conn); // Release the reference to the Bluetooth connection.
    auth_conn = NULL; // Set the global connection pointer to NULL, indicating no active authentication connection.
}

// Define a function to handle button state changes.
void button_changed(uint32_t button_state, uint32_t has_changed)
{
    uint32_t buttons = button_state & has_changed; // Determine which buttons have changed state.

    if (auth_conn) { // If there is an ongoing authentication connection.
        if (buttons & KEY_PASSKEY_ACCEPT) { // If the button to accept the passkey is pressed.
            num_comp_reply(true); // Reply positively to the numeric comparison.
        }

        if (buttons & KEY_PASSKEY_REJECT) { // If the button to reject the passkey is pressed.
            num_comp_reply(false); // Reply negatively to the numeric comparison.
        }
    }
}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

static void configure_gpio(void)
{
	int err;

#ifdef CONFIG_BT_NUS_SECURITY_ENABLED
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("Cannot init buttons (err: %d)", err);
	}
#endif /* CONFIG_BT_NUS_SECURITY_ENABLED */

	err = dk_leds_init();
	if (err) {
		LOG_ERR("Cannot init LEDs (err: %d)", err);
	}
}

int main(void)
{
	int blink_status = 0;
	int err = 0;

	configure_gpio();

	// err = uart_init();
	// if (err) {
	// 	error();
	// }

	if (IS_ENABLED(CONFIG_BT_NUS_SECURITY_ENABLED)) {
		err = bt_conn_auth_cb_register(&conn_auth_callbacks);
		if (err) {
			printk("Failed to register authorization callbacks.\n");
			return 0;
		}

		err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
		if (err) {
			printk("Failed to register authorization info callbacks.\n");
			return 0;
		}
	}

	err = bt_enable(NULL);
	if (err) {
		error();
	}

	LOG_INF("Bluetooth initialized");

	k_sem_give(&ble_init_ok);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	// err = bt_nus_init(&nus_cb);
	// if (err) {
	// 	LOG_ERR("Failed to initialize UART service (err: %d)", err);
	// 	return 0;
	// }

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

/*No longer receiving data from uart, so this function is redundant and will be rewritten using a cutsom BLE service*/

// void ble_write_thread(void)
// {
// 	/* Don't go any further until BLE is initialized */
// 	k_sem_take(&ble_init_ok, K_FOREVER);
// 	struct uart_data_t nus_data = {
// 		.len = 0,
// 	};

// 	for (;;) {
//         /* Wait indefinitely for data to be sent over bluetooth */
//         struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data,
//                              K_FOREVER);

		
	

//         /* Calculate payload length to be the minimum of remaining space in nus_data.data and buf->len */
//         int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
//         int loc = 0; // Location offset within buf->data

//         while (plen > 0) {
//             /* Copy data from UART buffer to BLE buffer */
//             memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
//             nus_data.len += plen; // Update length of data in BLE buffer
//             loc += plen; // Move location offset

// 			if (nus_data.data[nus_data.len - 1] == 'V') {
//                 /* Call the function to read voltage value */
//                 int voltage = pwr_measure_demo(); // Call your modified voltage reading function

//                 /* Convert the voltage value to a string */
//                 char voltage_str[10];
//                 snprintf(voltage_str, sizeof(voltage_str), "%d", voltage);

//                 /* Send the voltage value back over BLE using NUS */
//                 if (bt_nus_send(NULL, voltage_str, strlen(voltage_str))) {
//                     LOG_WRN("Failed to send voltage data over BLE connection");
//                 }
//                 nus_data.len = 0; // Reset BLE buffer length for next data batch
//             } 

//             /* Check if BLE buffer is full or if last character is newline or carriage return */
//             if (nus_data.len >= sizeof(nus_data.data) ||
//                (nus_data.data[nus_data.len - 1] == '\n') ||
//                (nus_data.data[nus_data.len - 1] == '\r')) {
//                 /* Attempt to send data over BLE. Log warning if failed */
//                 if (bt_nus_send(NULL, nus_data.data, nus_data.len)) { 
//                     LOG_WRN("Failed to send data over BLE connection");
//                 }
//                 nus_data.len = 0; // Reset BLE buffer length for next data batch
//             }

//             /* Calculate payload length for next iteration, considering any remaining data */
//             plen = MIN(sizeof(nus_data.data), buf->len - loc);
//         }

//         /* Free the allocated memory for buf after processing */
//         k_free(buf);
//     }
// }

/* Define a thread for BLE data writing with specified stack size, priority, and no arguments */
K_THREAD_DEFINE(ble_write_thread_id, STACKSIZE, ble_write_thread, NULL, NULL,
        NULL, PRIORITY, 0, 0);



