/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Nordic UART Service Client sample
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <bluetooth/services/nus.h>
#include <bluetooth/services/nus_client.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>

#include <zephyr/settings/settings.h>

#include <zephyr/drivers/uart.h>

#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>

#define LOG_MODULE_NAME central_uart
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define UART_BUF_SIZE 20 /**< The size of the UART buffer. */

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK /**< The mask for the accept passkey button. */
#define KEY_PASSKEY_REJECT DK_BTN2_MSK /**< The mask for the reject passkey button. */

#define CON_STATUS_LED DK_LED2 /**< The LED pin for the connection status. */

#define RUN_STATUS_LED DK_LED1 /**< The LED pin for the run status. */
#define RUN_LED_BLINK_INTERVAL 1000 /**< The interval for blinking the run status LED. */

#define NUS_WRITE_TIMEOUT K_MSEC(150) /**< The timeout for writing to the NUS service. */
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50) /**< The delay for waiting for the UART buffer. */
#define UART_BUF_SIZE 20 /**< The size of the UART buffer. */

#define KEY_PASSKEY_ACCEPT DK_BTN1_MSK /**< The mask for the accept passkey button. */
#define KEY_PASSKEY_REJECT DK_BTN2_MSK /**< The mask for the reject passkey button. */

#define CON_STATUS_LED DK_LED2 /**< The LED pin for the connection status. */

#define RUN_STATUS_LED DK_LED1 /**< The LED pin for the run status. */
#define RUN_LED_BLINK_INTERVAL 1000

#define NUS_WRITE_TIMEOUT K_MSEC(150)
#define UART_WAIT_FOR_BUF_DELAY K_MSEC(50)
#define UART_RX_TIMEOUT 50000 /* Wait for RX complete event time in microseconds. */

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(nordic_nus_uart));
static struct k_work_delayable uart_work;

K_SEM_DEFINE(nus_write_sem, 0, 1);

struct uart_data_t { // Define a structure for UART data
    void *fifo_reserved; // Pointer reserved for FIFO use
    uint8_t  data[UART_BUF_SIZE]; // Buffer for UART data, size defined by UART_BUF_SIZE
    uint16_t len; // Length of the data in the buffer
};

static K_FIFO_DEFINE(fifo_uart_tx_data); // Define a FIFO for UART transmission data
static K_FIFO_DEFINE(fifo_uart_rx_data); // Define a FIFO for UART reception data

static struct bt_conn *default_conn; // Pointer to a Bluetooth connection structure, used as the default connection
static struct bt_nus_client nus_client; // Instance of a Bluetooth NUS (Nordic UART Service) client structure

static void ble_data_sent(struct bt_nus_client *nus, uint8_t err,
                    const uint8_t *const data, uint16_t len)
{ // Callback function for when data is sent over BLE
    ARG_UNUSED(nus); // Macro to avoid compiler warnings for unused parameters
    ARG_UNUSED(data); // Macro to avoid compiler warnings for unused parameters
    ARG_UNUSED(len); // Macro to avoid compiler warnings for unused parameters

    k_sem_give(&nus_write_sem); // Signal that the NUS write operation is complete

    if (err) { // If there was an error in sending
        LOG_WRN("ATT error code: 0x%02X", err); // Log a warning with the ATT error code
    }
}


static uint8_t ble_data_received(struct bt_nus_client *nus,
                        const uint8_t *data, uint16_t len)
{ // Function to handle data received over BLE
    ARG_UNUSED(nus); // Macro to avoid compiler warnings for unused parameters

    int err; // Variable to store error codes

    for (uint16_t pos = 0; pos != len;) { // Loop through the received data
        struct uart_data_t *tx = k_malloc(sizeof(*tx)); // Allocate memory for UART data structure

        if (!tx) { // If memory allocation failed
            LOG_WRN("Not able to allocate UART send data buffer"); // Log a warning
            return BT_GATT_ITER_CONTINUE; // Continue to the next GATT attribute
        }

        /* Keep the last byte of TX buffer for potential LF char. */
        size_t tx_data_size = sizeof(tx->data) - 1; // Calculate the maximum data size, leaving space for LF character

        if ((len - pos) > tx_data_size) { // If remaining data exceeds buffer size
            tx->len = tx_data_size; // Set length to maximum buffer size minus space for LF
        } else { // If remaining data fits in buffer
            tx->len = (len - pos); // Set length to remaining data size
        }

        memcpy(tx->data, &data[pos], tx->len); // Copy the data to the UART buffer

        pos += tx->len; // Update position in received data

        /* Append the LF character when the CR character triggered
         * transmission from the peer.
         */
        if ((pos == len) && (data[len - 1] == '\r')) { // If the last received character is CR
            tx->data[tx->len] = '\n'; // Append LF to the data
            tx->len++; // Increment data length
        }

        err = uart_tx(uart, tx->data, tx->len, SYS_FOREVER_MS); // Transmit the data over UART
        if (err) { // If transmission failed
            k_fifo_put(&fifo_uart_tx_data, tx); // Put the data in the UART TX FIFO
        }
    }

    return BT_GATT_ITER_CONTINUE; // Continue to the next GATT attribute
}

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(dev); // Macro to suppress unused parameter warning for 'dev'

    static size_t aborted_len; // Static variable to keep track of length of aborted transmission
    struct uart_data_t *buf; // Pointer to a UART data structure
    static uint8_t *aborted_buf; // Static pointer to keep track of aborted transmission buffer
    static bool disable_req; // Static boolean to manage request disable state

    switch (evt->type) { // Switch on the type of UART event
    case UART_TX_DONE: // Case when UART transmission is done
        LOG_DBG("UART_TX_DONE"); // Log debug message for UART transmission done
        if ((evt->data.tx.len == 0) || // Check if transmitted length is 0 or buffer is NULL
            (!evt->data.tx.buf)) {
            return; // Exit the function if true
        }

        if (aborted_buf) { // Check if there is an aborted buffer
            buf = CONTAINER_OF(aborted_buf, struct uart_data_t, // Retrieve the uart_data_t struct from the aborted buffer
                       data[0]);
            aborted_buf = NULL; // Reset aborted buffer pointer
            aborted_len = 0; // Reset aborted length
        } else {
            buf = CONTAINER_OF(evt->data.tx.buf, // Retrieve the uart_data_t struct from the transmission buffer
                       struct uart_data_t,
                       data[0]);
        }

        k_free(buf); // Free the buffer

        buf = k_fifo_get(&fifo_uart_tx_data, K_NO_WAIT); // Get next buffer from FIFO without waiting
        if (!buf) {
            return; // Exit if no buffer is available
        }

        if (uart_tx(uart, buf->data, buf->len, SYS_FOREVER_MS)) { // Attempt to transmit the next buffer
            LOG_WRN("Failed to send data over UART"); // Log warning if transmission fails
        }

        break;

    case UART_RX_RDY: // Case when UART is ready to receive
        LOG_DBG("UART_RX_RDY"); // Log debug message for UART ready to receive
        buf = CONTAINER_OF(evt->data.rx.buf, struct uart_data_t, data[0]); // Retrieve the uart_data_t struct from the receive buffer
        buf->len += evt->data.rx.len; // Update buffer length with received length

        if (disable_req) { // Check if disable request is active
            return; // Exit if true
        }

        if ((evt->data.rx.buf[buf->len - 1] == '\n') || // Check if last received character is newline or carriage return
            (evt->data.rx.buf[buf->len - 1] == '\r')) {
            disable_req = true; // Set disable request
            uart_rx_disable(uart); // Disable UART receive
        }

        break;

    case UART_RX_DISABLED: // Case when UART receive is disabled
        LOG_DBG("UART_RX_DISABLED"); // Log debug message for UART receive disabled
        disable_req = false; // Reset disable request

        buf = k_malloc(sizeof(*buf)); // Allocate memory for new buffer
        if (buf) {
            buf->len = 0; // Initialize buffer length
        } else {
            LOG_WRN("Not able to allocate UART receive buffer"); // Log warning if allocation fails
            k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY); // Reschedule work with delay if buffer allocation fails
            return;
        }

        uart_rx_enable(uart, buf->data, sizeof(buf->data), // Enable UART receive with new buffer
                   UART_RX_TIMEOUT);

        break;

    case UART_RX_BUF_REQUEST: // Case when UART requests a receive buffer
        LOG_DBG("UART_RX_BUF_REQUEST"); // Log debug message for UART receive buffer request
        buf = k_malloc(sizeof(*buf)); // Allocate memory for new buffer
        if (buf) {
            buf->len = 0; // Initialize buffer length
            uart_rx_buf_rsp(uart, buf->data, sizeof(buf->data)); // Respond with the new buffer
        } else {
            LOG_WRN("Not able to allocate UART receive buffer"); // Log warning if allocation fails
        }

        break;

	case UART_RX_BUF_RELEASED: // Case when a UART receive buffer is released
    LOG_DBG("UART_RX_BUF_RELEASED"); // Log debug message for UART receive buffer released
    buf = CONTAINER_OF(evt->data.rx_buf.buf, struct uart_data_t, data[0]); // Retrieve the uart_data_t struct from the released buffer

    if (buf->len > 0) { // Check if the buffer contains data
        k_fifo_put(&fifo_uart_rx_data, buf); // Put the buffer into the receive FIFO if it contains data
    } else {
        k_free(buf); // Free the buffer if it's empty
    }

    break;

	case UART_TX_ABORTED: // Case when UART transmission is aborted
		LOG_DBG("UART_TX_ABORTED"); // Log debug message for UART transmission aborted
		if (!aborted_buf) { // Check if there's no previously aborted buffer
			aborted_buf = (uint8_t *)evt->data.tx.buf; // Set aborted_buf to the current buffer if none was previously set
		}

		aborted_len += evt->data.tx.len; // Increment the total length of aborted transmission
		buf = CONTAINER_OF(aborted_buf, struct uart_data_t, data[0]); // Retrieve the uart_data_t struct from the aborted buffer

		uart_tx(uart, &buf->data[aborted_len], buf->len - aborted_len, SYS_FOREVER_MS); // Attempt to retransmit the remainder of the aborted buffer

		break;

	default: // Default case for unhandled events
		break;
	}
}

// Function to handle UART work
static void uart_work_handler(struct k_work *item)
{
    struct uart_data_t *buf; // Define a pointer to uart_data_t structure

    buf = k_malloc(sizeof(*buf)); // Allocate memory for UART data buffer
    if (buf) { // Check if memory allocation was successful
        buf->len = 0; // Initialize buffer length to 0
    } else { // If memory allocation failed
        LOG_WRN("Not able to allocate UART receive buffer"); // Log a warning message
        k_work_reschedule(&uart_work, UART_WAIT_FOR_BUF_DELAY); // Reschedule the work with a delay
        return; // Exit the function
    }

    uart_rx_enable(uart, buf->data, sizeof(buf->data), UART_RX_TIMEOUT); // Enable UART receive with the allocated buffer
}

// Function to initialize UART
static int uart_init(void)
{
    int err; // Variable to store error status
    struct uart_data_t *rx; // Define a pointer to uart_data_t structure for receive buffer

    if (!device_is_ready(uart)) { // Check if UART device is ready
        LOG_ERR("UART device not ready"); // Log an error message if device is not ready
        return -ENODEV; // Return error code for device not found
    }

    rx = k_malloc(sizeof(*rx)); // Allocate memory for receive buffer
    if (rx) { // Check if memory allocation was successful
        rx->len = 0; // Initialize receive buffer length to 0
    } else { // If memory allocation failed
        return -ENOMEM; // Return error code for out of memory
    }

    k_work_init_delayable(&uart_work, uart_work_handler); // Initialize a delayable work item for UART work handler

    err = uart_callback_set(uart, uart_cb, NULL); // Set UART callback
    if (err) { // Check if setting callback failed
        return err; // Return the error code
    }

    return uart_rx_enable(uart, rx->data, sizeof(rx->data), UART_RX_TIMEOUT); // Enable UART receive with the allocated buffer and return the status
}

// Function called when service discovery is complete
static void discovery_complete(struct bt_gatt_dm *dm,
                   void *context)
{
    struct bt_nus_client *nus = context; // Cast context to bt_nus_client pointer
    LOG_INF("Service discovery completed"); // Log information message indicating service discovery completion

    bt_gatt_dm_data_print(dm); // Print the GATT discovery data

    bt_nus_handles_assign(dm, nus); // Assign handles to the NUS client instance
    bt_nus_subscribe_receive(nus); // Subscribe to NUS receive characteristic

    bt_gatt_dm_data_release(dm); // Release the GATT discovery data
}

// Callback function for when a specific service is not found during discovery
static void discovery_service_not_found(struct bt_conn *conn, void *context)
{
    LOG_INF("Service not found"); // Log an informational message indicating the service was not found
}

// Callback function for when an error occurs during GATT database discovery
static void discovery_error(struct bt_conn *conn, int err, void *context)
{
    LOG_WRN("Error while discovering GATT database: (%d)", err); // Log a warning with the error code
}

// Struct to define callbacks for GATT discovery events
struct bt_gatt_dm_cb discovery_cb = {
    .completed         = discovery_complete, // Callback when discovery is complete
    .service_not_found = discovery_service_not_found, // Callback when service is not found
    .error_found       = discovery_error, // Callback when an error is found
};

// Function to start GATT discovery on a Bluetooth connection
static void gatt_discover(struct bt_conn *conn)
{
    int err; // Variable to store error status

    if (conn != default_conn) { // Check if the connection is not the default connection
        return; // Exit the function if true
    }

    // Start GATT discovery with the specified UUID for NUS service
    err = bt_gatt_dm_start(conn, BT_UUID_NUS_SERVICE, &discovery_cb, &nus_client);
    if (err) { // Check if there was an error starting discovery
        LOG_ERR("could not start the discovery procedure, error code: %d", err); // Log an error message with the error code
    }
}

// Callback function for MTU exchange completion
static void exchange_func(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
    if (!err) { // Check if there was no error
        LOG_INF("MTU exchange done"); // Log an informational message indicating MTU exchange completion
    } else { // If there was an error
        LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err); // Log a warning with the error code
    }
}

static void connected(struct bt_conn *conn, uint8_t conn_err) // Define the connected callback function
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer to hold the string representation of the Bluetooth address
    int err; // Variable to hold error codes

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert the Bluetooth address to a string

    if (conn_err) { // Check if there was an error during connection
        LOG_INF("Failed to connect to %s (%d)", addr, conn_err); // Log the failure to connect

        if (default_conn == conn) { // Check if the failed connection is the default connection
            bt_conn_unref(default_conn); // Release the reference to the connection object
            default_conn = NULL; // Clear the default connection

            err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE); // Start Bluetooth scanning again
            if (err) { // Check if scanning failed to start
                LOG_ERR("Scanning failed to start (err %d)", err); // Log the failure to start scanning
            }
        }

        return; // Exit the function due to connection error
    }

    LOG_INF("Connected: %s", addr); // Log successful connection

	dk_set_led_on(CON_STATUS_LED);

    static struct bt_gatt_exchange_params exchange_params; // Static variable for MTU exchange parameters

    exchange_params.func = exchange_func; // Set the callback function for MTU exchange completion
    err = bt_gatt_exchange_mtu(conn, &exchange_params); // Initiate MTU exchange
    if (err) { // Check if MTU exchange initiation failed
        LOG_WRN("MTU exchange failed (err %d)", err); // Log the failure to initiate MTU exchange
    }

    err = bt_conn_set_security(conn, BT_SECURITY_L2); // Set the security level of the connection
    if (err) { // Check if setting security level failed
        LOG_WRN("Failed to set security: %d", err); // Log the failure to set security

        gatt_discover(conn); // Proceed to discover GATT services
    }

    err = bt_scan_stop(); // Attempt to stop Bluetooth scanning
    if ((!err) && (err != -EALREADY)) { // Check if stopping scanning failed and the error is not -EALREADY (scanning not started)
        LOG_ERR("Stop LE scan failed (err %d)", err); // Log the failure to stop scanning
    }
}


// Callback function for when a Bluetooth connection is disconnected
static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer for Bluetooth address string representation
    int err; // Variable for error codes

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert Bluetooth address to string

    LOG_INF("Disconnected: %s (reason %u)", addr, reason); // Log disconnection with reason

	dk_set_led_off(CON_STATUS_LED); 

    if (default_conn != conn) { // Check if the disconnected connection is not the default connection
        return; // Exit if true
    }

    bt_conn_unref(default_conn); // Release reference to the default connection object
    default_conn = NULL; // Reset default connection to NULL

    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE); // Start Bluetooth scanning again
    if (err) { // Check if scanning failed to start
        LOG_ERR("Scanning failed to start (err %d)", err); // Log scanning start failure
    }
}

// Callback function for when the security level of a Bluetooth connection changes
static void security_changed(struct bt_conn *conn, bt_security_t level,
                 enum bt_security_err err)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer for Bluetooth address string representation

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert Bluetooth address to string

    if (!err) { // Check if there was no error changing security level
        LOG_INF("Security changed: %s level %u", addr, level); // Log successful security level change
    } else { // If there was an error changing security level
        LOG_WRN("Security failed: %s level %u err %d", addr, level, err); // Log security level change failure
    }

    gatt_discover(conn); // Start GATT discovery on the connection
}

// Define Bluetooth connection callbacks
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected, // Callback for connection established
    .disconnected = disconnected, // Callback for disconnection
    .security_changed = security_changed // Callback for security level change
};

// Callback function for when a device matches the scan filter criteria
static void scan_filter_match(struct bt_scan_device_info *device_info,
                  struct bt_scan_filter_match *filter_match,
                  bool connectable)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer for Bluetooth address string representation

    bt_addr_le_to_str(device_info->recv_info->addr, addr, sizeof(addr)); // Convert Bluetooth address to string

    LOG_INF("Filters matched. Address: %s connectable: %d", addr, connectable); // Log filter match with device address and connectability
}

// Callback function for when connecting to a device fails
static void scan_connecting_error(struct bt_scan_device_info *device_info)
{
    LOG_WRN("Connecting failed"); // Log a warning that connecting failed
}

// Callback function for when a connection is being established
static void scan_connecting(struct bt_scan_device_info *device_info,
                struct bt_conn *conn)
{
    default_conn = bt_conn_ref(conn); // Store a reference to the connection object in default_conn
}

// Initializes the NUS (Nordic UART Service) client
static int nus_client_init(void)
{
    int err; // Variable to store error codes
    // Initialize parameters for NUS client, setting callbacks for data received and sent
    struct bt_nus_client_init_param init = {
        .cb = {
            .received = ble_data_received, // Callback for when data is received
            .sent = ble_data_sent, // Callback for when data is sent
        }
    };

    err = bt_nus_client_init(&nus_client, &init); // Initialize the NUS client with the specified parameters
    if (err) { // Check if initialization failed
        LOG_ERR("NUS Client initialization failed (err %d)", err); // Log an error message with the error code
        return err; // Return the error code
    }

    LOG_INF("NUS Client module initialized"); // Log that NUS Client module was successfully initialized
    return err; // Return the error code (0 if successful)
}

// Initialize scan callbacks with specific functions for events
BT_SCAN_CB_INIT(scan_cb, scan_filter_match, NULL,
        scan_connecting_error, scan_connecting);

// Initializes the scanning module
static int scan_init(void)
{
    int err; // Variable to store error codes
    // Initialize scanning parameters, setting connect_if_match to automatically connect if filters match
    struct bt_scan_init_param scan_init = {
        .connect_if_match = 1,
    };

    bt_scan_init(&scan_init); // Initialize scanning with the specified parameters
    bt_scan_cb_register(&scan_cb); // Register the scan callbacks

    // Add a filter for the NUS service UUID
    err = bt_scan_filter_add(BT_SCAN_FILTER_TYPE_UUID, BT_UUID_NUS_SERVICE);
    if (err) { // Check if adding the filter failed
        LOG_ERR("Scanning filters cannot be set (err %d)", err); // Log an error message with the error code
        return err; // Return the error code
    }

    // Enable the UUID filter for scanning
    err = bt_scan_filter_enable(BT_SCAN_UUID_FILTER, false);
    if (err) { // Check if enabling the filter failed
        LOG_ERR("Filters cannot be turned on (err %d)", err); // Log an error message with the error code
        return err; // Return the error code
    }

    LOG_INF("Scan module initialized"); // Log that the scan module was successfully initialized
    return err; // Return the error code (0 if successful)
}


// Callback function for when pairing is cancelled
static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer for Bluetooth address string representation

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert Bluetooth address to string

    LOG_INF("Pairing cancelled: %s", addr); // Log that pairing was cancelled with the device address
}

// Callback function for when pairing is complete
static void pairing_complete(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer for Bluetooth address string representation

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert Bluetooth address to string

    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded); // Log that pairing was completed, indicating if bonding occurred
}

// Callback function for when pairing fails
static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN]; // Buffer for Bluetooth address string representation

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr)); // Convert Bluetooth address to string

    LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason); // Log that pairing failed with the device address and reason
}

// Structure to register callbacks related to authentication events
static struct bt_conn_auth_cb conn_auth_callbacks = {
    .cancel = auth_cancel, // Callback for pairing cancellation
};

// Structure to register callbacks for pairing information events
static struct bt_conn_auth_info_cb conn_auth_info_callbacks = {
    .pairing_complete = pairing_complete, // Callback for pairing completion
    .pairing_failed = pairing_failed // Callback for pairing failure
};

// Main function
int main(void)
{
    int err; // Variable for error codes
	int blink_status = 0;

    // Register authorization callbacks
    err = bt_conn_auth_cb_register(&conn_auth_callbacks);
    if (err) {
        LOG_ERR("Failed to register authorization callbacks."); // Log error if registration fails
        return 0; // Exit program
    }

    // Register authorization info callbacks
    err = bt_conn_auth_info_cb_register(&conn_auth_info_callbacks);
    if (err) {
        printk("Failed to register authorization info callbacks.\n"); // Print error if registration fails
        return 0; // Exit program
    }

    // Initialize Bluetooth
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err); // Log error if initialization fails
        return 0; // Exit program
    }
    LOG_INF("Bluetooth initialized"); // Log Bluetooth initialization success

    // Load settings if enabled
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load(); // Load settings
    }

    // Initialize UART
    err = uart_init();
    if (err != 0) {
        LOG_ERR("uart_init failed (err %d)", err); // Log error if UART initialization fails
        return 0; // Exit program
    }

    // Initialize scanning
    err = scan_init();
    if (err != 0) {
        LOG_ERR("scan_init failed (err %d)", err); // Log error if scanning initialization fails
        return 0; // Exit program
    }

    // Initialize NUS client
    err = nus_client_init();
    if (err != 0) {
        LOG_ERR("nus_client_init failed (err %d)", err); // Log error if NUS client initialization fails
        return 0; // Exit program
    }

    printk("Starting Bluetooth Central UART example\n"); // Print start message

    // Start scanning
    err = bt_scan_start(BT_SCAN_TYPE_SCAN_ACTIVE);
    if (err) {
        LOG_ERR("Scanning failed to start (err %d)", err); // Log error if scanning fails to start
        return 0; // Exit program
    }
    LOG_INF("Scanning successfully started"); // Log scanning start success

    // Initialize UART data structure
    struct uart_data_t nus_data = {
        .len = 0, // Initialize length to 0
    };

    // Main loop
    for (;;) {
		dk_set_led(RUN_STATUS_LED, (++blink_status) % 2);
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
        // Wait indefinitely for data from UART
        struct uart_data_t *buf = k_fifo_get(&fifo_uart_rx_data, K_FOREVER);

        // Calculate payload length to be copied
        int plen = MIN(sizeof(nus_data.data) - nus_data.len, buf->len);
        int loc = 0; // Initialize location index

        // Process received UART data
        while (plen > 0) {
            // Copy data from buffer to nus_data
            memcpy(&nus_data.data[nus_data.len], &buf->data[loc], plen);
            nus_data.len += plen; // Update length of nus_data
            loc += plen; // Update location index

            // Check if nus_data is full or if a newline character is received
            if (nus_data.len >= sizeof(nus_data.data) ||
                (nus_data.data[nus_data.len - 1] == '\n') ||
                (nus_data.data[nus_data.len - 1] == '\r')) {
                // Send data over BLE
                err = bt_nus_client_send(&nus_client, nus_data.data, nus_data.len);
                if (err) {
                    LOG_WRN("Failed to send data over BLE connection (err %d)", err);
                }

                // Wait for send completion or timeout
                err = k_sem_take(&nus_write_sem, NUS_WRITE_TIMEOUT);
                if (err) {
                    LOG_WRN("NUS send timeout"); // Log timeout warning
                }

                nus_data.len = 0; // Reset data length
            }

            // Calculate next payload length
            plen = MIN(sizeof(nus_data.data), buf->len - loc);
        }

        k_free(buf); // Free the buffer
    }
}
