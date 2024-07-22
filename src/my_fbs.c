
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "freebot.h"
#include "my_fbs.h"

static fbcs_v_t fbcs_v;


static ssize_t fbcs_v_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_DBG("handle: %u, conn: %p", attr->handle, (void *)conn);

    fbcs_v = (fbcs_v_t)fb_v_measure();

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &fbcs_v, sizeof(fbcs_v));
}