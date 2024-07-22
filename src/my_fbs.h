
#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/bluetooth/gatt.h>

#include "freebot.h"

#define BT_UUID_16_FBS_VAL 0x1560
#define BT_UUID_16_FBS_VOLT_VAL 0x1561
#define BT_UUID_16_FBS BT_UUID_DECLARE_16(BT_UUID_16_FBS_VAL)
#define BT_UUID_16_FBS_AVGS BT_UUID_DECLARE_16(BT_UUID_16_FBS_VOLT_VAL)

/** @brief Voltage data format */
typedef uint16_t fbcs_v_t;


