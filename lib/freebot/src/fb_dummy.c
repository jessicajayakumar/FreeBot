/**
 * @file fb_dummy.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Dummy API implementation in order ease development on other nRF52840 boards.
 * @version 0.1
 * @date 2024-04-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <zephyr/logging/log.h>

#include "fb_io.h"
#include "fb_pwr.h"
#include "fb_motor.h"

LOG_MODULE_REGISTER(fb_dummy_api, LOG_LEVEL_DBG);

// -----------------------------------------------------------------------------
// FreeBot IO API
// -----------------------------------------------------------------------------

int fb_io_init(void)
{
    LOG_INF("Inialized FreeBot IO API");
    return 0;
}

void fb_set_led(uint8_t led_id)
{
    switch (led_id)
    {
    case D15:
        LOG_INF("D15 turned on");
        break;
    case D16:
        LOG_INF("D16 turned on");
        break;
    default:
        LOG_ERR("Could not turn LED on");
        break;
    }
}

void fb_clear_led(uint8_t led_id)
{
    switch (led_id)
    {
    case D15:
        LOG_INF("D15 turned off");
        break;
    case D16:
        LOG_INF("D16 turned off");
        break;
    default:
        LOG_ERR("Could not turn LED off");
        break;
    }
}

void fb_toggle_led(uint8_t led_id)
{
    switch (led_id)
    {
    case D15:
        LOG_INF("D15 toggled");
        break;
    case D16:
        LOG_INF("D16 toggled");
        break;
    default:
        LOG_ERR("Could not toggle LED");
        break;
    }
}

int fb_read_btn()
{
    LOG_WRN("Button read requested, always returning '1'");
    return 1;
}

// -----------------------------------------------------------------------------
// FreeBot Power API
// -----------------------------------------------------------------------------

int fb_pwr_init(void)
{
    LOG_INF("Inialized FreeBot power API");
    return 0;
}

int fb_v_measure(void)
{
    LOG_WRN("Voltage measurement requested, always returning 60mv");
    return 60;

}

int fb_v_measure_select(int v_sel)
{
    switch (v_sel)
    {
    case V_CAP:
        LOG_INF("Selected Vcap for measuring");
        break;
    case V_MOTOR:
        LOG_INF("Selected Vmotor for measuring");
        break;
    case V_IN:
        LOG_INF("Selected Vin for measuring");
        break;
    default:
        LOG_ERR("Could not select voltage for measuring");
        break;
    }
}

// -----------------------------------------------------------------------------
// FreeBot Motor API
// -----------------------------------------------------------------------------

int fb_motor_init(void)
{
    LOG_INF("Inialized FreeBot motor API");
    return 0;
}

void fb_stop(void)
{
    LOG_INF("Robot stopped");
}

void fb_straight_forw(void)
{
    LOG_INF("Robot going forward");
}

void fb_straight_back(void)
{
    LOG_INF("Robot going backward");
}

void fb_side_right(void)
{
    LOG_INF("Robot going right");
}

void fb_side_left(void)
{
    LOG_INF("Robot going left");
}

void fb_side_d45(void)
{
    LOG_INF("Robot going 45°");
}

void fb_side_d135(void)
{
    LOG_INF("Robot going 135°");
}

void fb_side_d225(void)
{
    LOG_INF("Robot going 225°");
}

void fb_side_d315(void)
{
    LOG_INF("Robot going 315°");
}

void fb_rotate_cw(void)
{
    LOG_INF("Robot rotating clockwise");
}

void fb_rotate_ccw(void)
{
    LOG_INF("Robot rotating counterclockwise");
}

void fb_get_motor_speed(fb_motor_speed_t *speeds)
{
    speeds->front_left = 60;
    speeds->front_right = 60;
    speeds->back_left = 60;
    speeds->back_right = 60;
    LOG_WRN("Motor speeds requested, always returning 60 rpm");
}

void fb_get_motor_angle(fb_motor_angle_t *angles)
{
    angles->front_left = 90;
    angles->front_right = 90;
    angles->back_left = 90;
    angles->back_right = 90;
    LOG_WRN("Motor angles requested, always returning 90°");
}
