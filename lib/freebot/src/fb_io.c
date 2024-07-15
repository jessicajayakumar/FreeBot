/**
 * @file fb_io.c
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief FreeBot led & button API
 * @version 0.1
 * @date 2024-03-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <zephyr/drivers/gpio.h>

#include "fb_io.h"

// -----------------------------------------------------------------------------
// FreeBot IO pins
// -----------------------------------------------------------------------------

static const struct gpio_dt_spec led_d15 = GPIO_DT_SPEC_GET(DT_NODELABEL(d15), gpios);
static const struct gpio_dt_spec led_d16 = GPIO_DT_SPEC_GET(DT_NODELABEL(d16), gpios);
static const struct gpio_dt_spec btn_sw2 = GPIO_DT_SPEC_GET(DT_NODELABEL(sw2), gpios);

// -----------------------------------------------------------------------------
// FreeBot IO private functions
// -----------------------------------------------------------------------------

/**
 * @brief Get the led's `gpio_dt_spec*` from `led_id`
 *
 * @param id
 * @return Led's `gpio_dt_spec` pointer. `NULL` for unknown led IDs.
 */
const struct gpio_dt_spec *get_led_from_id(uint8_t id)
{
    switch (id)
    {
    case D15:
        return &led_d15;
    case D16:
        return &led_d16;
    default:
        return NULL;
    }
}

// -----------------------------------------------------------------------------
// FreeBot IO API (public) functions
// -----------------------------------------------------------------------------

int fb_io_init(void)
{
    int err = 0;

    // Check if the GPIO pins for the LEDs and button are ready
    err |= !gpio_is_ready_dt(&led_d15);
    err |= !gpio_is_ready_dt(&led_d16);
    err |= !gpio_is_ready_dt(&btn_sw2);

    // If any of the GPIO pins are not ready, return the error
    if (err)
    {
        return err;
    }

    // Configure the GPIO pins for the LEDs and button as output and input respectively
    err |= gpio_pin_configure_dt(&led_d15, GPIO_OUTPUT | GPIO_INPUT);
    err |= gpio_pin_configure_dt(&led_d16, GPIO_OUTPUT | GPIO_INPUT);
    err |= gpio_pin_configure_dt(&btn_sw2, GPIO_INPUT);

    // If there is an error in configuring the GPIO pins, return the error
    if (err)
    {
        return err;
    }

    // Clear the LEDs D15 and D16
    fb_clear_led(D15);
    fb_clear_led(D16);
    return 0;
}

// Get the status of the specified LED
int fb_led_status(uint8_t led_id)
{
    const struct gpio_dt_spec *led = get_led_from_id(led_id);
    if (led != NULL)
    {
        return gpio_pin_get_dt(led);
    } else {
        return E_LED_UNK;
    }
}

// Set the specified LED
int fb_set_led(uint8_t led_id)
{
    const struct gpio_dt_spec *led = get_led_from_id(led_id);
    if (led != NULL)
    {
        gpio_pin_set_dt(led, 1);
        return 0;
    } else {
        return E_LED_UNK;
    }
}

// Clear the specified LED
int fb_clear_led(uint8_t led_id)
{
    const struct gpio_dt_spec *led = get_led_from_id(led_id);
    if (led != NULL)
    {
        gpio_pin_set_dt(led, 0);
        return 0;
    } else {
        return E_LED_UNK;
    }
}

// Toggle the specified LED
int fb_toggle_led(uint8_t led_id)
{
    const struct gpio_dt_spec *led = get_led_from_id(led_id);
    if (led != NULL)
    {
        gpio_pin_toggle_dt(led);
        return 0;
    } else {
        return E_LED_UNK;
    }
}

// Read the state of the button
int fb_read_btn()
{
    return gpio_pin_get_dt(&btn_sw2);
}
