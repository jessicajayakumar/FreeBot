/**
 * @file fb_motor.c
 * @author Mengyao Liu <mengyao.liu@kuleuven.be>
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief FreeBot motor drivers
 * @version 2.0
 * @date 2024-03-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "fb_motor.h"

/** @brief Holds a motor's GPIO handles */
struct motor_gpio
{
    struct gpio_dt_spec *in_a;
    struct gpio_dt_spec *in_b;
    struct gpio_dt_spec *hall_c1;
    struct gpio_dt_spec *hall_c2;
};

/** @brief Period over which motor RPM is calculated */
#define RPM_AVG_PERIOD K_MSEC(20)

#define RPM_THREAD_PRIORITY 7
#define RPM_THREAD_STACKSIZE 512

/** @brief For keeping track of motor timings based on hall sensors */
struct motor_timing
{
    int64_t step_prev;
    int64_t step_delta;
    uint64_t time_prev;
    uint64_t time_delta;
};

// -----------------------------------------------------------------------------
// Objects for motor feedback
// -----------------------------------------------------------------------------

// Keep track of a motor's steps: counts up/down every hall interrupt
static int64_t mfl_steps = 0;
static int64_t mfr_steps = 0;
static int64_t mbl_steps = 0;
static int64_t mbr_steps = 0;

// Callback info for hall interrupts
static struct gpio_callback mfl_hall_cb_data;
static struct gpio_callback mfr_hall_cb_data;
static struct gpio_callback mbl_hall_cb_data;
static struct gpio_callback mbr_hall_cb_data;

// Keep track of each motor's step_delta & time_delta for calculating its speed
static struct motor_timing mfl_timing = {.step_prev = 0, .step_delta = 0, .time_prev = 0, .time_delta = 0};
static struct motor_timing mfr_timing = {.step_prev = 0, .step_delta = 0, .time_prev = 0, .time_delta = 0};
static struct motor_timing mbl_timing = {.step_prev = 0, .step_delta = 0, .time_prev = 0, .time_delta = 0};
static struct motor_timing mbr_timing = {.step_prev = 0, .step_delta = 0, .time_prev = 0, .time_delta = 0};

// -----------------------------------------------------------------------------
// FreeBot Motor control pins
// -----------------------------------------------------------------------------

#define MOTOR_FL DT_NODELABEL(m1)
// Swap in_a and in_b for left-side motors
static const struct gpio_dt_spec mfl_in_a = GPIO_DT_SPEC_GET(MOTOR_FL, in_b_gpios);
static const struct gpio_dt_spec mfl_in_b = GPIO_DT_SPEC_GET(MOTOR_FL, in_a_gpios);
// Swap hall_c1 and hall_c2 for left-side motors
static const struct gpio_dt_spec mfl_hall_c1 = GPIO_DT_SPEC_GET(MOTOR_FL, hall_c2_gpios);
static const struct gpio_dt_spec mfl_hall_c2 = GPIO_DT_SPEC_GET(MOTOR_FL, hall_c1_gpios);
/** @brief Motor @ Front Left */
static const struct motor_gpio mfl = {
    .in_a = &mfl_in_a,
    .in_b = &mfl_in_b,
    .hall_c1 = &mfl_hall_c1,
    .hall_c2 = &mfl_hall_c2,
};

#define MOTOR_FR DT_NODELABEL(m2)
static const struct gpio_dt_spec mfr_in_a = GPIO_DT_SPEC_GET(MOTOR_FR, in_a_gpios);
static const struct gpio_dt_spec mfr_in_b = GPIO_DT_SPEC_GET(MOTOR_FR, in_b_gpios);
static const struct gpio_dt_spec mfr_hall_c1 = GPIO_DT_SPEC_GET(MOTOR_FR, hall_c1_gpios);
static const struct gpio_dt_spec mfr_hall_c2 = GPIO_DT_SPEC_GET(MOTOR_FR, hall_c2_gpios);
/** @brief Motor @ Front Right */
static const struct motor_gpio mfr = {
    .in_a = &mfr_in_a,
    .in_b = &mfr_in_b,
    .hall_c1 = &mfr_hall_c1,
    .hall_c2 = &mfr_hall_c2,
};

#define MOTOR_BL DT_NODELABEL(m3)
// Swap in_a and in_b for left-side motors
static const struct gpio_dt_spec mbl_in_a = GPIO_DT_SPEC_GET(MOTOR_BL, in_b_gpios);
static const struct gpio_dt_spec mbl_in_b = GPIO_DT_SPEC_GET(MOTOR_BL, in_a_gpios);
// Swap hall_c1 and hall_c2 for left-side motors
static const struct gpio_dt_spec mbl_hall_c1 = GPIO_DT_SPEC_GET(MOTOR_BL, hall_c2_gpios);
static const struct gpio_dt_spec mbl_hall_c2 = GPIO_DT_SPEC_GET(MOTOR_BL, hall_c1_gpios);
/** @brief Motor @ Back Left */
static const struct motor_gpio mbl = {
    .in_a = &mbl_in_a,
    .in_b = &mbl_in_b,
    .hall_c1 = &mbl_hall_c1,
    .hall_c2 = &mbl_hall_c2,
};

#define MOTOR_BR DT_NODELABEL(m4)
static const struct gpio_dt_spec mbr_in_a = GPIO_DT_SPEC_GET(MOTOR_BR, in_a_gpios);
static const struct gpio_dt_spec mbr_in_b = GPIO_DT_SPEC_GET(MOTOR_BR, in_b_gpios);
static const struct gpio_dt_spec mbr_hall_c1 = GPIO_DT_SPEC_GET(MOTOR_BR, hall_c1_gpios);
static const struct gpio_dt_spec mbr_hall_c2 = GPIO_DT_SPEC_GET(MOTOR_BR, hall_c2_gpios);
/** @brief Motor @ Back Right */
static const struct motor_gpio mbr = {
    .in_a = &mbr_in_a,
    .in_b = &mbr_in_b,
    .hall_c1 = &mbr_hall_c1,
    .hall_c2 = &mbr_hall_c2,
};

// -----------------------------------------------------------------------------
// FreeBot Motor private functions
// -----------------------------------------------------------------------------

/**
 * @brief Let a motor turn with given direction
 *
 * @param motor Pointer to motor's GPIO handles
 * @param direction Direction to turn (`1` = forwards, `0` = backwards)
 */
static inline void set_motor_direction(const struct motor_gpio *motor, uint8_t direction)
{
    gpio_pin_set_dt(motor->in_a, direction ? 0 : 1);
    gpio_pin_set_dt(motor->in_b, direction ? 1 : 0);
}

/**
 * @brief Stop a motor
 *
 * @param motor Pointer to motor's gpio handles
 */
static inline void set_motor_stop(const struct motor_gpio *motor)
{
    gpio_pin_set_dt(motor->in_a, 0);
    gpio_pin_set_dt(motor->in_b, 0);
}

/** @brief Convert steps to motor angle */
static inline int64_t calculate_motor_angle(int64_t motor_steps)
{
    // TODO: Verify conversion (below is an empirically based guess)
    return motor_steps * 360 / 1380;
}

/** @brief Update `motor_timing` struct */
static inline void update_motor_timing(struct motor_timing *motor, int64_t step_curr, uint64_t time_curr)
{
    motor->step_delta = step_curr - motor->step_prev;
    motor->step_prev = step_curr;
    motor->time_delta = time_curr - motor->time_prev;
    motor->time_prev = time_curr;
}

/** @brief Convert `motor_timing` info to RPM */
static inline int64_t calculate_motor_rpm(struct motor_timing *motor)
{
    int64_t step_d = motor->step_delta;
    uint64_t time_d = motor->time_delta;

    // TODO: Verify conversion (below is an educated guess)
    return (step_d * 60000) / (int64_t)k_ticks_to_ms_near64(time_d * 1380);
}

// -----------------------------------------------------------------------------
// FreeBot Motor interrupt handlers & threads (private)
// -----------------------------------------------------------------------------

/** @brief Generic hall sensor interrupt handler (called by motor specific ones) */
static inline void hall_isr(const struct motor_gpio *motor_pins, int64_t *motor_steps)
{
    int c1 = gpio_pin_get_dt(motor_pins->hall_c1);
    int c2 = gpio_pin_get_dt(motor_pins->hall_c2);

    if (c1 == c2)
    {
        (*motor_steps)++;
    }
    else
    {
        (*motor_steps)--;
    }
}

void mfr_hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mfr, &mfr_steps); }
void mfl_hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mfl, &mfl_steps); }
void mbr_hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mbr, &mbr_steps); }
void mbl_hall_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) { hall_isr(&mbl, &mbl_steps); }

/** @brief Periodically stores required info for calculating (average) motor speeds */
void motor_rpm_thread(void *, void *, void *)
{
    for (;;)
    {
        update_motor_timing(&mfr_timing, mfr_steps, k_uptime_ticks());
        update_motor_timing(&mfl_timing, mfl_steps, k_uptime_ticks());
        update_motor_timing(&mbr_timing, mbr_steps, k_uptime_ticks());
        update_motor_timing(&mbl_timing, mbl_steps, k_uptime_ticks());

        k_sleep(RPM_AVG_PERIOD);
    }
}

K_THREAD_DEFINE(motor_rpm, RPM_THREAD_STACKSIZE, motor_rpm_thread, NULL, NULL, NULL, RPM_THREAD_PRIORITY, 0, 0);

// -----------------------------------------------------------------------------
// FreeBot Motor API (public) functions
// -----------------------------------------------------------------------------

int fb_motor_init(void)
{
    int err = 0;

    err |= !gpio_is_ready_dt(mfr.in_a);
    err |= !gpio_is_ready_dt(mfr.in_b);
    err |= !gpio_is_ready_dt(mfr.hall_c1);
    err |= !gpio_is_ready_dt(mfr.hall_c2);
    err |= !gpio_is_ready_dt(mfl.in_a);
    err |= !gpio_is_ready_dt(mfl.in_b);
    err |= !gpio_is_ready_dt(mfl.hall_c1);
    err |= !gpio_is_ready_dt(mfl.hall_c2);
    err |= !gpio_is_ready_dt(mbr.in_a);
    err |= !gpio_is_ready_dt(mbr.in_b);
    err |= !gpio_is_ready_dt(mbr.hall_c1);
    err |= !gpio_is_ready_dt(mbr.hall_c2);
    err |= !gpio_is_ready_dt(mbl.in_a);
    err |= !gpio_is_ready_dt(mbl.in_b);
    err |= !gpio_is_ready_dt(mbl.hall_c1);
    err |= !gpio_is_ready_dt(mbl.hall_c2);

    if (err)
    {
        return err;
    }

    err |= gpio_pin_configure_dt(mfr.in_a, GPIO_OUTPUT_LOW);
    err |= gpio_pin_configure_dt(mfr.in_b, GPIO_OUTPUT_LOW);
    err |= gpio_pin_configure_dt(mfl.in_a, GPIO_OUTPUT_LOW);
    err |= gpio_pin_configure_dt(mfl.in_b, GPIO_OUTPUT_LOW);
    err |= gpio_pin_configure_dt(mbr.in_a, GPIO_OUTPUT_LOW);
    err |= gpio_pin_configure_dt(mbr.in_b, GPIO_OUTPUT_LOW);
    err |= gpio_pin_configure_dt(mbl.in_a, GPIO_OUTPUT_LOW);
    err |= gpio_pin_configure_dt(mbl.in_b, GPIO_OUTPUT_LOW);

    if (err)
    {
        return err;
    }

    err |= gpio_pin_configure_dt(mfr.hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(mfr.hall_c2, GPIO_INPUT);
    err |= gpio_pin_configure_dt(mfl.hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(mfl.hall_c2, GPIO_INPUT);
    err |= gpio_pin_configure_dt(mbr.hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(mbr.hall_c2, GPIO_INPUT);
    err |= gpio_pin_configure_dt(mbl.hall_c1, GPIO_INPUT);
    err |= gpio_pin_configure_dt(mbl.hall_c2, GPIO_INPUT);

    if (err)
    {
        return err;
    }

    err |= gpio_pin_interrupt_configure_dt(mfr.hall_c2, GPIO_INT_EDGE_BOTH);
    err |= gpio_pin_interrupt_configure_dt(mfl.hall_c2, GPIO_INT_EDGE_BOTH);
    err |= gpio_pin_interrupt_configure_dt(mbr.hall_c2, GPIO_INT_EDGE_BOTH);
    err |= gpio_pin_interrupt_configure_dt(mbl.hall_c2, GPIO_INT_EDGE_BOTH);

    if (err)
    {
        return err;
    }

    gpio_init_callback(&mfr_hall_cb_data, mfr_hall_isr, BIT(mfr.hall_c2->pin));
    gpio_init_callback(&mfl_hall_cb_data, mfl_hall_isr, BIT(mfl.hall_c2->pin));
    gpio_init_callback(&mbr_hall_cb_data, mbr_hall_isr, BIT(mbr.hall_c2->pin));
    gpio_init_callback(&mbl_hall_cb_data, mbl_hall_isr, BIT(mbl.hall_c2->pin));

    err |= gpio_add_callback_dt(mfr.hall_c2, &mfr_hall_cb_data);
    err |= gpio_add_callback_dt(mfl.hall_c2, &mfl_hall_cb_data);
    err |= gpio_add_callback_dt(mbr.hall_c2, &mbr_hall_cb_data);
    err |= gpio_add_callback_dt(mbl.hall_c2, &mbl_hall_cb_data);

    if (err)
    {
        return err;
    }

    fb_stop();
    return 0;
}

void fb_stop(void)
{
    set_motor_stop(&mfl);
    set_motor_stop(&mfr);
    set_motor_stop(&mbl);
    set_motor_stop(&mbr);
}

void fb_straight_forw(void)
{
    set_motor_direction(&mfl, 1);
    set_motor_direction(&mfr, 1);
    set_motor_direction(&mbl, 1);
    set_motor_direction(&mbr, 1);
}

void fb_straight_back(void)
{
    set_motor_direction(&mfl, 0);
    set_motor_direction(&mfr, 0);
    set_motor_direction(&mbl, 0);
    set_motor_direction(&mbr, 0);
}

void fb_side_right(void)
{
    set_motor_direction(&mfl, 0);
    set_motor_direction(&mfr, 1);
    set_motor_direction(&mbl, 1);
    set_motor_direction(&mbr, 0);
}

void fb_side_left(void)
{
    set_motor_direction(&mfl, 1);
    set_motor_direction(&mfr, 0);
    set_motor_direction(&mbl, 0);
    set_motor_direction(&mbr, 1);
}

void fb_side_d45(void)
{
    set_motor_stop(&mfl);
    set_motor_direction(&mfr, 1);
    set_motor_direction(&mbl, 1);
    set_motor_stop(&mbr);
}

void fb_side_d135(void)
{
    set_motor_direction(&mfl, 0);
    set_motor_stop(&mfr);
    set_motor_stop(&mbl);
    set_motor_direction(&mbr, 0);
}

void fb_side_d225(void)
{
    set_motor_stop(&mfl);
    set_motor_direction(&mfr, 0);
    set_motor_direction(&mbl, 0);
    set_motor_stop(&mbr);
}

void fb_side_d315(void)
{
    set_motor_direction(&mfl, 1);
    set_motor_stop(&mfr);
    set_motor_stop(&mbl);
    set_motor_direction(&mbr, 1);
}

void fb_rotate_cw(void)
{
    set_motor_direction(&mfl, 1);
    set_motor_direction(&mfr, 0);
    set_motor_direction(&mbl, 1);
    set_motor_direction(&mbr, 0);
}

void fb_rotate_ccw(void)
{
    set_motor_direction(&mfl, 0);
    set_motor_direction(&mfr, 1);
    set_motor_direction(&mbl, 0);
    set_motor_direction(&mbr, 1);
}

void fb_get_motor_angle(fb_motor_angle_t *angles)
{
    angles->front_left = calculate_motor_angle(mfl_steps);
    angles->front_right = calculate_motor_angle(mfr_steps);
    angles->back_left = calculate_motor_angle(mbl_steps);
    angles->back_right = calculate_motor_angle(mbr_steps);
}

void fb_get_motor_speed(fb_motor_speed_t *speeds)
{
    speeds->front_left = calculate_motor_rpm(&mfl_timing);
    speeds->front_right = calculate_motor_rpm(&mfr_timing);
    speeds->back_left = calculate_motor_rpm(&mbl_timing);
    speeds->back_right = calculate_motor_rpm(&mbr_timing);
}
