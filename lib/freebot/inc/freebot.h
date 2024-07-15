/**
 * @file freebot.h
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief Main FreeBot library file
 * @version 0.1
 * @date 2024-03-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef FREEBOT_H
#define FREEBOT_H

#include "fb_err.h"
#include "fb_io.h"
#include "fb_motor.h"
#include "fb_pwr.h"

/** @brief initialize FreeBot */
/**
 * @brief Initializes the FreeBot library.
 *
 * This function initializes the FreeBot library by initializing the motor, I/O, and power modules.
 *
 * @return An integer value indicating the success or failure of the initialization process.
 *         - 0: Initialization successful.
 *         - Non-zero: Initialization failed.
 */
int fb_init(void) {
    int err = 0;
    err |= fb_motor_init();
    err |= fb_io_init();
    err |= fb_pwr_init();
    return err;
}

#endif /* FREEBOT_H */
