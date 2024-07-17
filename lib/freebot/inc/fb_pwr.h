/**
 * @file fb_pwr.h
 * @author Lowie Deferme <lowie.deferme@kuleuven.be>
 * @brief FreeBot power API
 * @version 0.1
 * @date 2024-03-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef FB_PWR_H
#define FB_PWR_H

#include "fb_err.h"

#define V_CAP 0
#define V_MOTOR 1
#define V_IN 2

/**
 * @brief Initialize FreeBot's power functionality
 */
int fb_pwr_init(void);

/**
 * @brief Measure FreeBot's voltage
 *
 * @retval >0 : The measured voltage in mV
 * @retval `E_ADC_ERR` : Could not read from ADC
 */
int fb_v_measure(void);



/**
 * @brief Select voltage to measure
 *
 * @param v_sel Voltage to select
 * @retval 0 : Select was successful
 * @retval `E_DEV_UNK` : Unknown `v_sel`
 */
 int fb_v_measure_select(int v_sel);

#endif /* FB_PWR_H */
