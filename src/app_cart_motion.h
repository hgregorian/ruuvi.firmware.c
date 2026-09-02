#ifndef APP_CART_MOTION_H
#define APP_CART_MOTION_H

/**
 * @file app_cart_motion.h
 *
 * Motion-triggered telemetry burst control for the cart application.
 */

#include "ruuvi_driver_error.h"
#include "ruuvi_driver_sensor.h"

#include <stdint.h>

/*
 * DumpSense status values encoded in bits 7..6 of the RAWv2 movement
 * counter field.
 */
#define APP_CART_STATUS_NORMAL      (0U)
#define APP_CART_STATUS_DUMP        (1U)

/**
 * @brief Initialize cart motion handling.
 */
rd_status_t app_cart_motion_init (void);

/**
 * @brief Notify cart motion handler from accelerometer interrupt context.
 *
 * The interrupt is used only to enter active mode. Continued movement and
 * settling are determined from fresh accelerometer samples.
 */
void app_cart_motion_on_motion_isr (void);

/**
 * @brief Process a fresh sensor sample while cart telemetry is active.
 *
 * @param[in] p_data Fresh heartbeat sensor data.
 */
void app_cart_motion_on_sample (const rd_sensor_data_t * const p_data);

/**
 * @brief Get the current DumpSense application status.
 *
 * @return APP_CART_STATUS_NORMAL or APP_CART_STATUS_DUMP.
 */
uint8_t app_cart_motion_status_get (void);

#endif // APP_CART_MOTION_H
