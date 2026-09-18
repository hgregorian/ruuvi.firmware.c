#ifndef APP_CART_MOTION_H
#define APP_CART_MOTION_H

/**
 * @file app_cart_motion.h
 *
 * Motion-triggered telemetry burst control for the cart application.
 */

#include "ruuvi_driver_error.h"
#include "ruuvi_driver_sensor.h"

#include <stdbool.h>
#include <stdint.h>


typedef struct
{
    bool valid;
    bool active;
    bool dump_candidate;
    bool dump_evidence;
    bool dump_latched;
    bool rolling_candidate;
    bool rolling_evidence;
    bool upright;
    uint8_t status;
    uint8_t dump_candidate_hits;
    uint8_t dump_candidate_samples;
    uint32_t rolling_evidence_ms;
    float raw_angle_deg;
    float filtered_angle_deg;
    float gravity_angle_deg;
    float sample_g;
    float confidence;
} app_cart_motion_telemetry_t;

/*
 * DumpSense status values encoded in bits 7..6 of the RAWv2 movement
 * counter field.
 */
#define APP_CART_STATUS_NORMAL       (0U)
#define APP_CART_STATUS_DUMP         (1U)
#define APP_CART_STATUS_ROLLING      (2U)
#define APP_CART_STATUS_RESERVED     (3U)

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
 * @return One of the APP_CART_STATUS_* values.
 */
uint8_t app_cart_motion_status_get (void);

/**
 * @brief Return true while DumpSense is in active 100 ms analysis mode.
 */
bool app_cart_motion_active_get (void);

/**
 * @brief Copy the most recent derived DumpSense telemetry snapshot.
 *
 * RAW X/Y/Z are intentionally not duplicated here. RAWv2 remains the single
 * authoritative source for acceleration data.
 *
 * @param[out] p_telemetry Destination snapshot.
 * @return True when a valid active-sample snapshot is available.
 */
bool app_cart_motion_telemetry_get (
    app_cart_motion_telemetry_t * const p_telemetry);
#endif // APP_CART_MOTION_H
