/**
 * @file app_cart_motion.c
 *
 * Motion-triggered telemetry burst control for the cart application.
 */

#include "app_cart_motion.h"

#include "app_comms.h"
#include "app_config.h"
#include "app_heartbeat.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"

#include <math.h>
#include <stdbool.h>

#define CART_ACTIVE_INTERVAL_MS     (100U)
#define CART_IDLE_INTERVAL_MS       (120U * 1000U)
#define CART_IDLE_TIMEOUT_MS        (5000U)

/*
 * Change in acceleration vector required to consider the cart still moving.
 *
 * 0.05 g is an intentionally conservative initial value for hardware testing.
 * Compare squared magnitudes to avoid sqrtf().
 */
#define CART_SAMPLE_MOTION_G        (0.050F)
#define CART_SAMPLE_MOTION_G2       (CART_SAMPLE_MOTION_G * CART_SAMPLE_MOTION_G)

static ri_timer_id_t m_idle_timer;
static bool m_active;
static bool m_have_previous_sample;

static float m_previous_x;
static float m_previous_y;
static float m_previous_z;

static uint64_t m_last_motion_ms;

static void cart_idle_timer_restart (void)
{
    (void) ri_timer_stop (m_idle_timer);
    (void) ri_timer_start (m_idle_timer, CART_IDLE_TIMEOUT_MS, NULL);
}

static void cart_idle (void * p_event, uint16_t event_size)
{
    (void) p_event;
    (void) event_size;

    if (!m_active)
    {
        return;
    }

    /*
     * Protect against a timer callback which was already queued just before
     * a fresh motion sample reset the quiet period.
     */
    const uint64_t now_ms = ri_rtc_millis();
    const uint64_t quiet_ms = now_ms - m_last_motion_ms;

    if (quiet_ms < CART_IDLE_TIMEOUT_MS)
    {
        const uint32_t remaining_ms =
            (uint32_t) (CART_IDLE_TIMEOUT_MS - quiet_ms);

        (void) ri_timer_start (m_idle_timer, remaining_ms, NULL);
        return;
    }

    /*
     * Stop motion evaluation before taking the final sample so the final
     * heartbeat cannot restart the inactivity timer.
     */
    m_active = false;
    m_have_previous_sample = false;
    /*
     * Send one final fresh sample while fast advertising is still active.
     */
    app_heartbeat_now();
    /*
     * Return to low-power idle telemetry.
     *
     * Generate a fresh sample every 60 seconds and retain the stock
     * two-advertisement delivery behavior for each sample.
     */
    app_comms_bleadv_send_count_set (APP_NUM_REPEATS);
    app_comms_bleadv_interval_set (APP_BLE_INTERVAL_MS);
    (void) app_heartbeat_interval_set (CART_IDLE_INTERVAL_MS);
}

static void cart_idle_timeout_isr (void * const p_context)
{
    (void) p_context;
    (void) ri_scheduler_event_put (NULL, 0U, &cart_idle);
}

static void cart_motion (void * p_event, uint16_t event_size)
{
    (void) p_event;
    (void) event_size;

    m_last_motion_ms = ri_rtc_millis();
    cart_idle_timer_restart();

    if (!m_active)
    {
        /*
         * Enter active telemetry mode.
         */
        app_comms_bleadv_send_count_set (1U);
        app_comms_bleadv_interval_set (CART_ACTIVE_INTERVAL_MS);
        (void) app_heartbeat_interval_set (CART_ACTIVE_INTERVAL_MS);

        m_active = true;
        m_have_previous_sample = false;

        /*
         * Generate fresh telemetry immediately rather than waiting for the
         * first 100 ms heartbeat timer expiration.
         */
        app_heartbeat_now();
    }
}

rd_status_t app_cart_motion_init (void)
{
    rd_status_t err_code = RD_SUCCESS;

    if ( (!ri_timer_is_init()) || (!ri_scheduler_is_init()))
    {
        err_code |= RD_ERROR_INVALID_STATE;
    }
    else
    {
        m_active = false;
        m_have_previous_sample = false;
        m_last_motion_ms = 0U;

        err_code |= ri_timer_create (&m_idle_timer,
                                     RI_TIMER_MODE_SINGLE_SHOT,
                                     &cart_idle_timeout_isr);

        /*
         * DumpSense spends most of its life stationary. Keep periodic
         * idle telemetry infrequent while relying on the accelerometer
         * interrupt for immediate transition to active telemetry.
         */
        err_code |= app_heartbeat_interval_set (CART_IDLE_INTERVAL_MS);
    }

    return err_code;
}

void app_cart_motion_on_motion_isr (void)
{
    (void) ri_scheduler_event_put (NULL, 0U, &cart_motion);
}

void app_cart_motion_on_sample (const rd_sensor_data_t * const p_data)
{
    if ( (!m_active) || (NULL == p_data))
    {
        return;
    }

    /*
     * Keep the active BLE configuration asserted while sampling. Ruuvi's
     * startup advertising transition can otherwise restore the stock BLE
     * interval while cart active mode is already running.
     */
    app_comms_bleadv_send_count_set (1U);
    app_comms_bleadv_interval_set (CART_ACTIVE_INTERVAL_MS);

    const float x = rd_sensor_data_parse (p_data, RD_SENSOR_ACC_X_FIELD);
    const float y = rd_sensor_data_parse (p_data, RD_SENSOR_ACC_Y_FIELD);
    const float z = rd_sensor_data_parse (p_data, RD_SENSOR_ACC_Z_FIELD);

    if (isnan (x) || isnan (y) || isnan (z))
    {
        return;
    }

    if (m_have_previous_sample)
    {
        const float dx = x - m_previous_x;
        const float dy = y - m_previous_y;
        const float dz = z - m_previous_z;

        const float delta_g2 = (dx * dx) + (dy * dy) + (dz * dz);

        if (delta_g2 >= CART_SAMPLE_MOTION_G2)
        {
            m_last_motion_ms = ri_rtc_millis();
            cart_idle_timer_restart();
        }
    }
    else
    {
        /*
         * The first active sample establishes the comparison baseline and
         * starts a full quiet-period window.
         */
        m_have_previous_sample = true;
        m_last_motion_ms = ri_rtc_millis();
        cart_idle_timer_restart();
    }

    m_previous_x = x;
    m_previous_y = y;
    m_previous_z = z;
}
