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

#define CART_DUMP_CONFIRM_MS        (300U)
#define CART_DUMP_MIN_HOLD_MS       (15U * 1000U)

/*
 * Consider the cart inverted when its acceleration vector is more than
 * approximately 135 degrees from the upright reference vector.
 *
 * cos(135 degrees)^2 = 0.5. Checking the squared dot product avoids sqrtf().
 */
#define CART_DUMP_DOT_RATIO2        (0.50F)

/*
 * Consider the cart returned upright when its acceleration vector is less
 * than approximately 60 degrees from the upright reference vector.
 *
 * cos(60 degrees)^2 = 0.25.
 */
#define CART_UPRIGHT_DOT_RATIO2     (0.25F)

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

static bool m_have_upright_sample;
static bool m_dump_candidate;
static bool m_dump_latched;
static bool m_dump_armed;

static float m_previous_x;
static float m_previous_y;
static float m_previous_z;

static float m_upright_x;
static float m_upright_y;
static float m_upright_z;

static uint64_t m_last_motion_ms;
static uint64_t m_dump_candidate_since_ms;
static uint64_t m_dump_min_hold_until_ms;

static bool cart_is_inverted (const float x,
                              const float y,
                              const float z)
{
    if (!m_have_upright_sample)
    {
        return false;
    }

    const float dot =
        (x * m_upright_x) +
        (y * m_upright_y) +
        (z * m_upright_z);

    /*
     * Inversion requires the vectors to point in opposite hemispheres.
     */
    if (dot >= 0.0F)
    {
        return false;
    }

    const float sample_mag2 =
        (x * x) + (y * y) + (z * z);

    const float upright_mag2 =
        (m_upright_x * m_upright_x) +
        (m_upright_y * m_upright_y) +
        (m_upright_z * m_upright_z);

    /*
     * dot^2 / (|a|^2 |b|^2) >= 0.5 corresponds to an angle >= 135 degrees
     * when dot is negative.
     */
    return (dot * dot) >=
           (CART_DUMP_DOT_RATIO2 * sample_mag2 * upright_mag2);
}

static bool cart_is_upright (const float x,
                             const float y,
                             const float z)
{
    if (!m_have_upright_sample)
    {
        return false;
    }

    const float dot =
        (x * m_upright_x) +
        (y * m_upright_y) +
        (z * m_upright_z);

    /*
     * Upright requires both vectors to point into the same hemisphere.
     */
    if (dot <= 0.0F)
    {
        return false;
    }

    const float sample_mag2 =
        (x * x) + (y * y) + (z * z);

    const float upright_mag2 =
        (m_upright_x * m_upright_x) +
        (m_upright_y * m_upright_y) +
        (m_upright_z * m_upright_z);

    /*
     * dot^2 / (|a|^2 |b|^2) >= 0.25 corresponds to an angle <= 60 degrees
     * when dot is positive.
     */
    return (dot * dot) >=
           (CART_UPRIGHT_DOT_RATIO2 * sample_mag2 * upright_mag2);
}

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

    /*
     * While dump state is asserted, remain in active telemetry mode.
     * app_cart_motion_on_sample() clears the state only after the minimum
     * hold interval has elapsed and the cart has returned upright.
     */
    if (m_dump_latched)
    {
        cart_idle_timer_restart();
        return;
    }

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
     * Generate a fresh sample at CART_IDLE_INTERVAL_MS and retain the stock
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
        m_have_upright_sample = false;

        m_dump_candidate = false;
        m_dump_latched = false;
        m_dump_armed = true;

        m_last_motion_ms = 0U;
        m_dump_candidate_since_ms = 0U;
        m_dump_min_hold_until_ms = 0U;

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

    const uint64_t now_ms = ri_rtc_millis();

    /*
     * The first valid active sample establishes the normal cart orientation.
     * The cart is expected to enter active mode while still substantially
     * upright when it is first picked up or begins moving.
     */
    if (!m_have_upright_sample)
    {
        m_upright_x = x;
        m_upright_y = y;
        m_upright_z = z;
        m_have_upright_sample = true;
    }

    const bool inverted = cart_is_inverted (x, y, z);

    const bool upright = cart_is_upright (x, y, z);

    if (m_dump_latched)
    {
        /*
         * The dump status is asserted for at least CART_DUMP_MIN_HOLD_MS.
         * After that minimum interval, clear it only after the cart has
         * physically returned close to its normal upright orientation.
         */
        if ( (now_ms >= m_dump_min_hold_until_ms) && upright)
        {
            m_dump_latched = false;
            m_dump_armed = true;
        }
    }
    else if (inverted && m_dump_armed)
    {
        if (!m_dump_candidate)
        {
            m_dump_candidate = true;
            m_dump_candidate_since_ms = now_ms;
        }
        else if ( (now_ms - m_dump_candidate_since_ms) >=
                  CART_DUMP_CONFIRM_MS)
        {
            m_dump_candidate = false;
            m_dump_latched = true;
            m_dump_armed = false;
            m_dump_min_hold_until_ms = now_ms + CART_DUMP_MIN_HOLD_MS;

            /*
             * Keep active telemetry running while dump status is asserted.
             */
            cart_idle_timer_restart();
        }
    }
    else if (!inverted)
    {
        m_dump_candidate = false;
    }

    if (m_have_previous_sample)
    {
        const float dx = x - m_previous_x;
        const float dy = y - m_previous_y;
        const float dz = z - m_previous_z;

        const float delta_g2 = (dx * dx) + (dy * dy) + (dz * dz);

        if (delta_g2 >= CART_SAMPLE_MOTION_G2)
        {
            m_last_motion_ms = now_ms;
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
        m_last_motion_ms = now_ms;
        cart_idle_timer_restart();
    }

    m_previous_x = x;
    m_previous_y = y;
    m_previous_z = z;
}

uint8_t app_cart_motion_status_get (void)
{
    return m_dump_latched
           ? APP_CART_STATUS_DUMP
           : APP_CART_STATUS_NORMAL;
}
