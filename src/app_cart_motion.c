/**
 * @file app_cart_motion.c
 *
 * Motion-triggered telemetry burst control for the cart application.
 */

#include "app_cart_motion.h"

#include "app_comms.h"
#include "app_config.h"
#include "app_heartbeat.h"
#include "ruuvi_boards.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846F
#endif

#define CART_MOTION_INTERVAL_MS     (100U)
#define CART_DUMP_INTERVAL_MS       (100U)
// #define CART_IDLE_INTERVAL_MS       (120U * 1000U)
#define CART_IDLE_INTERVAL_MS       (10U * 1000U)
#define CART_IDLE_TIMEOUT_MS        (5000U)

#define CART_DUMP_CONFIRM_MS        (300U)
#define CART_DUMP_CONFIRM_SAMPLES   (4U)
#define CART_DUMP_REQUIRED_HITS     (3U)
#define CART_DUMP_MIN_HOLD_MS       (15U * 1000U)

/*
 * Consider the cart inverted when its acceleration vector is at least
 * CART_DUMP_ANGLE_DEG from the upright reference vector.
 */
#define CART_DUMP_ANGLE_DEG         (135.0F)

/*
 * Consider the cart returned upright when its acceleration vector is at most
 * CART_UPRIGHT_ANGLE_DEG from the upright reference vector.
 */
#define CART_UPRIGHT_ANGLE_DEG      (60.0F)

/*
 * Change in acceleration vector required to consider the cart still moving.
 *
 * 0.05 g is an intentionally conservative initial value for hardware testing.
 * Compare squared magnitudes to avoid sqrtf().
 */
#define CART_SAMPLE_MOTION_G          (0.050F)
#define CART_SAMPLE_MOTION_G2         (CART_SAMPLE_MOTION_G * CART_SAMPLE_MOTION_G)
#define CART_ROLLING_CONFIRM_MS       (3U * 1000U)
#define CART_ROLLING_GAP_TOLERANCE_MS (2000U)

/*
 * Consider the cart in its normal rolling posture when its acceleration
 * vector is between CART_ROLLING_MIN_ANGLE_DEG and
 * CART_ROLLING_MAX_ANGLE_DEG from the upright reference vector.
 */
#define CART_ROLLING_MIN_ANGLE_DEG    (10.0F)
#define CART_ROLLING_MAX_ANGLE_DEG    (75.0F)

static ri_timer_id_t m_idle_timer;
static bool m_active;
static bool m_have_previous_sample;
static bool m_have_upright_sample;
static bool m_dump_candidate;
static bool m_dump_latched;
static bool m_dump_armed;
static bool m_dump_fast;

static float m_previous_x;
static float m_previous_y;
static float m_previous_z;

static float m_upright_x;
static float m_upright_y;
static float m_upright_z;

static uint64_t m_last_motion_ms;
static uint8_t m_dump_candidate_samples;
static uint8_t m_dump_candidate_hits;
static uint64_t m_dump_min_hold_until_ms;
static bool m_rolling_candidate;
static bool m_rolling;
static uint64_t m_rolling_candidate_since_ms;
static uint64_t m_last_rolling_motion_ms;

static float cart_angle_from_reference_deg (const float x,
                                            const float y,
                                            const float z,
                                            const float ref_x,
                                            const float ref_y,
                                            const float ref_z)
{
    const float dot =
        (x * ref_x) +
        (y * ref_y) +
        (z * ref_z);

    const float sample_mag =
        sqrtf ((x * x) + (y * y) + (z * z));

    const float reference_mag =
        sqrtf ((ref_x * ref_x) +
               (ref_y * ref_y) +
               (ref_z * ref_z));

    if ((sample_mag <= 0.0F) || (reference_mag <= 0.0F))
    {
        return NAN;
    }

    float cosine = dot / (sample_mag * reference_mag);

    if (cosine > 1.0F)
    {
        cosine = 1.0F;
    }
    else if (cosine < -1.0F)
    {
        cosine = -1.0F;
    }

    return acosf (cosine) * (180.0F / M_PI);
}

static bool cart_is_inverted (const float angle_deg)
{
    return angle_deg >= CART_DUMP_ANGLE_DEG;
}

static bool cart_is_upright (const float angle_deg)
{
    return angle_deg <= CART_UPRIGHT_ANGLE_DEG;
}

static bool cart_is_rolling (const float angle_deg)
{
    return (angle_deg >= CART_ROLLING_MIN_ANGLE_DEG) &&
           (angle_deg <= CART_ROLLING_MAX_ANGLE_DEG);
}

static void cart_dump_candidate_reset (void)
{
    m_dump_candidate = false;
    m_dump_candidate_samples = 0U;
    m_dump_candidate_hits = 0U;
}

static void cart_dump_candidate_start (void)
{
    m_dump_candidate = true;
    m_dump_candidate_samples = 1U;
    m_dump_candidate_hits = 1U;
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
    const uint64_t quiet_ms = now_ms - m_last_motion_ms;

    if (quiet_ms < CART_IDLE_TIMEOUT_MS)
    {
        const uint32_t remaining_ms =
            (uint32_t) (CART_IDLE_TIMEOUT_MS - quiet_ms);

        (void) ri_timer_start (m_idle_timer, remaining_ms, NULL);
        return;
    }

    /*
     * A real dump necessarily involves continuing physical motion. If the
     * cart has been completely quiet for the full idle timeout, any remaining
     * DUMP latch is stale and must not prevent recovery to idle.
     */
    if (m_dump_latched)
    {
        m_dump_latched = false;
        cart_dump_candidate_reset();
        m_dump_armed = true;
        m_dump_fast = false;
    }

    /*
     * Stop motion evaluation before taking the final sample so the final
     * heartbeat cannot restart the inactivity timer.
     */
    m_active = false;
    m_have_previous_sample = false;
    m_rolling_candidate = false;
    m_rolling = false;

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
        app_comms_bleadv_interval_set (CART_MOTION_INTERVAL_MS);
        (void) app_heartbeat_interval_set (CART_MOTION_INTERVAL_MS);

        m_active = true;
        m_have_previous_sample = false;

        /*
         * Generate fresh telemetry immediately rather than waiting for the
         * first CART_MOTION_INTERVAL_MS heartbeat timer expiration.
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

        cart_dump_candidate_reset();
        m_dump_latched = false;
        m_dump_armed = true;
        m_dump_fast = false;

        m_last_motion_ms = 0U;
        m_dump_min_hold_until_ms = 0U;
        m_rolling_candidate = false;
        m_rolling = false;

        m_rolling_candidate_since_ms = 0U;
        m_last_rolling_motion_ms = 0U;

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
    if (NULL == p_data)
    {
        return;
    }

    const float x = rd_sensor_data_parse (p_data, RD_SENSOR_ACC_X_FIELD);
    const float y = rd_sensor_data_parse (p_data, RD_SENSOR_ACC_Y_FIELD);
    const float z = rd_sensor_data_parse (p_data, RD_SENSOR_ACC_Z_FIELD);

    if (isnan (x) || isnan (y) || isnan (z))
    {
        return;
    }

    /*
     * This lets the stationary startup heartbeat following a reboot establish
     * the permanent upright reference before active motion processing begins.
     */
    if (!m_have_upright_sample)
    {
        m_upright_x = x;
        m_upright_y = y;
        m_upright_z = z;
        m_have_upright_sample = true;
    }

    /*
     * Idle/startup samples may establish the permanent upright reference, but
     * all DUMP, ROLLING, and active-telemetry processing remains gated by
     * m_active.
     */
    if (!m_active)
    {
        return;
    }

    /*
     * Keep the active BLE configuration asserted while sampling. Ruuvi's
     * startup advertising transition can otherwise restore the stock BLE
     * interval while cart active mode is already running.
     */
    app_comms_bleadv_send_count_set (1U);
    app_comms_bleadv_interval_set (
        m_dump_fast
            ? CART_DUMP_INTERVAL_MS
            : CART_MOTION_INTERVAL_MS);

    const uint64_t now_ms = ri_rtc_millis();

    const float angle_deg =
        cart_angle_from_reference_deg (
            x,
            y,
            z,
            m_upright_x,
            m_upright_y,
            m_upright_z);

    const bool inverted =
        cart_is_inverted (angle_deg);

    const bool upright =
        cart_is_upright (angle_deg);

    const bool rolling =
        cart_is_rolling (angle_deg);

    /*
     * Use CART_DUMP_INTERVAL_MS telemetry only to protect delivery of the dump
     * event. After the minimum hold interval, return to CART_MOTION_INTERVAL_MS
     * even if the dump status remains asserted.
     */
    if (m_dump_fast && (now_ms >= m_dump_min_hold_until_ms))
    {
        m_dump_fast = false;

        app_comms_bleadv_interval_set (CART_MOTION_INTERVAL_MS);
        (void) app_heartbeat_interval_set (CART_MOTION_INTERVAL_MS);
    }

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
    else if (m_dump_armed)
    {
        if (!m_dump_candidate)
        {
            if (inverted)
            {
                cart_dump_candidate_start();
            }
        }
        else
        {
            m_dump_candidate_samples++;

            if (inverted)
            {
                m_dump_candidate_hits++;
            }

            if (m_dump_candidate_samples >= CART_DUMP_CONFIRM_SAMPLES)
            {
                if (m_dump_candidate_hits >= CART_DUMP_REQUIRED_HITS)
                {
                    cart_dump_candidate_reset();
                    m_dump_latched = true;
                    m_dump_armed = false;
                    m_dump_fast = true;
                    m_dump_min_hold_until_ms =
                        now_ms + CART_DUMP_MIN_HOLD_MS;
                    app_comms_bleadv_send_count_set (1U);
                    app_comms_bleadv_interval_set (CART_DUMP_INTERVAL_MS);
                    (void) app_heartbeat_interval_set (
                               CART_DUMP_INTERVAL_MS);

                    /*
                     * Keep non-idle telemetry running while dump status is
                     * asserted.
                     */
                    cart_idle_timer_restart();
                }
                else
                {
                    /*
                     * This 300 ms window did not contain enough inversion
                     * evidence. If the final sample is inverted, preserve it
                     * as the first sample of the next candidate window.
                     */
                    cart_dump_candidate_reset();

                    if (inverted)
                    {
                        cart_dump_candidate_start();
                    }
                }
            }
        }
    }
    if (m_have_previous_sample)
    {
        const float dx = x - m_previous_x;
        const float dy = y - m_previous_y;
        const float dz = z - m_previous_z;

        const float delta_g2 = (dx * dx) + (dy * dy) + (dz * dz);

        const bool sample_moving =
            delta_g2 >= CART_SAMPLE_MOTION_G2;

        const bool rolling_motion =
            sample_moving && rolling;

        if (sample_moving)
        {
            m_last_motion_ms = now_ms;
            cart_idle_timer_restart();
        }

        if (rolling_motion)
        {
            /*
             * A gap longer than CART_ROLLING_GAP_TOLERANCE_MS starts a new
             * sustained rolling-motion candidate.
             */
            if (m_rolling_candidate &&
                ((now_ms - m_last_rolling_motion_ms) >
                 CART_ROLLING_GAP_TOLERANCE_MS))
            {
                m_rolling_candidate = false;
            }
            if ((!m_rolling) && (!m_rolling_candidate))
            {
                m_rolling_candidate = true;
                m_rolling_candidate_since_ms = now_ms;
            }
            else if (m_rolling_candidate &&
                     ((now_ms - m_rolling_candidate_since_ms) >=
                      CART_ROLLING_CONFIRM_MS))
            {
                m_rolling_candidate = false;
                m_rolling = true;
            }
            m_last_rolling_motion_ms = now_ms;
        }
        else if ((m_rolling_candidate || m_rolling) &&
                 ((now_ms - m_last_rolling_motion_ms) >
                  CART_ROLLING_GAP_TOLERANCE_MS))
        {
            m_rolling_candidate = false;
            m_rolling = false;
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
    if (m_dump_latched)
    {
        return APP_CART_STATUS_DUMP;
    }

    if (m_rolling)
    {
        return APP_CART_STATUS_ROLLING;
    }

    return APP_CART_STATUS_NORMAL;
}
