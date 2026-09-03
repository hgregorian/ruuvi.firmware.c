/**
 * @file app_cart_motion.c
 *
 * Motion-triggered telemetry burst control for the cart application.
 */

#include "app_cart_motion.h"

#include "app_comms.h"
#include "app_config.h"
#include "app_heartbeat.h"
#include "app_led.h"
#include "ruuvi_boards.h"
#include "ruuvi_task_led.h"
#include "ruuvi_interface_power.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"
#include "ruuvi_interface_yield.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846F
#endif

#define CART_MOTION_INTERVAL_MS     (200U)
#define CART_DUMP_INTERVAL_MS       (100U)
#define CART_IDLE_INTERVAL_MS       (120U * 1000U)
#define CART_IDLE_TIMEOUT_MS        (5000U)

#define CART_DUMP_CONFIRM_MS        (300U)
#define CART_DUMP_MIN_HOLD_MS       (15U * 1000U)

#define CART_GESTURE_TIP_ANGLE_DEG       (65.0F)
#define CART_GESTURE_UPRIGHT_ANGLE_DEG   (15.0F)
#define CART_GESTURE_STEP_TIMEOUT_MS     (5U * 1000U)
#define CART_GESTURE_TIP_COUNT           (3U)

#define CART_GESTURE_DIAG_NONE            (0U)
#define CART_GESTURE_DIAG_TIP_1           (1U)
#define CART_GESTURE_DIAG_TIP_2           (2U)
#define CART_GESTURE_DIAG_TIP_3           (3U)
#define CART_GESTURE_DIAG_WAIT_IDLE       (4U)

#define CART_GESTURE_DIAG_ABORT_DUMP       (8U)
#define CART_GESTURE_DIAG_ABORT_TIMEOUT    (9U)
#define CART_GESTURE_DIAG_ABORT_WAIT_IDLE (10U)

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
#define CART_SAMPLE_MOTION_G        (0.050F)
#define CART_SAMPLE_MOTION_G2       (CART_SAMPLE_MOTION_G * CART_SAMPLE_MOTION_G)

#define CART_MOVING_CONFIRM_MS       (3U * 1000U)
#define CART_MOVING_GAP_TOLERANCE_MS (1000U)

/*
 * Consider the cart in its normal rolling posture when its acceleration
 * vector is between CART_MOVING_MIN_ANGLE_DEG and
 * CART_MOVING_MAX_ANGLE_DEG from the upright reference vector.
 */
#define CART_MOVING_MIN_ANGLE_DEG    (10.0F)
#define CART_MOVING_MAX_ANGLE_DEG    (60.0F)

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
static uint64_t m_dump_candidate_since_ms;
static uint64_t m_dump_min_hold_until_ms;

static bool m_moving_candidate;
static bool m_moving;
static uint64_t m_moving_candidate_since_ms;
static uint64_t m_last_rolling_motion_ms;

static uint8_t m_gesture_diag;

typedef enum
{
    CART_GESTURE_IDLE = 0,
    CART_GESTURE_IN_PROGRESS,
    CART_GESTURE_WAIT_IDLE
} cart_gesture_state_t;

static cart_gesture_state_t m_gesture_state;
static uint8_t m_gesture_tip_count;
static bool m_gesture_waiting_for_upright;
static uint64_t m_gesture_step_since_ms;

static void cart_gesture_reset (const uint8_t diag)
{
    m_gesture_state = CART_GESTURE_IDLE;
    m_gesture_tip_count = 0U;
    m_gesture_waiting_for_upright = false;
    m_gesture_step_since_ms = 0U;
    m_gesture_diag = diag;
}

static void cart_gesture_update (const float angle_deg,
                                 const uint64_t now_ms)
{
    const bool gesture_upright =
        angle_deg <= CART_GESTURE_UPRIGHT_ANGLE_DEG;

    const bool gesture_tip =
        (angle_deg >= CART_GESTURE_TIP_ANGLE_DEG) &&
        (angle_deg < CART_DUMP_ANGLE_DEG);

    /*
     * Entering the dump orientation invalidates any commissioning gesture in
     * progress. A normal dump may pass through the gesture angle range, but it
     * must never contribute toward a commissioning reset.
     */
    if (angle_deg >= CART_DUMP_ANGLE_DEG)
    {
        cart_gesture_reset(CART_GESTURE_DIAG_ABORT_DUMP);
        return;
    }

    if ((m_gesture_state == CART_GESTURE_IN_PROGRESS) &&
        ((now_ms - m_gesture_step_since_ms) >
         CART_GESTURE_STEP_TIMEOUT_MS))
    {
        cart_gesture_reset (CART_GESTURE_DIAG_ABORT_TIMEOUT);
        return;
    }

    switch (m_gesture_state)
    {
        case CART_GESTURE_IDLE:
            /*
             * A gesture must begin from the normal upright position.
             */
            if (gesture_upright)
            {
                m_gesture_state = CART_GESTURE_IN_PROGRESS;
                m_gesture_tip_count = 0U;
                m_gesture_waiting_for_upright = false;
                m_gesture_step_since_ms = now_ms;
            }
            break;

        case CART_GESTURE_IN_PROGRESS:
            if (!m_gesture_waiting_for_upright)
            {
                /*
                 * Wait for the next deliberate tip.
                 */
                if (gesture_tip)
                {
                    m_gesture_tip_count++;
                    m_gesture_waiting_for_upright = true;
                    m_gesture_step_since_ms = now_ms;
               
                    if (1U == m_gesture_tip_count)
                    {
                        m_gesture_diag = CART_GESTURE_DIAG_TIP_1;
                    }
                    else if (2U == m_gesture_tip_count)
                    {
                        m_gesture_diag = CART_GESTURE_DIAG_TIP_2;
                    }
                    else if (3U == m_gesture_tip_count)
                    {
                        m_gesture_diag = CART_GESTURE_DIAG_TIP_3;
                    }
                }
            }
            else if (gesture_upright)
            {
                /*
                 * Every tip must be followed by a full return upright.
                 */
                if (m_gesture_tip_count >= CART_GESTURE_TIP_COUNT)
                {
                    m_gesture_state = CART_GESTURE_WAIT_IDLE;
                    m_gesture_diag = CART_GESTURE_DIAG_WAIT_IDLE;
                }
                else
                {
                    m_gesture_waiting_for_upright = false;
                    m_gesture_step_since_ms = now_ms;
                }
            }
            break;

        case CART_GESTURE_WAIT_IDLE:
            /*
             * The completed gesture remains valid only while the cart stays in
             * its commissioning upright posture. cart_idle() performs the reset
             * once the normal idle timeout has elapsed.
             */
            if (!gesture_upright)
            {
                cart_gesture_reset(CART_GESTURE_DIAG_ABORT_WAIT_IDLE);
            }
            break;

        default:
            cart_gesture_reset(CART_GESTURE_DIAG_NONE);
            break;
    }
}

static float cart_angle_from_upright_deg (const float x,
                                          const float y,
                                          const float z)
{
    const float dot =
        (x * m_upright_x) +
        (y * m_upright_y) +
        (z * m_upright_z);

    const float sample_mag =
        sqrtf ((x * x) + (y * y) + (z * z));

    const float upright_mag =
        sqrtf ((m_upright_x * m_upright_x) +
               (m_upright_y * m_upright_y) +
               (m_upright_z * m_upright_z));

    if ((sample_mag <= 0.0F) || (upright_mag <= 0.0F))
    {
        return NAN;
    }

    float cosine = dot / (sample_mag * upright_mag);

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
    return (angle_deg >= CART_MOVING_MIN_ANGLE_DEG) &&
           (angle_deg <= CART_MOVING_MAX_ANGLE_DEG);
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
     * While dump state is asserted, remain out of idle telemetry mode.
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
     * A completed commissioning gesture requests a reboot once the cart has
     * reached the normal idle condition. Startup will then establish a fresh
     * upright reference.
     */
    if (m_gesture_state == CART_GESTURE_WAIT_IDLE)
    {
        (void) rt_led_write (RB_LED_RED, true);
        (void) ri_delay_ms (5000U);
        ri_power_reset();
    }

    /*
     * Stop motion evaluation before taking the final sample so the final
     * heartbeat cannot restart the inactivity timer.
     */
    m_active = false;
    m_have_previous_sample = false;

    m_moving_candidate = false;
    m_moving = false;
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

        m_dump_candidate = false;
        m_dump_latched = false;
        m_dump_armed = true;
        m_dump_fast = false;

        m_last_motion_ms = 0U;
        m_dump_candidate_since_ms = 0U;
        m_dump_min_hold_until_ms = 0U;

        m_moving_candidate = false;
        m_moving = false;

        m_moving_candidate_since_ms = 0U;
        m_last_rolling_motion_ms = 0U;

        m_gesture_state = CART_GESTURE_IDLE;
        m_gesture_tip_count = 0U;
        m_gesture_waiting_for_upright = false;
        m_gesture_step_since_ms = 0U;
        m_gesture_diag = CART_GESTURE_DIAG_NONE;

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
    app_comms_bleadv_interval_set (
        m_dump_fast
            ? CART_DUMP_INTERVAL_MS
            : CART_MOTION_INTERVAL_MS);

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

    const float angle_deg =
        cart_angle_from_upright_deg (x, y, z);

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
            m_dump_fast = true;
            m_dump_min_hold_until_ms = now_ms + CART_DUMP_MIN_HOLD_MS;

            app_comms_bleadv_send_count_set (1U);
            app_comms_bleadv_interval_set (CART_DUMP_INTERVAL_MS);
            (void) app_heartbeat_interval_set (CART_DUMP_INTERVAL_MS);

            /*
             * Keep non-idle telemetry running while dump status is asserted.
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
             * A gap longer than CART_MOVING_GAP_TOLERANCE_MS starts a new
             * sustained rolling-motion candidate.
             */
            if (m_moving_candidate &&
                ((now_ms - m_last_rolling_motion_ms) >
                 CART_MOVING_GAP_TOLERANCE_MS))
            {
                m_moving_candidate = false;
            }

            if ((!m_moving) && (!m_moving_candidate))
            {
                m_moving_candidate = true;
                m_moving_candidate_since_ms = now_ms;
            }
            else if (m_moving_candidate &&
                     ((now_ms - m_moving_candidate_since_ms) >=
                      CART_MOVING_CONFIRM_MS))
            {
                m_moving_candidate = false;
                m_moving = true;
            }

            m_last_rolling_motion_ms = now_ms;
        }
        else if ((m_moving_candidate || m_moving) &&
                 ((now_ms - m_last_rolling_motion_ms) >
                  CART_MOVING_GAP_TOLERANCE_MS))
        {
            m_moving_candidate = false;
            m_moving = false;
        }

        cart_gesture_update (angle_deg, now_ms);
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

    if (m_moving)
    {
        return APP_CART_STATUS_MOVING;
    }

    return APP_CART_STATUS_NORMAL;
}

uint8_t app_cart_motion_gesture_status_get (void)
{
    return m_gesture_diag;
}