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
#include "ruuvi_task_advertisement.h"
#include "ruuvi_interface_rtc.h"
#include "ruuvi_interface_scheduler.h"
#include "ruuvi_interface_timer.h"

#include <math.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846F
#endif

#define CART_ANALYSIS_INTERVAL_MS   (100U)
#define CART_ACTIVE_ADV_INTERVAL_MS (50U)
// #define CART_IDLE_INTERVAL_MS       (120U * 1000U)
#define CART_IDLE_INTERVAL_MS       (10U * 1000U)
#define CART_IDLE_TIMEOUT_MS        (5000U)
#define CART_IDLE_ADV_SEND_COUNT    (3U)
#define CART_IDLE_ADV_DRAIN_MS      (400U)

#define CART_DUMP_CONFIRM_MS        (300U)
#define CART_DUMP_CONFIRM_SAMPLES   (4U)
#define CART_DUMP_REQUIRED_HITS     (3U)
#define CART_DUMP_ACTIVE_HOLD_MS    (15U * 1000U)

/*
 * Consider the cart inverted when its acceleration vector is at least
 * CART_DUMP_ANGLE_DEG from the upright reference vector.
 */
#define CART_DUMP_ANGLE_DEG         (135.0F)


/*
 * Time constant for the low-pass filter used to estimate cart orientation
 * for DUMP/upright detection.
 *
 * The effective EMA coefficient is calculated from the active sample interval:
 *
 *     alpha = 1 - exp(-sample_interval / CART_DUMP_FILTER_TAU_MS)
 *
 * With CART_ANALYSIS_INTERVAL_MS = 100 ms and TAU = 280 ms:
 *
 *     alpha ~= 0.30
 *
 * Smaller TAU:
 *   - reacts faster to real cart rotation
 *   - passes more vibration / impact acceleration
 *   - increases risk of false DUMP detection
 *
 * Larger TAU:
 *   - rejects short acceleration spikes more strongly
 *   - produces a smoother gravity/orientation estimate
 *   - delays recognition of fast dump events
 *
 * Approximate examples at a 100 ms sample interval:
 *
 *     TAU  150 ms -> alpha ~= 0.49  (light filtering)
 *     TAU  280 ms -> alpha ~= 0.30  (current setting)
 *     TAU  500 ms -> alpha ~= 0.18  (stronger filtering)
 *     TAU 1000 ms -> alpha ~= 0.10  (very sluggish)
 *
 * Because alpha is derived from the sample interval, changing the active
 * sampling rate preserves approximately the same real-world filter response.
 */
#define CART_DUMP_FILTER_TAU_MS     (280.0F)

/*
 * High-g samples are more likely to be dominated by impact / translational
 * acceleration than gravity. Sample magnitude is normalized against the
 * learned stationary upright magnitude, so 1.0 g reflects this tag's own
 * measured baseline rather than a hard-coded standard-gravity constant.
 * Leave samples at or below the onset threshold fully trusted, then
 * exponentially reduce their influence on the DUMP EMA. Low-g samples remain
 * fully trusted because representative dump events can legitimately contain
 * brief low-g phases.
 *
 * confidence = 1.0                               , g <= onset
 * confidence = exp(-(g - onset) / decay)         , g > onset
 * effective_alpha = base_alpha * confidence
 */
#define CART_DUMP_CONFIDENCE_ONSET_G (1.8F)
#define CART_DUMP_CONFIDENCE_DECAY_G (1.0F)
#define CART_DUMP_MIN_HIT_CONFIDENCE (0.75F)

/*
 * Allow a latched DUMP to re-arm once the filtered acceleration vector returns
 * within CART_DUMP_REARM_ANGLE_DEG of the upright reference vector.
 */
#define CART_DUMP_REARM_ANGLE_DEG   (60.0F)

/*
 * Report the cart as upright when its filtered acceleration vector is at most
 * CART_UPRIGHT_ANGLE_DEG from the upright reference vector.
 */
#define CART_UPRIGHT_ANGLE_DEG      (15.0F)

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
static bool m_idle_restore_pending;
static bool m_active;
static bool m_have_previous_sample;
static bool m_have_upright_sample;
static bool m_have_dump_filter;
static bool m_dump_candidate;
static bool m_dump_latched;
static bool m_dump_armed;

static float m_previous_x;
static float m_previous_y;
static float m_previous_z;

static float m_upright_x;
static float m_upright_y;
static float m_upright_z;
static float m_upright_mag;

static float m_dump_filtered_x;
static float m_dump_filtered_y;
static float m_dump_filtered_z;
static float m_dump_filter_alpha;

static uint64_t m_last_motion_ms;
static uint8_t m_dump_candidate_samples;
static uint8_t m_dump_candidate_hits;
static uint64_t m_dump_min_hold_until_ms;
static bool m_rolling_candidate;
static bool m_rolling;
static uint32_t m_rolling_evidence_ms;
static uint64_t m_last_rolling_motion_ms;
static app_cart_motion_telemetry_t m_telemetry;

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

static bool cart_is_dump_rearmed (const float angle_deg)
{
    return angle_deg <= CART_DUMP_REARM_ANGLE_DEG;
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
    m_idle_restore_pending = false;
    (void) ri_timer_stop (m_idle_timer);
    (void) ri_timer_start (m_idle_timer, CART_IDLE_TIMEOUT_MS, NULL);
}

static void cart_idle (void * p_event, uint16_t event_size)
{
    (void) p_event;
    (void) event_size;

    if (m_idle_restore_pending)
    {
        m_idle_restore_pending = false;
        app_comms_bleadv_send_count_set (APP_NUM_REPEATS);
        app_comms_bleadv_interval_set (APP_BLE_INTERVAL_MS);
        return;
    }

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
    }

    /*
     * Stop motion evaluation before taking the final sample so the final
     * heartbeat cannot restart the inactivity timer.
     */
    m_active = false;
    m_have_previous_sample = false;
    m_have_dump_filter = false;
    m_rolling_candidate = false;
    m_rolling = false;
    m_rolling_evidence_ms = 0U;

    /*
     * Per-sample evidence must not remain asserted in the final idle packet.
     * Upright remains the most recent analyzed orientation, consistent with
     * the other derived telemetry values.
     */
    m_telemetry.dump_evidence = false;
    m_telemetry.rolling_evidence = false;

    /*
     * Repeat the final IDLE advertisements while the fast 50 ms advertising
     * interval is still active. This gives both RAWv2 and F0 multiple chances
     * to be received without adding another transition state or heartbeat.
     */
    app_comms_bleadv_send_count_set (CART_IDLE_ADV_SEND_COUNT);
    app_heartbeat_now();

    /*
     * Return the heartbeat to low-power idle telemetry immediately, but leave
     * the advertiser at the active 50 ms interval long enough for the repeated
     * RAWv2 + F0 packets above to drain before restoring the stock advertising
     * configuration.
     */
    (void) app_heartbeat_interval_set (CART_IDLE_INTERVAL_MS);
    m_idle_restore_pending = true;
    (void) ri_timer_start (m_idle_timer, CART_IDLE_ADV_DRAIN_MS, NULL);
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

    if (!m_active)
    {
        m_last_motion_ms = ri_rtc_millis();
        cart_idle_timer_restart();

        /*
         * Discard any slow IDLE advertisement still in progress or queued.
         * ACTIVE telemetry supersedes stale IDLE telemetry, and allowing the
         * old 1285 ms advertisements to drain can fill the small advertising
         * queue before the 50 ms ACTIVE stream takes over.
         */
        (void) rt_adv_stop();

        /*
         * Enter active telemetry mode.
         */
        app_comms_bleadv_send_count_set (1U);
        app_comms_bleadv_interval_set (CART_ACTIVE_ADV_INTERVAL_MS);
        (void) app_heartbeat_interval_set (CART_ANALYSIS_INTERVAL_MS);

        m_active = true;
        m_have_previous_sample = false;
        m_have_dump_filter = false;

        /*
         * Generate fresh telemetry immediately rather than waiting for the
         * first CART_ANALYSIS_INTERVAL_MS heartbeat timer expiration.
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
        m_idle_restore_pending = false;
        m_active = false;
        m_have_previous_sample = false;
        m_have_upright_sample = false;
        m_have_dump_filter = false;

        cart_dump_candidate_reset();
        m_dump_latched = false;
        m_dump_armed = true;

        m_last_motion_ms = 0U;
        m_dump_min_hold_until_ms = 0U;
        m_rolling_candidate = false;
        m_rolling = false;

        m_rolling_evidence_ms = 0U;
        m_last_rolling_motion_ms = 0U;
        m_telemetry = (app_cart_motion_telemetry_t) {0};

        m_dump_filter_alpha =
            1.0F - expf (
                -((float) CART_ANALYSIS_INTERVAL_MS) /
                CART_DUMP_FILTER_TAU_MS);

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
        m_upright_mag =
            sqrtf ((x * x) + (y * y) + (z * z));
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
    app_comms_bleadv_interval_set (CART_ACTIVE_ADV_INTERVAL_MS);

    const uint64_t now_ms = ri_rtc_millis();

    /*
     * Use a low-pass filtered acceleration vector for DUMP orientation so
     * short wheel impacts and vibration cannot dominate the gravity estimate.
     * ROLLING and sample-to-sample motion continue to use the raw XYZ values.
     */
    const float sample_mag =
        sqrtf ((x * x) + (y * y) + (z * z));
    const float sample_g =
        sample_mag / m_upright_mag;
    float confidence = 1.0F;

    if (sample_g > CART_DUMP_CONFIDENCE_ONSET_G)
    {
        confidence =
            expf (
                -(sample_g - CART_DUMP_CONFIDENCE_ONSET_G) /
                CART_DUMP_CONFIDENCE_DECAY_G);
    }

    if (!m_have_dump_filter)
    {
        m_dump_filtered_x = x;
        m_dump_filtered_y = y;
        m_dump_filtered_z = z;
        m_have_dump_filter = true;
    }
    else
    {
        const float effective_alpha =
            m_dump_filter_alpha * confidence;

        m_dump_filtered_x +=
            effective_alpha * (x - m_dump_filtered_x);
        m_dump_filtered_y +=
            effective_alpha * (y - m_dump_filtered_y);
        m_dump_filtered_z +=
            effective_alpha * (z - m_dump_filtered_z);
    }

    const float angle_deg =
        cart_angle_from_reference_deg (
            x,
            y,
            z,
            m_upright_x,
            m_upright_y,
            m_upright_z);

    const float dump_angle_deg =
        cart_angle_from_reference_deg (
            m_dump_filtered_x,
            m_dump_filtered_y,
            m_dump_filtered_z,
            m_upright_x,
            m_upright_y,
            m_upright_z);

    const bool inverted =
        cart_is_inverted (dump_angle_deg);

    const bool dump_evidence =
        inverted &&
        (confidence >= CART_DUMP_MIN_HIT_CONFIDENCE);

    const bool dump_rearmed =
        cart_is_dump_rearmed (dump_angle_deg);

    const bool upright =
        cart_is_upright (dump_angle_deg);

    const bool rolling =
        cart_is_rolling (angle_deg);

    bool rolling_evidence = false;

    if (m_dump_latched)
    {
        /*
         * While the cart remains active, keep DUMP latched for
         * CART_DUMP_ACTIVE_HOLD_MS. After that interval, clear it only once the cart
         * has returned close to its normal upright orientation.
         *
         * The idle timeout may clear a stale DUMP sooner if the cart has been
         * completely quiet for CART_IDLE_TIMEOUT_MS.
         */
        if ( (now_ms >= m_dump_min_hold_until_ms) && dump_rearmed)
        {
            m_dump_latched = false;
            m_dump_armed = true;
        }
    }
    else if (m_dump_armed)
    {
        if (!m_dump_candidate)
        {
            if (dump_evidence)
            {
                cart_dump_candidate_start();
            }
        }
        else
        {
            m_dump_candidate_samples++;

            if (dump_evidence)
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
                    m_dump_min_hold_until_ms =
                        now_ms + CART_DUMP_ACTIVE_HOLD_MS;
                    app_comms_bleadv_send_count_set (1U);

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

                    if (dump_evidence)
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

        rolling_evidence =
            sample_moving && rolling;

        if (sample_moving)
        {
            m_last_motion_ms = now_ms;
            cart_idle_timer_restart();
        }

        if (rolling_evidence)
        {
            /*
             * A gap longer than CART_ROLLING_GAP_TOLERANCE_MS starts a new
             * sustained rolling-motion candidate. Shorter interruptions are
             * tolerated, but do not count toward CART_ROLLING_CONFIRM_MS.
             */
            if (m_rolling_candidate &&
                ((now_ms - m_last_rolling_motion_ms) >
                 CART_ROLLING_GAP_TOLERANCE_MS))
            {
                m_rolling_candidate = false;
                m_rolling_evidence_ms = 0U;
            }
            if ((!m_rolling) && (!m_rolling_candidate))
            {
                m_rolling_candidate = true;
                m_rolling_evidence_ms = 0U;
            }
            if (m_rolling_candidate)
            {
                m_rolling_evidence_ms += CART_ANALYSIS_INTERVAL_MS;

                if (m_rolling_evidence_ms >= CART_ROLLING_CONFIRM_MS)
                {
                    m_rolling_candidate = false;
                    m_rolling = true;
                }
            }
            m_last_rolling_motion_ms = now_ms;
        }
        else if ((m_rolling_candidate || m_rolling) &&
                 ((now_ms - m_last_rolling_motion_ms) >
                  CART_ROLLING_GAP_TOLERANCE_MS))
        {
            m_rolling_candidate = false;
            m_rolling = false;
            m_rolling_evidence_ms = 0U;
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

    m_telemetry.valid = true;
    m_telemetry.active = m_active;
    m_telemetry.dump_candidate = m_dump_candidate;
    m_telemetry.dump_evidence = dump_evidence;
    m_telemetry.dump_latched = m_dump_latched;
    m_telemetry.rolling_candidate = m_rolling_candidate;
    m_telemetry.rolling_evidence = rolling_evidence;
    m_telemetry.upright = upright;
    m_telemetry.status = app_cart_motion_status_get();
    m_telemetry.dump_candidate_hits = m_dump_candidate_hits;
    m_telemetry.dump_candidate_samples = m_dump_candidate_samples;
    m_telemetry.rolling_evidence_ms = m_rolling_evidence_ms;
    m_telemetry.raw_angle_deg = angle_deg;
    m_telemetry.filtered_angle_deg = dump_angle_deg;
    m_telemetry.sample_g = sample_g;
    m_telemetry.confidence = confidence;

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

bool app_cart_motion_active_get (void)
{
    return m_active;
}

bool app_cart_motion_telemetry_get (
    app_cart_motion_telemetry_t * const p_telemetry)
{
    if (NULL == p_telemetry)
    {
        return false;
    }

    *p_telemetry = m_telemetry;
    p_telemetry->active = m_active;
    p_telemetry->dump_candidate = m_dump_candidate;
    p_telemetry->dump_latched = m_dump_latched;
    p_telemetry->rolling_candidate = m_rolling_candidate;
    p_telemetry->status = app_cart_motion_status_get();
    p_telemetry->dump_candidate_hits = m_dump_candidate_hits;
    p_telemetry->dump_candidate_samples = m_dump_candidate_samples;
    p_telemetry->rolling_evidence_ms = m_rolling_evidence_ms;
    return m_telemetry.valid;
}
