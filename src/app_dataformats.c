#include "app_cart_motion.h"
#include "app_dataformats.h"
#include "app_sensor.h"
#include "ruuvi_endpoints.h"
#include "ruuvi_endpoint_3.h"
#include "ruuvi_endpoint_5.h"
#include "ruuvi_endpoint_8.h"
#include "ruuvi_endpoint_fa.h"
#include "ruuvi_interface_aes.h"
#include "ruuvi_interface_communication_ble_advertising.h"
#include "ruuvi_interface_communication_radio.h"
#include "ruuvi_interface_communication.h"
#include "ruuvi_task_adc.h"

#include <math.h>
#include <string.h>

#ifdef CEEDLING
#   define TESTABLE_STATIC
#else
#   define TESTABLE_STATIC static
#endif

#if (RE_8_ENABLED || RE_FA_ENABLED)
uint32_t app_data_encrypt (const uint8_t * const cleartext,
                           uint8_t * const ciphertext,
                           const size_t data_size,
                           const uint8_t * const key,
                           const size_t key_size)
{
    rd_status_t err_code = RD_SUCCESS;
    uint32_t ret_code = 0;

    if (16U != key_size)
    {
        err_code |= RD_ERROR_INVALID_LENGTH;
    }
    else
    {
        err_code |= ri_aes_ecb_128_encrypt (cleartext,
                                            ciphertext,
                                            key,
                                            data_size);
    }

    if (RD_SUCCESS != err_code)
    {
        ret_code = 1;
    }

    RD_ERROR_CHECK (err_code, ~RD_ERROR_FATAL);
    return ret_code;
}
#endif

static uint16_t ep_5_measurement_count;

app_dataformat_t app_dataformat_next (const app_dataformats_t formats,
                                      const app_dataformat_t state)
{
    // TODO: Return enabled value instead of hardcoded one
    return DF_5;
}

#if RE_3_ENABLED
TESTABLE_STATIC rd_status_t
encode_to_3 (uint8_t * const output,
             size_t * const output_length,
             const rd_sensor_data_t * const data)
{
    rd_status_t err_code = RD_SUCCESS;
    re_status_t enc_code = RE_SUCCESS;
    re_3_data_t ep_data = {0};
    ep_data.accelerationx_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_X_FIELD);
    ep_data.accelerationy_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_Y_FIELD);
    ep_data.accelerationz_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_Z_FIELD);
    ep_data.humidity_rh       = rd_sensor_data_parse (data, RD_SENSOR_HUMI_FIELD);
    ep_data.pressure_pa       = rd_sensor_data_parse (data, RD_SENSOR_PRES_FIELD);
    ep_data.temperature_c     = rd_sensor_data_parse (data, RD_SENSOR_TEMP_FIELD);
    err_code |= rt_adc_vdd_get (&ep_data.battery_v);
    enc_code |= re_3_encode (output, &ep_data, RD_FLOAT_INVALID);

    if (RE_SUCCESS != enc_code)
    {
        err_code |= RD_ERROR_INTERNAL;
    }

    *output_length = RE_3_DATA_LENGTH;
    return err_code;
}
#endif

#if RE_5_ENABLED
TESTABLE_STATIC rd_status_t
encode_to_5 (uint8_t * const output,
             size_t * const output_length,
             const rd_sensor_data_t * const data)
{
    rd_status_t err_code = RD_SUCCESS;
    re_status_t enc_code = RE_SUCCESS;
    re_5_data_t ep_data = {0};
    ep_5_measurement_count++;
    ep_5_measurement_count %= (RE_5_SEQCTR_MAX + 1);
    ep_data.accelerationx_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_X_FIELD);
    ep_data.accelerationy_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_Y_FIELD);
    ep_data.accelerationz_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_Z_FIELD);
    ep_data.humidity_rh       = rd_sensor_data_parse (data, RD_SENSOR_HUMI_FIELD);
    ep_data.pressure_pa       = rd_sensor_data_parse (data, RD_SENSOR_PRES_FIELD);
    ep_data.temperature_c     = rd_sensor_data_parse (data, RD_SENSOR_TEMP_FIELD);
    ep_data.measurement_count = ep_5_measurement_count;

    /*
     * DumpSense status/report byte encoded into RAWv2 movement_count:
     *
     *   status 0: NORMAL
     *   status 1: DUMP
     *   status 2: ROLLING
     *   status 3: reserved
     *
     * Bits 5..0 contain the rolling report sequence.
     */
    const uint8_t report_seq =
        (uint8_t) (ep_5_measurement_count & 0x3FU);

    const uint8_t status =
        app_cart_motion_status_get();

    ep_data.movement_count =
        (uint8_t) ((status << 6U) | report_seq);

    err_code |= ri_radio_address_get (&ep_data.address);
    err_code |= ri_adv_tx_power_get (&ep_data.tx_power);
    err_code |= rt_adc_vdd_get (&ep_data.battery_v);
    enc_code |= re_5_encode (output, &ep_data);

    if (RE_SUCCESS != enc_code)
    {
        err_code |= RD_ERROR_INTERNAL;
    }

    *output_length = RE_5_DATA_LENGTH;
    return err_code;
}
#endif

#if RE_8_ENABLED
#ifndef APP_8_KEY
// "RuuviComRuuviTag"
#define APP_8_KEY { 0x52, 0x75, 0x75, 0x76, 0x69, 0x43, 0x6F, 0x6D, 0x52, 0x75, 0x75, 0x76, 0x69, 0x54, 0x61, 0x67}
#endif
static const uint8_t ep_8_key[RE_8_CIPHERTEXT_LENGTH] = APP_8_KEY;

TESTABLE_STATIC rd_status_t
ep_8_key_generate (uint8_t * const key)
{
    rd_status_t err_code = RD_SUCCESS;
    memcpy (key, ep_8_key, RE_8_CIPHERTEXT_LENGTH);
    uint64_t device_id = 0;
    err_code |= ri_comm_id_get (&device_id);

    for (uint8_t ii = 0U; ii < 8; ii++)
    {
        key[ii] = key[ii] ^ ( (device_id >> (ii * 8U)) & 0xFFU);
    }

    return err_code;
}

TESTABLE_STATIC rd_status_t
encode_to_8 (uint8_t * const output,
             size_t * const output_length,
             const rd_sensor_data_t * const data)
{
    static uint16_t ep_8_measurement_count = 0;
    uint8_t final_key[RE_8_CIPHERTEXT_LENGTH] = { 0 };
    rd_status_t err_code = RD_SUCCESS;
    re_status_t enc_code = RE_SUCCESS;
    re_8_data_t ep_data = {0};
    ep_8_measurement_count++;
    ep_8_measurement_count %= (RE_8_SEQCTR_MAX + 1);
    ep_data.humidity_rh       = rd_sensor_data_parse (data, RD_SENSOR_HUMI_FIELD);
    ep_data.pressure_pa       = rd_sensor_data_parse (data, RD_SENSOR_PRES_FIELD);
    ep_data.temperature_c     = rd_sensor_data_parse (data, RD_SENSOR_TEMP_FIELD);
    ep_data.message_counter = ep_8_measurement_count;
    uint8_t mvtctr = (uint8_t) (app_sensor_event_count_get() % (RE_8_MVTCTR_MAX + 1));
    ep_data.movement_count    = mvtctr;
    err_code |= ep_8_key_generate (final_key);
    err_code |= ri_radio_address_get (&ep_data.address);
    err_code |= ri_adv_tx_power_get (&ep_data.tx_power);
    err_code |= rt_adc_vdd_get (&ep_data.battery_v);
    enc_code |= re_8_encode (output,
                             &ep_data,
                             &app_data_encrypt,
                             final_key,
                             RE_8_CIPHERTEXT_LENGTH);

    if (RE_SUCCESS != enc_code)
    {
        err_code |= RD_ERROR_INTERNAL;
    }

    *output_length = RE_5_DATA_LENGTH;
    return err_code;
}
#endif

#if RE_FA_ENABLED
#ifndef APP_FA_KEY
#define APP_FA_KEY {00, 11, 22, 33, 44, 55, 66, 77, 88, 99, 11, 12, 13, 14, 15, 16}
#endif
static const uint8_t ep_fa_key[RE_FA_CIPHERTEXT_LENGTH] = APP_FA_KEY;

TESTABLE_STATIC rd_status_t
encode_to_fa (uint8_t * const output,
              size_t * const output_length,
              const rd_sensor_data_t * const data)
{
    static uint8_t ep_fa_measurement_count = 0;
    rd_status_t err_code = RD_SUCCESS;
    re_status_t enc_code = RE_SUCCESS;
    re_fa_data_t ep_data = {0};
    ep_fa_measurement_count++;
    ep_fa_measurement_count %= 0xFFU;
    ep_data.accelerationx_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_X_FIELD);
    ep_data.accelerationy_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_Y_FIELD);
    ep_data.accelerationz_g   = rd_sensor_data_parse (data, RD_SENSOR_ACC_Z_FIELD);
    ep_data.humidity_rh       = rd_sensor_data_parse (data, RD_SENSOR_HUMI_FIELD);
    ep_data.pressure_pa       = rd_sensor_data_parse (data, RD_SENSOR_PRES_FIELD);
    ep_data.temperature_c     = rd_sensor_data_parse (data, RD_SENSOR_TEMP_FIELD);
    ep_data.message_counter   = ep_fa_measurement_count;
    err_code |= rt_adc_vdd_get (&ep_data.battery_v);
    err_code |= ri_radio_address_get (&ep_data.address);
    enc_code |= re_fa_encode (output,
                              &ep_data,
                              &app_data_encrypt,
                              ep_fa_key,
                              RE_FA_CIPHERTEXT_LENGTH); //!< Cipher length == key lenght

    if (RE_SUCCESS != enc_code)
    {
        err_code |= RD_ERROR_INTERNAL;
    }

    *output_length = RE_FA_DATA_LENGTH;
    return err_code;
}
#endif

rd_status_t app_dataformat_encode (uint8_t * const output,
                                   size_t * const output_length,
                                   const rd_sensor_data_t * const p_data,
                                   const app_dataformat_t format)
{
    rd_status_t err_code = RD_SUCCESS;

    switch (format)
    {
#       if RE_3_ENABLED

        case DF_3:
            err_code |= encode_to_3 (output, output_length, p_data);
            break;
#       endif
#       if RE_5_ENABLED

        case DF_5:
            err_code |= encode_to_5 (output, output_length, p_data);
            break;
#       endif
#       if RE_8_ENABLED

        case DF_8:
            err_code |= encode_to_8 (output, output_length, p_data);
            break;
#       endif
#       if RE_FA_ENABLED

        case DF_FA:
            err_code |= encode_to_fa (output, output_length, p_data);
            break;
#       endif

        default:
            err_code |= RD_ERROR_NOT_ENABLED;
    }

    return err_code;
}


#define DUMPSENSE_FORMAT_ID          (0xF0U)
#define DUMPSENSE_SCHEMA_VERSION     (0x01U)
#define DUMPSENSE_DATA_LENGTH        (24U)
#define DUMPSENSE_ANGLE_SCALE        (100.0F)
#define DUMPSENSE_G_SCALE            (1000.0F)
#define DUMPSENSE_CONFIDENCE_SCALE   (255.0F)
#define DUMPSENSE_ROLLING_TICK_MS    (100U)

#define DUMPSENSE_FLAG_DUMP_CANDIDATE    (1U << 2U)
#define DUMPSENSE_FLAG_DUMP_EVIDENCE     (1U << 3U)
#define DUMPSENSE_FLAG_DUMP_LATCHED      (1U << 4U)
#define DUMPSENSE_FLAG_ROLLING_CANDIDATE (1U << 5U)
#define DUMPSENSE_FLAG_ROLLING_EVIDENCE  (1U << 6U)
#define DUMPSENSE_FLAG_UPRIGHT           (1U << 7U)

static uint8_t m_raw_adv_nomem_count;
static uint8_t m_f0_adv_nomem_count;
static uint8_t m_raw_adv_other_error_count;
static uint8_t m_f0_adv_other_error_count;
static uint16_t m_raw_adv_last_fail_sequence = 0xFFFFU;
static uint16_t m_f0_adv_last_fail_sequence = 0xFFFFU;

static void dumpsense_adv_diag_count_increment (uint8_t * const p_count)
{
    if (*p_count < 0xFFU)
    {
        (*p_count)++;
    }
}

static void dumpsense_adv_diag_record (
    const rd_status_t err_code,
    const uint16_t sequence,
    uint8_t * const p_nomem_count,
    uint8_t * const p_other_error_count,
    uint16_t * const p_last_fail_sequence)
{
    if (RD_SUCCESS == err_code)
    {
        return;
    }

    if (0U != (err_code & RD_ERROR_NO_MEM))
    {
        dumpsense_adv_diag_count_increment (p_nomem_count);
    }

    if (0U != (err_code & ~RD_ERROR_NO_MEM))
    {
        dumpsense_adv_diag_count_increment (p_other_error_count);
    }

    *p_last_fail_sequence = sequence;
}

void app_dataformat_adv_diag_record_raw (
    const rd_status_t err_code,
    const uint16_t sequence)
{
    dumpsense_adv_diag_record (
        err_code,
        sequence,
        &m_raw_adv_nomem_count,
        &m_raw_adv_other_error_count,
        &m_raw_adv_last_fail_sequence);
}

void app_dataformat_adv_diag_record_f0 (
    const rd_status_t err_code,
    const uint16_t sequence)
{
    dumpsense_adv_diag_record (
        err_code,
        sequence,
        &m_f0_adv_nomem_count,
        &m_f0_adv_other_error_count,
        &m_f0_adv_last_fail_sequence);
}

static void dumpsense_put_u16_be (
    uint8_t * const output,
    const size_t offset,
    const uint16_t value)
{
    output[offset] = (uint8_t) ((value >> 8U) & 0xFFU);
    output[offset + 1U] = (uint8_t) (value & 0xFFU);
}

static uint16_t dumpsense_u16_from_float (
    const float value,
    const float scale)
{
    if ((!isfinite (value)) || (value <= 0.0F))
    {
        return 0U;
    }

    const float scaled = value * scale;
    if (scaled >= 65535.0F)
    {
        return 65535U;
    }

    return (uint16_t) lroundf (scaled);
}

uint16_t app_dataformat_rawv2_sequence_get (void)
{
    return ep_5_measurement_count;
}

rd_status_t app_dataformat_encode_dumpsense (
    uint8_t * const output,
    size_t * const output_length)
{
    if ((NULL == output) || (NULL == output_length) ||
        (*output_length < DUMPSENSE_DATA_LENGTH))
    {
        return RD_ERROR_INVALID_PARAM;
    }

    app_cart_motion_telemetry_t telemetry = {0};
    (void) app_cart_motion_telemetry_get (&telemetry);

    memset (output, 0xFF, DUMPSENSE_DATA_LENGTH);

    output[0] = DUMPSENSE_FORMAT_ID;
    output[1] = DUMPSENSE_SCHEMA_VERSION;

    /*
     * Bytes 2..3: paired RAWv2 measurement sequence.
     *
     * heartbeat() encodes RAWv2 first, which advances the RAWv2 measurement
     * sequence, then encodes this F0 packet from the same already-analyzed
     * 100 ms sample. Both packets therefore share this sequence identifier.
     */
    dumpsense_put_u16_be (
        output,
        2U,
        app_dataformat_rawv2_sequence_get());

    dumpsense_put_u16_be (
        output,
        4U,
        dumpsense_u16_from_float (
            telemetry.raw_angle_deg,
            DUMPSENSE_ANGLE_SCALE));

    dumpsense_put_u16_be (
        output,
        6U,
        dumpsense_u16_from_float (
            telemetry.filtered_angle_deg,
            DUMPSENSE_ANGLE_SCALE));

    dumpsense_put_u16_be (
        output,
        8U,
        dumpsense_u16_from_float (
            telemetry.sample_g,
            DUMPSENSE_G_SCALE));

    uint8_t confidence_q8 = 0U;
    if (isfinite (telemetry.confidence) && (telemetry.confidence > 0.0F))
    {
        const float q =
            telemetry.confidence * DUMPSENSE_CONFIDENCE_SCALE;
        confidence_q8 =
            (uint8_t) ((q >= 255.0F) ? 255U : lroundf (q));
    }
    output[10] = confidence_q8;

    uint8_t operational_state = 0U;

    if (telemetry.dump_latched)
    {
        operational_state = 3U; /* DUMP */
    }
    else if (telemetry.status == APP_CART_STATUS_ROLLING)
    {
        operational_state = 2U; /* ROLLING */
    }
    else if (telemetry.active)
    {
        operational_state = 1U; /* ACTIVE / HANDLING */
    }
    else
    {
        operational_state = 0U; /* IDLE */
    }

    uint8_t state_flags = operational_state & 0x03U;

    if (telemetry.dump_candidate)
    {
        state_flags |= DUMPSENSE_FLAG_DUMP_CANDIDATE;
    }
    if (telemetry.dump_evidence)
    {
        state_flags |= DUMPSENSE_FLAG_DUMP_EVIDENCE;
    }
    if (telemetry.dump_latched)
    {
        state_flags |= DUMPSENSE_FLAG_DUMP_LATCHED;
    }
    if (telemetry.rolling_candidate)
    {
        state_flags |= DUMPSENSE_FLAG_ROLLING_CANDIDATE;
    }
    if (telemetry.rolling_evidence)
    {
        state_flags |= DUMPSENSE_FLAG_ROLLING_EVIDENCE;
    }
    if (telemetry.upright)
    {
        state_flags |= DUMPSENSE_FLAG_UPRIGHT;
    }
    output[11] = state_flags;

    output[12] = telemetry.dump_candidate_hits;
    output[13] = telemetry.dump_candidate_samples;

    uint32_t rolling_ticks =
        telemetry.rolling_evidence_ms / DUMPSENSE_ROLLING_TICK_MS;
    if (rolling_ticks > 65535U)
    {
        rolling_ticks = 65535U;
    }
    dumpsense_put_u16_be (output, 14U, (uint16_t) rolling_ticks);

    /*
     * Bytes 16..23: advertising queue diagnostics.
     *
     *   16      RAWv2 RD_ERROR_NO_MEM count
     *   17      F0 RD_ERROR_NO_MEM count
     *   18      RAWv2 other advertising error count
     *   19      F0 other advertising error count
     *   20..21  Last RAWv2 advertising failure sequence
     *   22..23  Last F0 advertising failure sequence
     *
     * Counts saturate at 255. A last-failure sequence of 0xFFFF means that
     * format has not had an advertising send failure since boot.
     */
    output[16] = m_raw_adv_nomem_count;
    output[17] = m_f0_adv_nomem_count;
    output[18] = m_raw_adv_other_error_count;
    output[19] = m_f0_adv_other_error_count;
    dumpsense_put_u16_be (output, 20U, m_raw_adv_last_fail_sequence);
    dumpsense_put_u16_be (output, 22U, m_f0_adv_last_fail_sequence);

    *output_length = DUMPSENSE_DATA_LENGTH;
    return RD_SUCCESS;
}
