#include "app_magnet_detect.h"

#include "app_config.h"
#include "app_cs1238_task.h"
#if APP_ENABLE_OLED
#include "app_oled_ui.h"
#endif
#if APP_ENABLE_RAW_CAPTURE
#include "app_raw_capture.h"
#endif
#include "stm32f1xx_hal.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define MAG_CALIB_SAMPLES 24U
#define MAG_CALIB_TIMEOUT_MS 6000U
#define MAG_PRINT_PERIOD_MS 200U
#define MAG_UPDATE_PERIOD_MS 50U
#define MAG_SIDE_DETECT_THRESHOLD_RAW 70000L
#define MAG_SIDE_CENTER_MIN_RAW 90000L
#define MAG_SIDE_NEAREST_MARGIN_RAW 70000L
#define MAG_SIDE_CENTER_MARGIN_RAW 50000L
#define MAG_RAW_GLITCH_LIMIT 1000000L
#define MAG_RAW_ABS_LIMIT 8000000L
#define MAG_IIR_DIV 4L
#define MAG_DECISION_CONFIRM_COUNT 2U
#define MAG_RECOVER_OK_COUNT 5U

static MagDetectInfo_t s_mag_info;
static uint8_t s_have_clean_raw[MAG_DETECT_CHANNEL_COUNT];
static uint8_t s_recalibration_requested;
static uint8_t s_recover_ok_count;
static uint8_t s_pending_count;
static uint32_t s_calib_start_tick;
static uint32_t s_last_print_tick;
static uint32_t s_last_update_tick;
static int64_t s_calib_sum[MAG_DETECT_CHANNEL_COUNT];
static uint32_t s_calib_count;
static int32_t s_last_clean_raw[MAG_DETECT_CHANNEL_COUNT];
static MagTarget_t s_pending_target;

static void Mag_BeginCalibration(void);
static void Mag_UpdateCalibration(const AppCS1238Snapshot_t *snapshot, uint32_t now);
static void Mag_UpdateReady(const AppCS1238Snapshot_t *snapshot, uint32_t now);
static void Mag_UpdateSensorError(const AppCS1238Snapshot_t *snapshot, uint32_t now);
static uint8_t Mag_SnapshotReady(const AppCS1238Snapshot_t *snapshot);
static uint8_t Mag_GetCleanRawSet(const AppCS1238Snapshot_t *snapshot,
                                  int32_t raw[MAG_DETECT_CHANNEL_COUNT]);
static uint8_t Mag_AcceptCleanRaw(uint8_t *has_last, int32_t *last_raw, int32_t raw);
static void Mag_UpdateStrengths(const int32_t raw[MAG_DETECT_CHANNEL_COUNT]);
static MagTarget_t Mag_ClassifyTarget(void);
static void Mag_ApplyCandidate(MagTarget_t candidate);
static void Mag_SetImmediateTarget(MagTarget_t target);
static MagSignal_t Mag_TargetToSignal(MagTarget_t target);
static int32_t Mag_Abs32(int32_t value);
static int32_t Mag_Max32(int32_t a, int32_t b);
static void Mag_UpdateConfidence(void);
static void Mag_UpdateOLED(void);
static void Mag_PrintStatus(const AppCS1238Snapshot_t *snapshot, uint32_t now);
static const char *MagStateToStr(MagState_t state);
static const char *MagTargetToStr(MagTarget_t target);
static const char *MagSignalToStr(MagSignal_t signal);
static const char *MagCSStatusToStr(AppCS1238ChStatus_t status);
static void Mag_DebugPrintf(const char *fmt, ...);

void App_MagDetect_Init(void)
{
    memset(&s_mag_info, 0, sizeof(s_mag_info));
    s_mag_info.state = MAG_STATE_INIT;
    s_mag_info.target = MAG_TARGET_UNKNOWN;
    s_mag_info.candidate = MAG_TARGET_UNKNOWN;
    s_mag_info.signal = MAG_SIGNAL_IDLE;
    Mag_BeginCalibration();
    Mag_UpdateOLED();
}

void App_MagDetect_Task(void)
{
    AppCS1238Snapshot_t snapshot;
    uint32_t now = HAL_GetTick();

    if (s_recalibration_requested != 0U)
    {
        s_recalibration_requested = 0U;
        Mag_BeginCalibration();
    }

    if ((uint32_t)(now - s_last_update_tick) < MAG_UPDATE_PERIOD_MS)
    {
        return;
    }
    s_last_update_tick = now;

    App_CS1238_GetSnapshot(&snapshot);

    switch (s_mag_info.state)
    {
    case MAG_STATE_CALIBRATING:
        Mag_UpdateCalibration(&snapshot, now);
        break;

    case MAG_STATE_READY:
        Mag_UpdateReady(&snapshot, now);
        break;

    case MAG_STATE_SENSOR_ERROR:
        Mag_UpdateSensorError(&snapshot, now);
        break;

    case MAG_STATE_INIT:
    default:
        Mag_BeginCalibration();
        break;
    }

    Mag_PrintStatus(&snapshot, now);
    Mag_UpdateOLED();
}

void App_MagDetect_RequestRecalibration(void)
{
    s_recalibration_requested = 1U;
}

const MagDetectInfo_t *App_MagDetect_GetInfo(void)
{
    return &s_mag_info;
}

static void Mag_BeginCalibration(void)
{
    uint32_t now = HAL_GetTick();

    memset(s_calib_sum, 0, sizeof(s_calib_sum));
    memset(s_have_clean_raw, 0, sizeof(s_have_clean_raw));
    memset(s_last_clean_raw, 0, sizeof(s_last_clean_raw));
    memset(s_mag_info.raw, 0, sizeof(s_mag_info.raw));
    memset(s_mag_info.baseline, 0, sizeof(s_mag_info.baseline));
    memset(s_mag_info.delta, 0, sizeof(s_mag_info.delta));
    memset(s_mag_info.strength, 0, sizeof(s_mag_info.strength));
    memset(s_mag_info.filt_strength, 0, sizeof(s_mag_info.filt_strength));

    s_calib_count = 0UL;
    s_recover_ok_count = 0U;
    s_pending_count = 0U;
    s_pending_target = MAG_TARGET_NONE;
    s_calib_start_tick = now;
    s_last_print_tick = 0UL;

    s_mag_info.state = MAG_STATE_CALIBRATING;
    s_mag_info.target = MAG_TARGET_NONE;
    s_mag_info.candidate = MAG_TARGET_NONE;
    s_mag_info.signal = MAG_SIGNAL_IDLE;
    s_mag_info.detected = 0U;
    s_mag_info.holding = 0U;
    s_mag_info.confidence_percent = 0U;
    s_mag_info.valid_sample_count = 0UL;
    s_mag_info.left_strength = 0;
    s_mag_info.right_strength = 0;
    s_mag_info.last_update_tick = now;
}

static void Mag_UpdateCalibration(const AppCS1238Snapshot_t *snapshot, uint32_t now)
{
    int32_t raw[MAG_DETECT_CHANNEL_COUNT];
    uint8_t i;

    if (Mag_GetCleanRawSet(snapshot, raw) != 0U)
    {
        for (i = 0U; i < MAG_DETECT_CHANNEL_COUNT; i++)
        {
            s_mag_info.raw[i] = raw[i];
            s_calib_sum[i] += raw[i];
        }

        s_calib_count++;
        s_mag_info.valid_sample_count = s_calib_count;
        s_mag_info.last_update_tick = now;

        if (s_calib_count >= MAG_CALIB_SAMPLES)
        {
            for (i = 0U; i < MAG_DETECT_CHANNEL_COUNT; i++)
            {
                s_mag_info.baseline[i] = (int32_t)(s_calib_sum[i] / (int64_t)s_calib_count);
                s_mag_info.filt_strength[i] = 0;
            }

            s_mag_info.target = MAG_TARGET_NONE;
            s_mag_info.candidate = MAG_TARGET_NONE;
            s_mag_info.signal = MAG_SIGNAL_IDLE;
            s_mag_info.detected = 0U;
            s_mag_info.holding = 0U;
            s_mag_info.confidence_percent = 0U;
            s_mag_info.left_strength = 0;
            s_mag_info.right_strength = 0;
            s_mag_info.state = MAG_STATE_READY;

            Mag_DebugPrintf("[MAG] calib done baseA=%ld baseB=%ld baseC=%ld baseD=%ld\r\n",
                            (long)s_mag_info.baseline[MAG_CH_ADC1_A],
                            (long)s_mag_info.baseline[MAG_CH_ADC1_B],
                            (long)s_mag_info.baseline[MAG_CH_ADC2_A],
                            (long)s_mag_info.baseline[MAG_CH_ADC2_B]);
        }
        return;
    }

    if ((uint32_t)(now - s_calib_start_tick) >= MAG_CALIB_TIMEOUT_MS)
    {
        Mag_SetImmediateTarget(MAG_TARGET_UNKNOWN);
        s_mag_info.state = MAG_STATE_SENSOR_ERROR;
        s_mag_info.detected = 0U;
        s_mag_info.confidence_percent = 0U;
        s_recover_ok_count = 0U;
    }
}

static void Mag_UpdateReady(const AppCS1238Snapshot_t *snapshot, uint32_t now)
{
    int32_t raw[MAG_DETECT_CHANNEL_COUNT];
    MagTarget_t candidate;

    if (Mag_GetCleanRawSet(snapshot, raw) == 0U)
    {
        Mag_SetImmediateTarget(MAG_TARGET_UNKNOWN);
        s_mag_info.state = MAG_STATE_SENSOR_ERROR;
        s_mag_info.detected = 0U;
        s_mag_info.confidence_percent = 0U;
        s_recover_ok_count = 0U;
        return;
    }

    Mag_UpdateStrengths(raw);
    candidate = Mag_ClassifyTarget();
    Mag_ApplyCandidate(candidate);
    Mag_UpdateConfidence();

    s_mag_info.valid_sample_count++;
    s_mag_info.last_update_tick = now;
}

static void Mag_UpdateSensorError(const AppCS1238Snapshot_t *snapshot, uint32_t now)
{
    s_mag_info.detected = 0U;
    s_mag_info.holding = 0U;
    s_mag_info.confidence_percent = 0U;
    s_mag_info.last_update_tick = now;

    if (Mag_SnapshotReady(snapshot) != 0U)
    {
        if (s_recover_ok_count < MAG_RECOVER_OK_COUNT)
        {
            s_recover_ok_count++;
        }
        if (s_recover_ok_count >= MAG_RECOVER_OK_COUNT)
        {
            Mag_BeginCalibration();
        }
    }
    else
    {
        s_recover_ok_count = 0U;
    }
}

static uint8_t Mag_SnapshotReady(const AppCS1238Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return 0U;
    }

    if ((snapshot->adc1_a.status != APP_CS1238_CH_STATUS_OK) ||
        (snapshot->adc1_b.status != APP_CS1238_CH_STATUS_OK) ||
        (snapshot->adc2_a.status != APP_CS1238_CH_STATUS_OK) ||
        (snapshot->adc2_b.status != APP_CS1238_CH_STATUS_OK))
    {
        return 0U;
    }

    if ((snapshot->adc1_a.sample_count == 0U) ||
        (snapshot->adc1_b.sample_count == 0U) ||
        (snapshot->adc2_a.sample_count == 0U) ||
        (snapshot->adc2_b.sample_count == 0U))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t Mag_GetCleanRawSet(const AppCS1238Snapshot_t *snapshot,
                                  int32_t raw[MAG_DETECT_CHANNEL_COUNT])
{
    int32_t candidate[MAG_DETECT_CHANNEL_COUNT];
    uint8_t i;

    if (raw == NULL)
    {
        return 0U;
    }

    if (Mag_SnapshotReady(snapshot) == 0U)
    {
        return 0U;
    }

    candidate[MAG_CH_ADC1_A] = snapshot->adc1_a.raw_last;
    candidate[MAG_CH_ADC1_B] = snapshot->adc1_b.raw_last;
    candidate[MAG_CH_ADC2_A] = snapshot->adc2_a.raw_last;
    candidate[MAG_CH_ADC2_B] = snapshot->adc2_b.raw_last;

    for (i = 0U; i < MAG_DETECT_CHANNEL_COUNT; i++)
    {
        if (Mag_AcceptCleanRaw(&s_have_clean_raw[i], &s_last_clean_raw[i], candidate[i]) == 0U)
        {
            return 0U;
        }
    }

    for (i = 0U; i < MAG_DETECT_CHANNEL_COUNT; i++)
    {
        raw[i] = s_last_clean_raw[i];
    }

    return 1U;
}

static uint8_t Mag_AcceptCleanRaw(uint8_t *has_last, int32_t *last_raw, int32_t raw)
{
    if ((has_last == NULL) || (last_raw == NULL))
    {
        return 0U;
    }

    if (Mag_Abs32(raw) >= MAG_RAW_ABS_LIMIT)
    {
        return 0U;
    }

    if ((*has_last != 0U) && (Mag_Abs32(raw - *last_raw) > MAG_RAW_GLITCH_LIMIT))
    {
        return 0U;
    }

    *last_raw = raw;
    *has_last = 1U;
    return 1U;
}

static void Mag_UpdateStrengths(const int32_t raw[MAG_DETECT_CHANNEL_COUNT])
{
    uint8_t i;

    for (i = 0U; i < MAG_DETECT_CHANNEL_COUNT; i++)
    {
        s_mag_info.raw[i] = raw[i];
        s_mag_info.delta[i] = s_mag_info.raw[i] - s_mag_info.baseline[i];
        s_mag_info.strength[i] = Mag_Abs32(s_mag_info.delta[i]);
        s_mag_info.filt_strength[i] +=
            (s_mag_info.strength[i] - s_mag_info.filt_strength[i]) / MAG_IIR_DIV;
    }

    s_mag_info.left_strength = s_mag_info.filt_strength[MAG_CH_ADC1_A] +
                               s_mag_info.filt_strength[MAG_CH_ADC2_A];
    s_mag_info.right_strength = s_mag_info.filt_strength[MAG_CH_ADC1_B] +
                                s_mag_info.filt_strength[MAG_CH_ADC2_B];
}

static MagTarget_t Mag_ClassifyTarget(void)
{
    int32_t max_side = Mag_Max32(s_mag_info.left_strength, s_mag_info.right_strength);
    int32_t abs_diff = Mag_Abs32(s_mag_info.left_strength - s_mag_info.right_strength);

    if (max_side < MAG_SIDE_DETECT_THRESHOLD_RAW)
    {
        return MAG_TARGET_NONE;
    }

    if ((s_mag_info.left_strength >= MAG_SIDE_CENTER_MIN_RAW) &&
        (s_mag_info.right_strength >= MAG_SIDE_CENTER_MIN_RAW) &&
        (abs_diff <= MAG_SIDE_CENTER_MARGIN_RAW))
    {
        return MAG_TARGET_CENTER;
    }

    if (s_mag_info.left_strength > (s_mag_info.right_strength + MAG_SIDE_NEAREST_MARGIN_RAW))
    {
        return MAG_TARGET_LEFT;
    }

    if (s_mag_info.right_strength > (s_mag_info.left_strength + MAG_SIDE_NEAREST_MARGIN_RAW))
    {
        return MAG_TARGET_RIGHT;
    }

    return MAG_TARGET_UNKNOWN;
}

static void Mag_ApplyCandidate(MagTarget_t candidate)
{
    s_mag_info.candidate = candidate;
    s_mag_info.detected = (candidate == MAG_TARGET_NONE) ? 0U : 1U;
    s_mag_info.holding = 0U;

    if (candidate == MAG_TARGET_UNKNOWN)
    {
        if ((s_mag_info.target == MAG_TARGET_LEFT) ||
            (s_mag_info.target == MAG_TARGET_RIGHT) ||
            (s_mag_info.target == MAG_TARGET_CENTER))
        {
            s_mag_info.holding = 1U;
            s_mag_info.signal = Mag_TargetToSignal(s_mag_info.target);
            return;
        }

        Mag_SetImmediateTarget(MAG_TARGET_UNKNOWN);
        return;
    }

    if (candidate == s_mag_info.target)
    {
        s_pending_count = 0U;
        s_pending_target = candidate;
        s_mag_info.signal = Mag_TargetToSignal(s_mag_info.target);
        return;
    }

    if (candidate != s_pending_target)
    {
        s_pending_target = candidate;
        s_pending_count = 1U;
    }
    else if (s_pending_count < MAG_DECISION_CONFIRM_COUNT)
    {
        s_pending_count++;
    }

    if (s_pending_count >= MAG_DECISION_CONFIRM_COUNT)
    {
        Mag_SetImmediateTarget(candidate);
    }
    else
    {
        s_mag_info.holding = 1U;
        s_mag_info.signal = Mag_TargetToSignal(s_mag_info.target);
        if ((s_mag_info.target == MAG_TARGET_LEFT) ||
            (s_mag_info.target == MAG_TARGET_RIGHT) ||
            (s_mag_info.target == MAG_TARGET_CENTER))
        {
            s_mag_info.detected = 1U;
        }
    }
}

static void Mag_SetImmediateTarget(MagTarget_t target)
{
    s_mag_info.target = target;
    s_mag_info.candidate = target;
    s_mag_info.signal = Mag_TargetToSignal(target);
    s_mag_info.holding = 0U;
    s_pending_target = target;
    s_pending_count = 0U;
}

static MagSignal_t Mag_TargetToSignal(MagTarget_t target)
{
    switch (target)
    {
    case MAG_TARGET_LEFT:
        return MAG_SIGNAL_TURN_LEFT;

    case MAG_TARGET_RIGHT:
        return MAG_SIGNAL_TURN_RIGHT;

    case MAG_TARGET_CENTER:
        return MAG_SIGNAL_STRAIGHT;

    case MAG_TARGET_UNKNOWN:
        return MAG_SIGNAL_HOLD;

    case MAG_TARGET_NONE:
    default:
        return MAG_SIGNAL_IDLE;
    }
}

static int32_t Mag_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static int32_t Mag_Max32(int32_t a, int32_t b)
{
    return (a > b) ? a : b;
}

static void Mag_UpdateConfidence(void)
{
    int32_t max_side = Mag_Max32(s_mag_info.left_strength, s_mag_info.right_strength);
    int32_t abs_diff = Mag_Abs32(s_mag_info.left_strength - s_mag_info.right_strength);
    int32_t confidence = 0;

    if ((s_mag_info.target == MAG_TARGET_NONE) || (max_side <= 0))
    {
        s_mag_info.confidence_percent = 0U;
        return;
    }

    if (s_mag_info.target == MAG_TARGET_CENTER)
    {
        confidence = 100L - ((abs_diff * 100L) / MAG_SIDE_CENTER_MARGIN_RAW);
        if (confidence < 0)
        {
            confidence = 0;
        }
    }
    else
    {
        confidence = (abs_diff * 100L) / max_side;
        if (s_mag_info.holding != 0U)
        {
            confidence /= 2L;
        }
    }

    if (confidence > 100L)
    {
        confidence = 100L;
    }

    s_mag_info.confidence_percent = (uint8_t)confidence;
}

static void Mag_UpdateOLED(void)
{
#if APP_ENABLE_OLED
    App_OLED_SetMagnetDebug(MagTargetToStr(s_mag_info.target),
                            MagSignalToStr(s_mag_info.signal),
                            s_mag_info.detected,
                            s_mag_info.confidence_percent);
#endif
}

static void Mag_PrintStatus(const AppCS1238Snapshot_t *snapshot, uint32_t now)
{
#if APP_ENABLE_RAW_CAPTURE
    if (App_RawCapture_IsActive() != 0U)
    {
        return;
    }
#endif

    if ((uint32_t)(now - s_last_print_tick) < MAG_PRINT_PERIOD_MS)
    {
        return;
    }
    s_last_print_tick = now;

    if (s_mag_info.state == MAG_STATE_CALIBRATING)
    {
        Mag_DebugPrintf("[MAG] state=CAL n=%lu A=%ld B=%ld C=%ld D=%ld\r\n",
                        (unsigned long)s_calib_count,
                        (long)s_mag_info.raw[MAG_CH_ADC1_A],
                        (long)s_mag_info.raw[MAG_CH_ADC1_B],
                        (long)s_mag_info.raw[MAG_CH_ADC2_A],
                        (long)s_mag_info.raw[MAG_CH_ADC2_B]);
    }
    else if (s_mag_info.state == MAG_STATE_READY)
    {
        Mag_DebugPrintf("[MAG] side=%s cand=%s turn=%s det=%u hold=%u conf=%u L=%ld R=%ld A=%ld B=%ld C=%ld D=%ld\r\n",
                        MagTargetToStr(s_mag_info.target),
                        MagTargetToStr(s_mag_info.candidate),
                        MagSignalToStr(s_mag_info.signal),
                        (unsigned int)s_mag_info.detected,
                        (unsigned int)s_mag_info.holding,
                        (unsigned int)s_mag_info.confidence_percent,
                        (long)s_mag_info.left_strength,
                        (long)s_mag_info.right_strength,
                        (long)s_mag_info.filt_strength[MAG_CH_ADC1_A],
                        (long)s_mag_info.filt_strength[MAG_CH_ADC1_B],
                        (long)s_mag_info.filt_strength[MAG_CH_ADC2_A],
                        (long)s_mag_info.filt_strength[MAG_CH_ADC2_B]);
    }
    else if (s_mag_info.state == MAG_STATE_SENSOR_ERROR)
    {
        Mag_DebugPrintf("[MAG] state=ERR a=%s b=%s c=%s d=%s\r\n",
                        MagCSStatusToStr(snapshot->adc1_a.status),
                        MagCSStatusToStr(snapshot->adc1_b.status),
                        MagCSStatusToStr(snapshot->adc2_a.status),
                        MagCSStatusToStr(snapshot->adc2_b.status));
    }
    else
    {
        Mag_DebugPrintf("[MAG] state=%s\r\n", MagStateToStr(s_mag_info.state));
    }
}

static const char *MagStateToStr(MagState_t state)
{
    switch (state)
    {
    case MAG_STATE_CALIBRATING:
        return "CAL";

    case MAG_STATE_READY:
        return "READY";

    case MAG_STATE_SENSOR_ERROR:
        return "ERR";

    case MAG_STATE_INIT:
    default:
        return "INIT";
    }
}

static const char *MagTargetToStr(MagTarget_t target)
{
    switch (target)
    {
    case MAG_TARGET_LEFT:
        return "LEFT";

    case MAG_TARGET_RIGHT:
        return "RIGHT";

    case MAG_TARGET_CENTER:
        return "CENTER";

    case MAG_TARGET_UNKNOWN:
        return "UNKNOWN";

    case MAG_TARGET_NONE:
    default:
        return "NONE";
    }
}

static const char *MagSignalToStr(MagSignal_t signal)
{
    switch (signal)
    {
    case MAG_SIGNAL_TURN_LEFT:
        return "TURN_L";

    case MAG_SIGNAL_TURN_RIGHT:
        return "TURN_R";

    case MAG_SIGNAL_STRAIGHT:
        return "STRAIGHT";

    case MAG_SIGNAL_HOLD:
        return "HOLD";

    case MAG_SIGNAL_IDLE:
    default:
        return "IDLE";
    }
}

static const char *MagCSStatusToStr(AppCS1238ChStatus_t status)
{
    switch (status)
    {
    case APP_CS1238_CH_STATUS_OK:
        return "OK";

    case APP_CS1238_CH_STATUS_TIMEOUT:
        return "TO";

    case APP_CS1238_CH_STATUS_ERROR:
        return "ERR";

    case APP_CS1238_CH_STATUS_WAIT:
    default:
        return "WAIT";
    }
}

static void Mag_DebugPrintf(const char *fmt, ...)
{
#if APP_ENABLE_MAGNET_DEBUG_PRINT
    char buf[256];
    va_list args;
    int len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }

    if (len >= (int)sizeof(buf))
    {
        len = (int)sizeof(buf) - 1;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)len, 30U);
#else
    (void)fmt;
#endif
}
