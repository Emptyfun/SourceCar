#include "app_motor_auto_test.h"

#include "app_motion.h"
#include "app_motor_serial.h"
#include "stm32f1xx_hal.h"

#include <stdint.h>
#include <string.h>

#define AUTO_TEST_SPEED 100
#define AUTO_TEST_RUN_MS 3000U
#define AUTO_TEST_STOP_MS 1000U
#define AUTO_TEST_INIT_SETTLE_MS 700U
#define AUTO_TEST_PRINT_LINE_MS 50U

typedef enum
{
    AUTO_TEST_WAIT_INIT = 0,
    AUTO_TEST_SETTLE,
    AUTO_TEST_RUN_ACTION,
    AUTO_TEST_PRINT_SUMMARY,
    AUTO_TEST_DONE
} AutoTestState_t;

typedef struct
{
    const char *name;
    int16_t vx;
    int16_t wz;
    uint32_t duration_ms;
    uint8_t collect;
    int8_t expected_sign[4];
} AutoTestAction_t;

typedef struct
{
    int32_t total[4];
    int32_t min_delta[4];
    int32_t max_delta[4];
    uint16_t nonzero[4];
    uint16_t samples;
    uint8_t has_data;
} AutoTestStats_t;

static const AutoTestAction_t s_actions[] = {
    {"STOP_BASE", 0, 0, AUTO_TEST_STOP_MS, 1U, {0, 0, 0, 0}},
    {"FORWARD", AUTO_TEST_SPEED, 0, AUTO_TEST_RUN_MS, 1U, {1, -1, 1, -1}},
    {"STOP_FWD", 0, 0, AUTO_TEST_STOP_MS, 0U, {0, 0, 0, 0}},
    {"BACKWARD", -AUTO_TEST_SPEED, 0, AUTO_TEST_RUN_MS, 1U, {-1, 1, -1, 1}},
    {"STOP_BACK", 0, 0, AUTO_TEST_STOP_MS, 0U, {0, 0, 0, 0}},
    {"LEFT", 0, -AUTO_TEST_SPEED, AUTO_TEST_RUN_MS, 1U, {-1, -1, 1, 1}},
    {"STOP_LEFT", 0, 0, AUTO_TEST_STOP_MS, 0U, {0, 0, 0, 0}},
    {"RIGHT", 0, AUTO_TEST_SPEED, AUTO_TEST_RUN_MS, 1U, {1, 1, -1, -1}},
    {"STOP_RIGHT", 0, 0, AUTO_TEST_STOP_MS, 0U, {0, 0, 0, 0}}
};

static AutoTestState_t s_state;
static uint8_t s_action_index;
static uint8_t s_result_count;
static uint32_t s_state_tick;
static uint32_t s_last_delta_seq;
static uint32_t s_last_print_tick;
static uint8_t s_print_line;
static AutoTestStats_t s_current;
static AutoTestStats_t s_results[5];
static const AutoTestAction_t *s_result_actions[5];

static void AutoTest_StartAction(uint8_t index);
static void AutoTest_CollectLatestDelta(void);
static void AutoTest_FinishAction(void);
static void AutoTest_BeginSummaryPrint(void);
static void AutoTest_PrintSummaryTask(uint32_t now);
static void AutoTest_PrintActionHeader(const AutoTestAction_t *action,
                                       const AutoTestStats_t *stats);
static void AutoTest_PrintMotorLine(const AutoTestAction_t *action,
                                    const AutoTestStats_t *stats,
                                    uint8_t motor_index);
static void AutoTest_ResetStats(AutoTestStats_t *stats);
static void AutoTest_AddDelta(AutoTestStats_t *stats, const int32_t delta[4]);
static int32_t AutoTest_Abs32(int32_t value);
static char AutoTest_SignChar(int32_t value);
static char AutoTest_ExpectedSignChar(int8_t value);
static const char *AutoTest_MapResult(const AutoTestAction_t *action,
                                      const AutoTestStats_t *stats);

void App_MotorAutoTest_Init(void)
{
    s_state = AUTO_TEST_WAIT_INIT;
    s_action_index = 0U;
    s_result_count = 0U;
    s_state_tick = HAL_GetTick();
    s_last_delta_seq = 0UL;
    s_last_print_tick = 0UL;
    s_print_line = 0U;
    AutoTest_ResetStats(&s_current);
    memset(s_results, 0, sizeof(s_results));
    memset(s_result_actions, 0, sizeof(s_result_actions));
    MotorSerial_Printf("[AUTO] wheel-off motor statistic test armed\r\n");
}

void App_MotorAutoTest_Task(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_state)
    {
    case AUTO_TEST_WAIT_INIT:
        if (MotorSerial_IsInitDone() != 0U)
        {
            MotorSerial_Printf("[AUTO] init done, settling\r\n");
            Motion_Stop();
            MotorSerial_ResetFeedback();
            MotorSerial_SendUploadMode(1U, 0U, 0U);
            s_state_tick = now;
            s_state = AUTO_TEST_SETTLE;
        }
        break;

    case AUTO_TEST_SETTLE:
        if ((uint32_t)(now - s_state_tick) >= AUTO_TEST_INIT_SETTLE_MS)
        {
            MotorSerial_ResetFeedback();
            s_last_delta_seq = MotorSerial_GetDeltaSequence();
            AutoTest_StartAction(0U);
        }
        break;

    case AUTO_TEST_RUN_ACTION:
        AutoTest_CollectLatestDelta();
        if ((uint32_t)(now - s_state_tick) >= s_actions[s_action_index].duration_ms)
        {
            AutoTest_FinishAction();
            s_action_index++;
            if (s_action_index >= (uint8_t)(sizeof(s_actions) / sizeof(s_actions[0])))
            {
                Motion_Stop();
                MotorSerial_SendUploadMode(0U, 0U, 0U);
                AutoTest_BeginSummaryPrint();
            }
            else
            {
                AutoTest_StartAction(s_action_index);
            }
        }
        break;

    case AUTO_TEST_PRINT_SUMMARY:
        AutoTest_PrintSummaryTask(now);
        break;

    case AUTO_TEST_DONE:
    default:
        break;
    }
}

static void AutoTest_StartAction(uint8_t index)
{
    const AutoTestAction_t *action = &s_actions[index];

    AutoTest_ResetStats(&s_current);
    s_last_delta_seq = MotorSerial_GetDeltaSequence();
    s_state_tick = HAL_GetTick();

    if ((action->vx == 0) && (action->wz == 0))
    {
        Motion_Stop();
    }
    else
    {
        Motion_SetVxWz(action->vx, action->wz);
    }

    MotorSerial_Printf("[AUTO] step=%s vx=%d wz=%d ms=%lu\r\n",
                       action->name,
                       (int)action->vx,
                       (int)action->wz,
                       (unsigned long)action->duration_ms);
    s_state = AUTO_TEST_RUN_ACTION;
}

static void AutoTest_CollectLatestDelta(void)
{
    const MotorFeedback_t *feedback = MotorSerial_GetFeedback();
    uint32_t seq = MotorSerial_GetDeltaSequence();

    if ((feedback == 0) || (feedback->has_encoder_delta == 0U))
    {
        return;
    }

    if (seq == s_last_delta_seq)
    {
        return;
    }

    s_last_delta_seq = seq;
    AutoTest_AddDelta(&s_current, feedback->encoder_delta_10ms);
}

static void AutoTest_FinishAction(void)
{
    const AutoTestAction_t *action = &s_actions[s_action_index];

    Motion_Stop();
    AutoTest_CollectLatestDelta();

    if ((action->collect != 0U) &&
        (s_result_count < (uint8_t)(sizeof(s_results) / sizeof(s_results[0]))))
    {
        s_results[s_result_count] = s_current;
        s_result_actions[s_result_count] = action;
        s_result_count++;
    }
}

static void AutoTest_BeginSummaryPrint(void)
{
    s_print_line = 0U;
    s_last_print_tick = 0UL;
    s_state = AUTO_TEST_PRINT_SUMMARY;
}

static void AutoTest_PrintSummaryTask(uint32_t now)
{
    uint8_t result_index;
    uint8_t sub_line;
    uint8_t total_lines = (uint8_t)(2U + (s_result_count * 5U) + 1U);

    if ((uint32_t)(now - s_last_print_tick) < AUTO_TEST_PRINT_LINE_MS)
    {
        return;
    }
    s_last_print_tick = now;

    if (s_print_line == 0U)
    {
        MotorSerial_Printf("\r\n[AUTO] ===== MOTOR STAT RESULT =====\r\n");
    }
    else if (s_print_line == 1U)
    {
        MotorSerial_Printf("[AUTO] speed=%d run_ms=%lu stop_ms=%lu upload=MAll delta\r\n",
                       AUTO_TEST_SPEED,
                       (unsigned long)AUTO_TEST_RUN_MS,
                       (unsigned long)AUTO_TEST_STOP_MS);
    }
    else if (s_print_line < (uint8_t)(total_lines - 1U))
    {
        result_index = (uint8_t)((s_print_line - 2U) / 5U);
        sub_line = (uint8_t)((s_print_line - 2U) % 5U);
        if (sub_line == 0U)
        {
            AutoTest_PrintActionHeader(s_result_actions[result_index], &s_results[result_index]);
        }
        else
        {
            AutoTest_PrintMotorLine(s_result_actions[result_index],
                                    &s_results[result_index],
                                    (uint8_t)(sub_line - 1U));
        }
    }
    else
    {
        MotorSerial_Printf("[AUTO] done, motor stopped, upload off\r\n");
        s_state = AUTO_TEST_DONE;
    }

    s_print_line++;
}

static void AutoTest_PrintActionHeader(const AutoTestAction_t *action,
                                       const AutoTestStats_t *stats)
{
    uint8_t i;
    int32_t max_abs = 0;
    int32_t min_abs = 0;
    uint8_t min_set = 0U;
    uint32_t balance_pct = 0UL;

    if ((action == 0) || (stats == 0))
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        int32_t abs_total = AutoTest_Abs32(stats->total[i]);
        if (abs_total > max_abs)
        {
            max_abs = abs_total;
        }
        if ((min_set == 0U) || (abs_total < min_abs))
        {
            min_abs = abs_total;
            min_set = 1U;
        }
    }

    if (max_abs > 0)
    {
        balance_pct = (uint32_t)(((max_abs - min_abs) * 100L) / max_abs);
    }

    MotorSerial_Printf("[AUTO] %s samples=%u map=%s balance=%lu%%\r\n",
                       action->name,
                       (unsigned int)stats->samples,
                       AutoTest_MapResult(action, stats),
                       (unsigned long)balance_pct);
}

static void AutoTest_PrintMotorLine(const AutoTestAction_t *action,
                                    const AutoTestStats_t *stats,
                                    uint8_t motor_index)
{
    int32_t avg_x100 = 0;

    if ((action == 0) || (stats == 0) || (motor_index >= 4U))
    {
        return;
    }

    if (stats->samples != 0U)
    {
        avg_x100 = (int32_t)(((int64_t)stats->total[motor_index] * 100L) /
                             (int64_t)stats->samples);
    }

    MotorSerial_Printf("[AUTO] %s M%u total=%ld avg=%ld.%02ld min=%ld max=%ld nz=%u sign=%c exp=%c\r\n",
                       action->name,
                       (unsigned int)(motor_index + 1U),
                       (long)stats->total[motor_index],
                       (long)(avg_x100 / 100L),
                       (long)AutoTest_Abs32(avg_x100 % 100L),
                       (long)stats->min_delta[motor_index],
                       (long)stats->max_delta[motor_index],
                       (unsigned int)stats->nonzero[motor_index],
                       AutoTest_SignChar(stats->total[motor_index]),
                       AutoTest_ExpectedSignChar(action->expected_sign[motor_index]));
}

static void AutoTest_ResetStats(AutoTestStats_t *stats)
{
    if (stats != 0)
    {
        memset(stats, 0, sizeof(*stats));
    }
}

static void AutoTest_AddDelta(AutoTestStats_t *stats, const int32_t delta[4])
{
    uint8_t i;

    if ((stats == 0) || (delta == 0))
    {
        return;
    }

    for (i = 0U; i < 4U; i++)
    {
        if (stats->has_data == 0U)
        {
            stats->min_delta[i] = delta[i];
            stats->max_delta[i] = delta[i];
        }
        else
        {
            if (delta[i] < stats->min_delta[i])
            {
                stats->min_delta[i] = delta[i];
            }
            if (delta[i] > stats->max_delta[i])
            {
                stats->max_delta[i] = delta[i];
            }
        }

        stats->total[i] += delta[i];
        if (delta[i] != 0)
        {
            stats->nonzero[i]++;
        }
    }

    stats->samples++;
    stats->has_data = 1U;
}

static int32_t AutoTest_Abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static char AutoTest_SignChar(int32_t value)
{
    if (value > 0)
    {
        return '+';
    }
    if (value < 0)
    {
        return '-';
    }
    return '0';
}

static char AutoTest_ExpectedSignChar(int8_t value)
{
    if (value > 0)
    {
        return '+';
    }
    if (value < 0)
    {
        return '-';
    }
    return '0';
}

static const char *AutoTest_MapResult(const AutoTestAction_t *action,
                                      const AutoTestStats_t *stats)
{
    uint8_t i;

    if ((action == 0) || (stats == 0) || (stats->samples == 0U))
    {
        return "NO_DATA";
    }

    for (i = 0U; i < 4U; i++)
    {
        if (action->expected_sign[i] == 0)
        {
            continue;
        }
        if (AutoTest_SignChar(stats->total[i]) != AutoTest_ExpectedSignChar(action->expected_sign[i]))
        {
            return "CHECK";
        }
    }

    return "OK";
}
