#include "app_turn_debug.h"

#include "app_config.h"
#include "app_hmc5883l.h"
#include "app_motion.h"
#include "app_motor_serial.h"
#include "stm32f1xx_hal.h"

#include <stdlib.h>
#include <string.h>

#define TURNDBG_DEFAULT_KP             1.5f
#define TURNDBG_DEFAULT_MIN_WZ         70
#define TURNDBG_DEFAULT_MAX_WZ         150
#define TURNDBG_DEFAULT_STEP_WZ        10
#define TURNDBG_DEFAULT_TOL_DEG        5.0f
#define TURNDBG_DEFAULT_CTRL_PERIOD_MS 50U
#define TURNDBG_DEFAULT_LOG_ENABLE     1U
#define TURNDBG_HMC_TIMEOUT_MS         10000U
#define TURNDBG_HMC_LOG_PERIOD_MS      200U
#define TURNDBG_FIXED_LOG_PERIOD_MS    500U
#define TURNDBG_MAX_FIXED_MS           60000UL
#define TURNDBG_MAX_HMC_ANGLE_DEG      180.0f

typedef enum
{
    TURNDBG_STATE_IDLE = 0,
    TURNDBG_STATE_FIXED,
    TURNDBG_STATE_HMC_TURN,
    TURNDBG_STATE_DONE,
    TURNDBG_STATE_ERROR
} TurnDebugState_t;

static TurnDebugState_t s_state;
static float s_kp;
static float s_tol_deg;
static float s_target_deg;
static int16_t s_min_wz;
static int16_t s_max_wz;
static int16_t s_step_wz;
static int16_t s_target_wz;
static int16_t s_actual_wz;
static int16_t s_fixed_wz;
static uint8_t s_log_enable;
static uint32_t s_ctrl_period_ms;
static uint32_t s_start_tick;
static uint32_t s_last_ctrl_tick;
static uint32_t s_last_log_tick;
static uint32_t s_fixed_duration_ms;

static void TurnDebug_PrintHelp(void);
static void TurnDebug_PrintStatus(void);
static void TurnDebug_StartFixed(int8_t dir_sign, int32_t speed, uint32_t duration_ms);
static void TurnDebug_StartHmc(int8_t dir_sign, float angle_deg);
static void TurnDebug_TaskFixed(uint32_t now);
static void TurnDebug_TaskHmc(uint32_t now);
static void TurnDebug_StopOutput(const char *reason);
static void TurnDebug_LogFixed(uint32_t now, uint8_t force);
static void TurnDebug_LogHmc(uint32_t now,
                             float delta_deg,
                             float error_deg,
                             int16_t target_wz,
                             uint8_t force);
static float TurnDebug_WrapTo180(float deg);
static float TurnDebug_AbsFloat(float value);
static int16_t TurnDebug_AbsI16(int16_t value);
static int16_t TurnDebug_ClampI16(int32_t value, int16_t min_value, int16_t max_value);
static int16_t TurnDebug_RampToInt16(int16_t current, int16_t target, int16_t step);
static int16_t TurnDebug_RoundToI16(float value);
static long TurnDebug_FloatToDeci(float value);
static long TurnDebug_FloatToCenti(float value);
static long TurnDebug_AbsLong(long value);
static const char *TurnDebug_SignText(long value);
static uint8_t TurnDebug_StartsWith(const char *text, const char *prefix);
static const char *TurnDebug_SkipSpaces(const char *text);
static const char *TurnDebug_SkipWord(const char *text);
static int8_t TurnDebug_ParseDirection(const char **text);
static int32_t TurnDebug_ParseInt(const char *text, int32_t default_value, uint8_t *ok);
static int32_t TurnDebug_ParseFixed100(const char *text, int32_t default_value, uint8_t *ok);
static const char *TurnDebug_StateText(TurnDebugState_t state);

void App_TurnDebug_Init(void)
{
    s_state = TURNDBG_STATE_IDLE;
    s_kp = TURNDBG_DEFAULT_KP;
    s_tol_deg = TURNDBG_DEFAULT_TOL_DEG;
    s_min_wz = TURNDBG_DEFAULT_MIN_WZ;
    s_max_wz = TURNDBG_DEFAULT_MAX_WZ;
    s_step_wz = TURNDBG_DEFAULT_STEP_WZ;
    s_log_enable = TURNDBG_DEFAULT_LOG_ENABLE;
    s_ctrl_period_ms = TURNDBG_DEFAULT_CTRL_PERIOD_MS;
    s_target_deg = 0.0f;
    s_target_wz = 0;
    s_actual_wz = 0;
    s_fixed_wz = 0;
    s_start_tick = 0UL;
    s_last_ctrl_tick = 0UL;
    s_last_log_tick = 0UL;
    s_fixed_duration_ms = 0UL;

    MotorSerial_Printf("[TURNDBG] ready, type turn help\r\n");
}

void App_TurnDebug_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (s_state == TURNDBG_STATE_FIXED)
    {
        TurnDebug_TaskFixed(now);
    }
    else if (s_state == TURNDBG_STATE_HMC_TURN)
    {
        TurnDebug_TaskHmc(now);
    }
}

void App_TurnDebug_HandleCommand(const char *cmd)
{
    const char *arg = cmd;
    uint8_t ok = 0U;

    arg = TurnDebug_SkipWord(arg);
    arg = TurnDebug_SkipSpaces(arg);

    if ((*arg == '\0') || (strcmp(arg, "help") == 0))
    {
        TurnDebug_PrintHelp();
        return;
    }

    if (strcmp(arg, "status") == 0)
    {
        TurnDebug_PrintStatus();
        return;
    }

    if (strcmp(arg, "stop") == 0)
    {
        App_TurnDebug_Stop();
        return;
    }

    if (strcmp(arg, "ref") == 0)
    {
        App_HMC5883L_ResetDeltaReference();
        MotorSerial_Printf("[TURNDBG] hmc ref delta=0\r\n");
        return;
    }

    if (TurnDebug_StartsWith(arg, "log "))
    {
        int32_t value = TurnDebug_ParseInt(&arg[4], 1L, &ok);
        if (ok == 0U)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn log 0|1\r\n");
            return;
        }
        s_log_enable = (value != 0L) ? 1U : 0U;
        MotorSerial_Printf("[TURNDBG] log=%u\r\n", (unsigned int)s_log_enable);
        return;
    }

    if (TurnDebug_StartsWith(arg, "kp "))
    {
        int32_t kp_centi = TurnDebug_ParseFixed100(&arg[3], 150L, &ok);
        long kp_abs;

        if ((ok == 0U) || (kp_centi <= 0L) || (kp_centi > 1000L))
        {
            MotorSerial_Printf("[TURNDBG] usage: turn kp 1.5\r\n");
            return;
        }

        s_kp = (float)kp_centi / 100.0f;
        kp_abs = TurnDebug_AbsLong(TurnDebug_FloatToCenti(s_kp));
        MotorSerial_Printf("[TURNDBG] kp=%ld.%02ld\r\n", kp_abs / 100L, kp_abs % 100L);
        return;
    }

    if (TurnDebug_StartsWith(arg, "min "))
    {
        int32_t value = TurnDebug_ParseInt(&arg[4], (int32_t)s_min_wz, &ok);
        if (ok == 0U)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn min 70\r\n");
            return;
        }
        s_min_wz = TurnDebug_ClampI16(value, 1, 300);
        if (s_min_wz > s_max_wz)
        {
            s_max_wz = s_min_wz;
        }
        MotorSerial_Printf("[TURNDBG] min_wz=%d\r\n", (int)s_min_wz);
        return;
    }

    if (TurnDebug_StartsWith(arg, "max "))
    {
        int32_t value = TurnDebug_ParseInt(&arg[4], (int32_t)s_max_wz, &ok);
        if (ok == 0U)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn max 150\r\n");
            return;
        }
        s_max_wz = TurnDebug_ClampI16(value, 1, 300);
        if (s_min_wz > s_max_wz)
        {
            s_min_wz = s_max_wz;
        }
        MotorSerial_Printf("[TURNDBG] max_wz=%d\r\n", (int)s_max_wz);
        return;
    }

    if (TurnDebug_StartsWith(arg, "step "))
    {
        int32_t value = TurnDebug_ParseInt(&arg[5], (int32_t)s_step_wz, &ok);
        if (ok == 0U)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn step 10\r\n");
            return;
        }
        s_step_wz = TurnDebug_ClampI16(value, 1, 100);
        MotorSerial_Printf("[TURNDBG] step_wz=%d\r\n", (int)s_step_wz);
        return;
    }

    if (TurnDebug_StartsWith(arg, "tol "))
    {
        int32_t tol_centi = TurnDebug_ParseFixed100(&arg[4], 500L, &ok);
        long tol_abs;

        if ((ok == 0U) || (tol_centi <= 0L) || (tol_centi > 4500L))
        {
            MotorSerial_Printf("[TURNDBG] usage: turn tol 5\r\n");
            return;
        }

        s_tol_deg = (float)tol_centi / 100.0f;
        tol_abs = TurnDebug_AbsLong(TurnDebug_FloatToDeci(s_tol_deg));
        MotorSerial_Printf("[TURNDBG] tol=%ld.%01lddeg\r\n", tol_abs / 10L, tol_abs % 10L);
        return;
    }

    if (TurnDebug_StartsWith(arg, "period "))
    {
        int32_t value = TurnDebug_ParseInt(&arg[7], (int32_t)s_ctrl_period_ms, &ok);
        if (ok == 0U)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn period 50\r\n");
            return;
        }
        if (value < 10L)
        {
            value = 10L;
        }
        if (value > 1000L)
        {
            value = 1000L;
        }
        s_ctrl_period_ms = (uint32_t)value;
        MotorSerial_Printf("[TURNDBG] period=%lums\r\n", (unsigned long)s_ctrl_period_ms);
        return;
    }

    if (TurnDebug_StartsWith(arg, "fixed "))
    {
        const char *p = TurnDebug_SkipSpaces(&arg[6]);
        int8_t dir_sign = TurnDebug_ParseDirection(&p);
        int32_t speed;
        int32_t duration_ms;

        if (dir_sign == 0)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn fixed R 80 3000\r\n");
            return;
        }

        p = TurnDebug_SkipSpaces(p);
        speed = TurnDebug_ParseInt(p, 80L, &ok);
        if (ok == 0U)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn fixed R 80 3000\r\n");
            return;
        }
        p = TurnDebug_SkipWord(p);
        p = TurnDebug_SkipSpaces(p);
        duration_ms = TurnDebug_ParseInt(p, 3000L, &ok);
        if (ok == 0U)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn fixed R 80 3000\r\n");
            return;
        }
        if (duration_ms <= 0L)
        {
            duration_ms = 3000L;
        }
        if (duration_ms > (int32_t)TURNDBG_MAX_FIXED_MS)
        {
            duration_ms = (int32_t)TURNDBG_MAX_FIXED_MS;
        }

        TurnDebug_StartFixed(dir_sign, speed, (uint32_t)duration_ms);
        return;
    }

    if (TurnDebug_StartsWith(arg, "hmc "))
    {
        const char *p = TurnDebug_SkipSpaces(&arg[4]);
        int8_t dir_sign = TurnDebug_ParseDirection(&p);
        int32_t angle_centi;

        if (dir_sign == 0)
        {
            MotorSerial_Printf("[TURNDBG] usage: turn hmc R 90\r\n");
            return;
        }

        p = TurnDebug_SkipSpaces(p);
        angle_centi = TurnDebug_ParseFixed100(p, 9000L, &ok);
        if ((ok == 0U) || (angle_centi <= 0L))
        {
            MotorSerial_Printf("[TURNDBG] usage: turn hmc R 90\r\n");
            return;
        }

        TurnDebug_StartHmc(dir_sign, (float)angle_centi / 100.0f);
        return;
    }

    MotorSerial_Printf("[TURNDBG] unknown, type turn help\r\n");
}

void App_TurnDebug_Stop(void)
{
    TurnDebug_StopOutput("cmd");
    s_state = TURNDBG_STATE_IDLE;
    s_target_wz = 0;
    s_actual_wz = 0;
}

static void TurnDebug_PrintHelp(void)
{
    MotorSerial_Printf("[TURNDBG] commands:\r\n");
    MotorSerial_Printf("[TURNDBG] turn status | turn stop | turn ref | turn log 0|1\r\n");
    MotorSerial_Printf("[TURNDBG] turn fixed R 80 3000 | turn fixed L 80 3000\r\n");
    MotorSerial_Printf("[TURNDBG] turn hmc R 90 | turn hmc L 90 | turn hmc R 180 | turn hmc L 180\r\n");
    MotorSerial_Printf("[TURNDBG] turn kp 1.5 | turn min 70 | turn max 150 | turn step 10 | turn tol 5 | turn period 50\r\n");
}

static void TurnDebug_PrintStatus(void)
{
    const HMC5883L_Info_t *hmc = App_HMC5883L_GetInfo();
    long kp = TurnDebug_FloatToCenti(s_kp);
    long tol = TurnDebug_FloatToDeci(s_tol_deg);
    long head = TurnDebug_FloatToDeci(hmc->heading_deg);
    long delta = TurnDebug_FloatToDeci(App_HMC5883L_GetDeltaDeg());
    long target = TurnDebug_FloatToDeci(s_target_deg);
    long kp_abs = TurnDebug_AbsLong(kp);
    long tol_abs = TurnDebug_AbsLong(tol);
    long head_abs = TurnDebug_AbsLong(head);
    long delta_abs = TurnDebug_AbsLong(delta);
    long target_abs = TurnDebug_AbsLong(target);

    MotorSerial_Printf("[TURNDBG] state=%s target=%s%ld.%01ld awz=%d twz=%d\r\n",
                       TurnDebug_StateText(s_state),
                       TurnDebug_SignText(target),
                       target_abs / 10L,
                       target_abs % 10L,
                       (int)s_actual_wz,
                       (int)s_target_wz);
    MotorSerial_Printf("[TURNDBG] kp=%ld.%02ld min=%d max=%d step=%d tol=%ld.%01ld period=%lums log=%u\r\n",
                       kp_abs / 100L,
                       kp_abs % 100L,
                       (int)s_min_wz,
                       (int)s_max_wz,
                       (int)s_step_wz,
                       tol_abs / 10L,
                       tol_abs % 10L,
                       (unsigned long)s_ctrl_period_ms,
                       (unsigned int)s_log_enable);
    MotorSerial_Printf("[TURNDBG] hmc_ready=%u head=%s%ld.%01ld delta=%s%ld.%01ld raw=%d,%d,%d age=%lums\r\n",
                       (unsigned int)App_HMC5883L_IsReady(),
                       TurnDebug_SignText(head),
                       head_abs / 10L,
                       head_abs % 10L,
                       TurnDebug_SignText(delta),
                       delta_abs / 10L,
                       delta_abs % 10L,
                       (int)hmc->x,
                       (int)hmc->y,
                       (int)hmc->z,
                       (unsigned long)(HAL_GetTick() - hmc->last_update_tick));
}

static void TurnDebug_StartFixed(int8_t dir_sign, int32_t speed, uint32_t duration_ms)
{
    int32_t speed_abs;

    if (MotorSerial_IsInitDone() == 0U)
    {
        MotorSerial_Printf("[TURN-FIXED] wait motor init\r\n");
        return;
    }

    if (duration_ms > TURNDBG_MAX_FIXED_MS)
    {
        duration_ms = TURNDBG_MAX_FIXED_MS;
    }

    if ((int32_t)duration_ms <= 0L)
    {
        duration_ms = 3000UL;
    }

    speed_abs = (speed < 0L) ? -speed : speed;
    speed_abs = (int32_t)TurnDebug_ClampI16(speed_abs, 1, 300);

    App_HMC5883L_ResetDeltaReference();
    s_state = TURNDBG_STATE_FIXED;
    s_fixed_wz = (int16_t)((dir_sign > 0) ? speed_abs : -speed_abs);
    s_fixed_duration_ms = duration_ms;
    s_start_tick = HAL_GetTick();
    s_last_log_tick = s_start_tick;
    s_target_wz = s_fixed_wz;
    s_actual_wz = s_fixed_wz;
    Motion_SetVxWz(0, s_fixed_wz);

    MotorSerial_Printf("[TURN-FIXED] start dir=%c wz=%d duration=%lums\r\n",
                       (dir_sign > 0) ? 'R' : 'L',
                       (int)s_fixed_wz,
                       (unsigned long)s_fixed_duration_ms);
    TurnDebug_LogFixed(s_start_tick, 1U);
}

static void TurnDebug_StartHmc(int8_t dir_sign, float angle_deg)
{
    long target;
    long target_abs;

    if (MotorSerial_IsInitDone() == 0U)
    {
        MotorSerial_Printf("[TURN-HMC] wait motor init\r\n");
        return;
    }

    if (App_HMC5883L_IsReady() == 0U)
    {
        TurnDebug_StopOutput("hmc_not_ready");
        s_state = TURNDBG_STATE_ERROR;
        MotorSerial_Printf("[TURN-HMC] error hmc_not_ready\r\n");
        return;
    }

    if (angle_deg > TURNDBG_MAX_HMC_ANGLE_DEG)
    {
        angle_deg = TURNDBG_MAX_HMC_ANGLE_DEG;
    }

    App_HMC5883L_ResetDeltaReference();
    s_state = TURNDBG_STATE_HMC_TURN;
    s_target_deg = (dir_sign > 0) ? angle_deg : -angle_deg;
    s_target_wz = 0;
    s_actual_wz = 0;
    s_start_tick = HAL_GetTick();
    s_last_ctrl_tick = s_start_tick - s_ctrl_period_ms;
    s_last_log_tick = s_start_tick;

    target = TurnDebug_FloatToDeci(s_target_deg);
    target_abs = TurnDebug_AbsLong(target);
    MotorSerial_Printf("[TURN-HMC] start dir=%c target=%s%ld.%01ld kp=%ld.%02ld min=%d max=%d step=%d tol=%ld.%01ld\r\n",
                       (dir_sign > 0) ? 'R' : 'L',
                       TurnDebug_SignText(target),
                       target_abs / 10L,
                       target_abs % 10L,
                       TurnDebug_AbsLong(TurnDebug_FloatToCenti(s_kp)) / 100L,
                       TurnDebug_AbsLong(TurnDebug_FloatToCenti(s_kp)) % 100L,
                       (int)s_min_wz,
                       (int)s_max_wz,
                       (int)s_step_wz,
                       TurnDebug_AbsLong(TurnDebug_FloatToDeci(s_tol_deg)) / 10L,
                       TurnDebug_AbsLong(TurnDebug_FloatToDeci(s_tol_deg)) % 10L);
}

static void TurnDebug_TaskFixed(uint32_t now)
{
    uint32_t elapsed = now - s_start_tick;
    long delta;
    long delta_abs;

    TurnDebug_LogFixed(now, 0U);

    if (elapsed < s_fixed_duration_ms)
    {
        return;
    }

    TurnDebug_StopOutput("fixed_done");
    delta = TurnDebug_FloatToDeci(App_HMC5883L_GetDeltaDeg());
    delta_abs = TurnDebug_AbsLong(delta);
    s_state = TURNDBG_STATE_DONE;
    s_actual_wz = 0;
    s_target_wz = 0;
    MotorSerial_Printf("[TURN-FIXED] done elapsed=%lums delta=%s%ld.%01ld\r\n",
                       (unsigned long)elapsed,
                       TurnDebug_SignText(delta),
                       delta_abs / 10L,
                       delta_abs % 10L);
}

static void TurnDebug_TaskHmc(uint32_t now)
{
    float delta_deg;
    float error_deg;
    float target_wz_f;
    int16_t target_wz;
    uint32_t elapsed;
    long delta;
    long err;
    long target;
    long delta_abs;
    long err_abs;
    long target_abs;

    if ((uint32_t)(now - s_last_ctrl_tick) < s_ctrl_period_ms)
    {
        TurnDebug_LogHmc(now,
                         App_HMC5883L_GetDeltaDeg(),
                         TurnDebug_WrapTo180(s_target_deg - App_HMC5883L_GetDeltaDeg()),
                         s_target_wz,
                         0U);
        return;
    }
    s_last_ctrl_tick = now;

    elapsed = now - s_start_tick;
    if (elapsed > TURNDBG_HMC_TIMEOUT_MS)
    {
        delta_deg = App_HMC5883L_GetDeltaDeg();
        error_deg = TurnDebug_WrapTo180(s_target_deg - delta_deg);
        TurnDebug_StopOutput("hmc_timeout");
        s_state = TURNDBG_STATE_ERROR;
        s_actual_wz = 0;
        s_target_wz = 0;
        target = TurnDebug_FloatToDeci(s_target_deg);
        delta = TurnDebug_FloatToDeci(delta_deg);
        err = TurnDebug_FloatToDeci(error_deg);
        target_abs = TurnDebug_AbsLong(target);
        delta_abs = TurnDebug_AbsLong(delta);
        err_abs = TurnDebug_AbsLong(err);
        MotorSerial_Printf("[TURN-HMC] timeout target=%s%ld.%01ld delta=%s%ld.%01ld err=%s%ld.%01ld elapsed=%lums\r\n",
                           TurnDebug_SignText(target),
                           target_abs / 10L,
                           target_abs % 10L,
                           TurnDebug_SignText(delta),
                           delta_abs / 10L,
                           delta_abs % 10L,
                           TurnDebug_SignText(err),
                           err_abs / 10L,
                           err_abs % 10L,
                           (unsigned long)elapsed);
        return;
    }

    if (App_HMC5883L_IsReady() == 0U)
    {
        TurnDebug_StopOutput("hmc_lost");
        s_state = TURNDBG_STATE_ERROR;
        s_actual_wz = 0;
        s_target_wz = 0;
        MotorSerial_Printf("[TURN-HMC] error hmc_lost elapsed=%lums\r\n", (unsigned long)elapsed);
        return;
    }

    delta_deg = App_HMC5883L_GetDeltaDeg();
    error_deg = TurnDebug_WrapTo180(s_target_deg - delta_deg);

    if (TurnDebug_AbsFloat(error_deg) <= s_tol_deg)
    {
        TurnDebug_StopOutput("hmc_done");
        s_state = TURNDBG_STATE_DONE;
        s_actual_wz = 0;
        s_target_wz = 0;
        TurnDebug_LogHmc(now, delta_deg, error_deg, 0, 1U);
        target = TurnDebug_FloatToDeci(s_target_deg);
        delta = TurnDebug_FloatToDeci(delta_deg);
        err = TurnDebug_FloatToDeci(error_deg);
        target_abs = TurnDebug_AbsLong(target);
        delta_abs = TurnDebug_AbsLong(delta);
        err_abs = TurnDebug_AbsLong(err);
        MotorSerial_Printf("[TURN-HMC] done target=%s%ld.%01ld delta=%s%ld.%01ld err=%s%ld.%01ld elapsed=%lums\r\n",
                           TurnDebug_SignText(target),
                           target_abs / 10L,
                           target_abs % 10L,
                           TurnDebug_SignText(delta),
                           delta_abs / 10L,
                           delta_abs % 10L,
                           TurnDebug_SignText(err),
                           err_abs / 10L,
                           err_abs % 10L,
                           (unsigned long)elapsed);
        return;
    }

    target_wz_f = s_kp * error_deg;
    target_wz = TurnDebug_RoundToI16(target_wz_f);
    target_wz = TurnDebug_ClampI16(target_wz, (int16_t)-s_max_wz, s_max_wz);

    if (TurnDebug_AbsI16(target_wz) < s_min_wz)
    {
        target_wz = (error_deg >= 0.0f) ? s_min_wz : (int16_t)-s_min_wz;
    }

    s_target_wz = target_wz;
    s_actual_wz = TurnDebug_RampToInt16(s_actual_wz, s_target_wz, s_step_wz);
    Motion_SetVxWz(0, s_actual_wz);
    TurnDebug_LogHmc(now, delta_deg, error_deg, s_target_wz, 0U);
}

static void TurnDebug_StopOutput(const char *reason)
{
    Motion_StopOutput();
    MotorSerial_Printf("[TURNDBG] stop output pwm=0,0,0,0 reason=%s\r\n", reason);
}

static void TurnDebug_LogFixed(uint32_t now, uint8_t force)
{
    uint32_t elapsed;
    long head;
    long delta;
    long head_abs;
    long delta_abs;

    if ((s_log_enable == 0U) && (force == 0U))
    {
        return;
    }

    if ((force == 0U) && ((uint32_t)(now - s_last_log_tick) < TURNDBG_FIXED_LOG_PERIOD_MS))
    {
        return;
    }
    s_last_log_tick = now;

    elapsed = now - s_start_tick;
    head = TurnDebug_FloatToDeci(App_HMC5883L_GetHeadingDeg());
    delta = TurnDebug_FloatToDeci(App_HMC5883L_GetDeltaDeg());
    head_abs = TurnDebug_AbsLong(head);
    delta_abs = TurnDebug_AbsLong(delta);

    MotorSerial_Printf("[TURN-FIXED] t=%lums head=%s%ld.%01ld delta=%s%ld.%01ld wz=%d\r\n",
                       (unsigned long)elapsed,
                       TurnDebug_SignText(head),
                       head_abs / 10L,
                       head_abs % 10L,
                       TurnDebug_SignText(delta),
                       delta_abs / 10L,
                       delta_abs % 10L,
                       (int)s_fixed_wz);
}

static void TurnDebug_LogHmc(uint32_t now,
                             float delta_deg,
                             float error_deg,
                             int16_t target_wz,
                             uint8_t force)
{
    uint32_t elapsed;
    long target;
    long delta;
    long error;
    long head;
    long target_abs;
    long delta_abs;
    long error_abs;
    long head_abs;

    if ((s_log_enable == 0U) && (force == 0U))
    {
        return;
    }

    if ((force == 0U) && ((uint32_t)(now - s_last_log_tick) < TURNDBG_HMC_LOG_PERIOD_MS))
    {
        return;
    }
    s_last_log_tick = now;

    elapsed = now - s_start_tick;
    target = TurnDebug_FloatToDeci(s_target_deg);
    delta = TurnDebug_FloatToDeci(delta_deg);
    error = TurnDebug_FloatToDeci(error_deg);
    head = TurnDebug_FloatToDeci(App_HMC5883L_GetHeadingDeg());
    target_abs = TurnDebug_AbsLong(target);
    delta_abs = TurnDebug_AbsLong(delta);
    error_abs = TurnDebug_AbsLong(error);
    head_abs = TurnDebug_AbsLong(head);

    MotorSerial_Printf("[TURN-HMC] t=%lums target=%s%ld.%01ld delta=%s%ld.%01ld err=%s%ld.%01ld twz=%d awz=%d head=%s%ld.%01ld\r\n",
                       (unsigned long)elapsed,
                       TurnDebug_SignText(target),
                       target_abs / 10L,
                       target_abs % 10L,
                       TurnDebug_SignText(delta),
                       delta_abs / 10L,
                       delta_abs % 10L,
                       TurnDebug_SignText(error),
                       error_abs / 10L,
                       error_abs % 10L,
                       (int)target_wz,
                       (int)s_actual_wz,
                       TurnDebug_SignText(head),
                       head_abs / 10L,
                       head_abs % 10L);
}

static float TurnDebug_WrapTo180(float deg)
{
    while (deg > 180.0f)
    {
        deg -= 360.0f;
    }

    while (deg < -180.0f)
    {
        deg += 360.0f;
    }

    return deg;
}

static float TurnDebug_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t TurnDebug_AbsI16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static int16_t TurnDebug_ClampI16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < (int32_t)min_value)
    {
        return min_value;
    }

    if (value > (int32_t)max_value)
    {
        return max_value;
    }

    return (int16_t)value;
}

static int16_t TurnDebug_RampToInt16(int16_t current, int16_t target, int16_t step)
{
    int32_t diff = (int32_t)target - (int32_t)current;

    if (step < 1)
    {
        step = 1;
    }

    if (diff > (int32_t)step)
    {
        return (int16_t)(current + step);
    }

    if (diff < -(int32_t)step)
    {
        return (int16_t)(current - step);
    }

    return target;
}

static int16_t TurnDebug_RoundToI16(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

static long TurnDebug_FloatToDeci(float value)
{
    if (value >= 0.0f)
    {
        return (long)((value * 10.0f) + 0.5f);
    }

    return (long)((value * 10.0f) - 0.5f);
}

static long TurnDebug_FloatToCenti(float value)
{
    if (value >= 0.0f)
    {
        return (long)((value * 100.0f) + 0.5f);
    }

    return (long)((value * 100.0f) - 0.5f);
}

static long TurnDebug_AbsLong(long value)
{
    return (value < 0L) ? -value : value;
}

static const char *TurnDebug_SignText(long value)
{
    return (value < 0L) ? "-" : "";
}

static uint8_t TurnDebug_StartsWith(const char *text, const char *prefix)
{
    while (*prefix != '\0')
    {
        if (*text != *prefix)
        {
            return 0U;
        }
        text++;
        prefix++;
    }

    return 1U;
}

static const char *TurnDebug_SkipSpaces(const char *text)
{
    while (*text == ' ')
    {
        text++;
    }

    return text;
}

static const char *TurnDebug_SkipWord(const char *text)
{
    while ((*text != '\0') && (*text != ' '))
    {
        text++;
    }

    return text;
}

static int8_t TurnDebug_ParseDirection(const char **text)
{
    const char *p = TurnDebug_SkipSpaces(*text);
    int8_t dir = 0;

    if (*p == 'r')
    {
        dir = 1;
    }
    else if (*p == 'l')
    {
        dir = -1;
    }

    if (dir != 0)
    {
        p++;
        *text = p;
    }

    return dir;
}

static int32_t TurnDebug_ParseInt(const char *text, int32_t default_value, uint8_t *ok)
{
    char *end;
    long value;

    text = TurnDebug_SkipSpaces(text);
    value = strtol(text, &end, 10);
    if (end == text)
    {
        if (ok != NULL)
        {
            *ok = 0U;
        }
        return default_value;
    }

    if (ok != NULL)
    {
        *ok = 1U;
    }
    return (int32_t)value;
}

static int32_t TurnDebug_ParseFixed100(const char *text, int32_t default_value, uint8_t *ok)
{
    int32_t sign = 1L;
    int32_t whole = 0L;
    int32_t frac = 0L;
    uint8_t frac_digits = 0U;
    uint8_t has_digit = 0U;

    text = TurnDebug_SkipSpaces(text);

    if (*text == '-')
    {
        sign = -1L;
        text++;
    }
    else if (*text == '+')
    {
        text++;
    }

    while ((*text >= '0') && (*text <= '9'))
    {
        whole = (whole * 10L) + (int32_t)(*text - '0');
        has_digit = 1U;
        text++;
    }

    if (*text == '.')
    {
        text++;
        while ((*text >= '0') && (*text <= '9'))
        {
            if (frac_digits < 2U)
            {
                frac = (frac * 10L) + (int32_t)(*text - '0');
                frac_digits++;
            }
            has_digit = 1U;
            text++;
        }
    }

    if (has_digit == 0U)
    {
        if (ok != NULL)
        {
            *ok = 0U;
        }
        return default_value;
    }

    while (frac_digits < 2U)
    {
        frac *= 10L;
        frac_digits++;
    }

    if (ok != NULL)
    {
        *ok = 1U;
    }
    return sign * ((whole * 100L) + frac);
}

static const char *TurnDebug_StateText(TurnDebugState_t state)
{
    switch (state)
    {
    case TURNDBG_STATE_FIXED:
        return "FIXED";
    case TURNDBG_STATE_HMC_TURN:
        return "HMC_TURN";
    case TURNDBG_STATE_DONE:
        return "DONE";
    case TURNDBG_STATE_ERROR:
        return "ERROR";
    case TURNDBG_STATE_IDLE:
    default:
        return "IDLE";
    }
}
