#include "app_motion_primitive.h"

#include "app_config.h"
#if APP_ENABLE_HMC5883L
#include "app_hmc5883l.h"
#endif
#include "app_motion.h"
#include "app_motor_serial.h"
#include "app_odometry.h"
#include "stm32f1xx_hal.h"

#define PRIM_DISTANCE_TOLERANCE_MM  20.0f
#define PRIM_ANGLE_TOLERANCE_DEG    2.0f
#define PRIM_TURN_FINE_ENTRY_DEG    35.0f
#define PRIM_TURN_SUPER_FINE_ENTRY_DEG 8.0f
#define PRIM_TURN_FINE_SPEED        30
#define PRIM_TURN_SUPER_FINE_SPEED  16
#define PRIM_TURN_SETTLE_MS         500U
#define PRIM_CONTROL_PERIOD_MS      50U
#define PRIM_TURN_DEBUG_PERIOD_MS   200U
#define PRIM_STRAIGHT_HEADING_KP    3.0f
#define PRIM_STRAIGHT_WZ_LIMIT      60
#define PRIM_TIMEOUT_MS             20000U

#define PRIM_HEADING_HOLD_ENABLE    1
#define PRIM_HEADING_CTRL_PERIOD_MS 50U
#define PRIM_HEADING_LOG_PERIOD_MS  200U
#define PRIM_HEADING_KP             8.0f
#define PRIM_HEADING_DEADBAND_DEG   2.0f
#define PRIM_HEADING_MAX_WZ         35
#define PRIM_HEADING_RAMP_STEP      5
#define PRIM_HEADING_WZ_SIGN        1

typedef enum
{
    PRIM_TURN_PHASE_IDLE = 0,
    PRIM_TURN_PHASE_COARSE,
    PRIM_TURN_PHASE_FINE,
    PRIM_TURN_PHASE_SETTLE
} PrimTurnPhase_t;

static MotionPrimitiveState_t s_state;
static float s_move_target_mm;
static float s_turn_target_deg;
static int16_t s_speed;
static uint32_t s_start_tick;
static uint32_t s_phase_tick;
static uint32_t s_last_control_tick;
static uint32_t s_last_turn_debug_tick;
static PrimTurnPhase_t s_turn_phase;
static int16_t s_turn_cmd_wz;
static uint8_t s_turn_debug_enabled = 1U;
static float s_turn_progress_deg;
static float s_turn_last_heading_deg;
static float s_heading_target_deg;
static int16_t s_heading_cmd_wz;
static uint32_t s_last_heading_log_tick;

static float Prim_AbsFloat(float value);
static int16_t Prim_AbsI16(int16_t value);
#if APP_ENABLE_HMC5883L_STRAIGHT_HOLD
static int16_t Prim_ClampI16(int32_t value, int16_t limit_abs);
static int16_t Prim_RoundFloatToI16(float value);
#endif
static float Prim_WrapTo180(float deg);
static int16_t Prim_RampToI16(int16_t current, int16_t target, int16_t step);
static int16_t Prim_ClampSignedI16(int16_t value, int16_t limit_abs);
static int16_t Prim_RoundFloatToI16_All(float value);
static void Prim_RunMoveDistance(uint32_t now);
static void Prim_RunMoveDistanceHeading(uint32_t now);
static void Prim_RunTurnAngle(uint32_t now);
static void Prim_UpdateStraightHeadingHold(uint32_t now);
static float Prim_GetTurnProgressDeg(void);
static float Prim_GetTurnRemainingDeg(float progress);
static float Prim_WrapDeltaDeg(float now_deg, float ref_deg);
static void Prim_ResetRelativeTurnTracker(void);
static void Prim_UpdateRelativeTurnProgress(void);
static const char *Prim_TurnPhaseText(PrimTurnPhase_t phase);
static void Prim_DebugTurn(uint32_t now, float progress, float remaining);
static uint8_t Prim_CoarseShouldEnterFine(float remaining);
static int16_t Prim_GetFineTurnSpeedAbs(float remaining);
static int16_t Prim_MakeTurnSpeed(float remaining, int16_t speed_abs);
static void Prim_StartTurnFine(float remaining, float progress);
static void Prim_StartTurnSettle(float progress);
static void Prim_FinishTurn(float progress);
static void Prim_SetTurnWz(int16_t wz);

void MotionPrimitive_Init(void)
{
    s_state = PRIM_IDLE;
    s_move_target_mm = 0.0f;
    s_turn_target_deg = 0.0f;
    s_speed = 0;
    s_start_tick = 0UL;
    s_phase_tick = 0UL;
    s_last_control_tick = 0UL;
    s_last_turn_debug_tick = 0UL;
    s_turn_phase = PRIM_TURN_PHASE_IDLE;
    s_turn_cmd_wz = 0;
    s_turn_progress_deg = 0.0f;
    s_turn_last_heading_deg = 0.0f;
    s_heading_target_deg = 0.0f;
    s_heading_cmd_wz = 0;
    s_last_heading_log_tick = 0UL;
}

void MotionPrimitive_MoveDistance_Start(float distance_mm, int16_t speed)
{
    if (speed < 0)
    {
        speed = (int16_t)-speed;
    }
    if (speed == 0)
    {
        speed = 100;
    }

    Odom_ResetLocalAccum();
#if APP_ENABLE_HMC5883L_STRAIGHT_HOLD
    if (App_HMC5883L_IsReady() == 0U)
    {
        Motion_Stop();
        s_state = PRIM_ERROR;
        MotorSerial_Printf("[PRIM] move error HMC not ready\r\n");
        return;
    }
    App_HMC5883L_ResetDeltaReference();
    s_turn_last_heading_deg = App_HMC5883L_GetHeadingDeg();
    s_turn_progress_deg = 0.0f;
#endif
    s_turn_phase = PRIM_TURN_PHASE_IDLE;
    s_move_target_mm = distance_mm;
    s_speed = (distance_mm >= 0.0f) ? speed : (int16_t)-speed;
    s_state = PRIM_MOVE_DISTANCE;
    s_start_tick = HAL_GetTick();
    s_last_control_tick = s_start_tick;
    Motion_SetVxWz(s_speed, 0);
    MotorSerial_Printf("[PRIM] move start target=%ldmm speed=%d\r\n",
                       (long)distance_mm,
                       (int)s_speed);
}

void MotionPrimitive_MoveDistanceHeading_Start(float distance_mm,
                                               int16_t speed,
                                               float target_heading_deg)
{
    if (speed < 0)
    {
        speed = (int16_t)-speed;
    }
    if (speed == 0)
    {
        speed = 100;
    }

    Odom_ResetLocalAccum();
    s_turn_phase = PRIM_TURN_PHASE_IDLE;
    s_move_target_mm = distance_mm;
    s_speed = (distance_mm >= 0.0f) ? speed : (int16_t)-speed;
    s_state = PRIM_MOVE_DISTANCE_HEADING;
    s_start_tick = HAL_GetTick();
    s_last_control_tick = s_start_tick;
    s_last_heading_log_tick = s_start_tick;
    s_heading_target_deg = target_heading_deg;
    s_heading_cmd_wz = 0;

    Motion_SetVxWz(s_speed, 0);

    MotorSerial_Printf("[PRIM-HH] start target=%ldmm speed=%d heading=%ld\r\n",
                       (long)distance_mm,
                       (int)s_speed,
                       (long)s_heading_target_deg);
}

void MotionPrimitive_TurnAngle_Start(float angle_deg, int16_t turn_speed)
{
    if (turn_speed < 0)
    {
        turn_speed = (int16_t)-turn_speed;
    }
    if (turn_speed == 0)
    {
        turn_speed = 80;
    }

    Odom_ResetLocalAccum();
#if APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP
    if (App_HMC5883L_IsReady() == 0U)
    {
        Motion_Stop();
        s_state = PRIM_ERROR;
        MotorSerial_Printf("[PRIM] turn error HMC not ready\r\n");
        return;
    }
    App_HMC5883L_ResetDeltaReference();
#endif
    Prim_ResetRelativeTurnTracker();
    s_turn_target_deg = angle_deg;
    s_speed = (angle_deg >= 0.0f) ? turn_speed : (int16_t)-turn_speed;
    s_state = PRIM_TURN_ANGLE;
    s_start_tick = HAL_GetTick();
    s_phase_tick = s_start_tick;
    s_last_control_tick = s_start_tick;
    s_last_turn_debug_tick = s_start_tick;
    s_turn_phase = PRIM_TURN_PHASE_COARSE;
    Prim_SetTurnWz(s_speed);
    MotorSerial_Printf("[PRIM] turn relative coarse rel_target=%lddeg speed=%d\r\n",
                       (long)angle_deg,
                       (int)s_speed);
}

void MotionPrimitive_Task(void)
{
    uint32_t now = HAL_GetTick();

    if ((s_state != PRIM_MOVE_DISTANCE) &&
        (s_state != PRIM_MOVE_DISTANCE_HEADING) &&
        (s_state != PRIM_TURN_ANGLE))
    {
        return;
    }

    if ((uint32_t)(now - s_start_tick) >= PRIM_TIMEOUT_MS)
    {
        Motion_Stop();
        s_state = PRIM_ERROR;
        MotorSerial_Printf("[PRIM] error timeout\r\n");
        return;
    }

    if (s_state == PRIM_MOVE_DISTANCE)
    {
        Prim_RunMoveDistance(now);
    }
    else if (s_state == PRIM_MOVE_DISTANCE_HEADING)
    {
        Prim_RunMoveDistanceHeading(now);
    }
    else
    {
        Prim_RunTurnAngle(now);
    }
}

uint8_t MotionPrimitive_IsBusy(void)
{
    return ((s_state == PRIM_MOVE_DISTANCE) ||
            (s_state == PRIM_MOVE_DISTANCE_HEADING) ||
            (s_state == PRIM_TURN_ANGLE)) ? 1U : 0U;
}

uint8_t MotionPrimitive_IsDone(void)
{
    return (s_state == PRIM_DONE) ? 1U : 0U;
}

uint8_t MotionPrimitive_IsError(void)
{
    return (s_state == PRIM_ERROR) ? 1U : 0U;
}

void MotionPrimitive_ClearDone(void)
{
    if ((s_state == PRIM_DONE) || (s_state == PRIM_ERROR))
    {
        s_state = PRIM_IDLE;
        s_turn_phase = PRIM_TURN_PHASE_IDLE;
        s_heading_cmd_wz = 0;
    }
}

void MotionPrimitive_Abort(void)
{
    Motion_Stop();
    s_state = PRIM_IDLE;
    s_turn_phase = PRIM_TURN_PHASE_IDLE;
    s_heading_cmd_wz = 0;
}

MotionPrimitiveState_t MotionPrimitive_GetState(void)
{
    return s_state;
}

void MotionPrimitive_SetTurnDebug(uint8_t enabled)
{
    s_turn_debug_enabled = (enabled != 0U) ? 1U : 0U;
}

static float Prim_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t Prim_AbsI16(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

#if APP_ENABLE_HMC5883L_STRAIGHT_HOLD
static int16_t Prim_ClampI16(int32_t value, int16_t limit_abs)
{
    if (limit_abs < 0)
    {
        limit_abs = (int16_t)-limit_abs;
    }

    if (value > limit_abs)
    {
        return limit_abs;
    }

    if (value < (int32_t)-limit_abs)
    {
        return (int16_t)-limit_abs;
    }

    return (int16_t)value;
}

static int16_t Prim_RoundFloatToI16(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}
#endif

static float Prim_WrapTo180(float deg)
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

static int16_t Prim_RampToI16(int16_t current, int16_t target, int16_t step)
{
    if (step <= 0)
    {
        return target;
    }

    if (current < target)
    {
        current = (int16_t)(current + step);
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current = (int16_t)(current - step);
        if (current < target)
        {
            current = target;
        }
    }

    return current;
}

static int16_t Prim_ClampSignedI16(int16_t value, int16_t limit_abs)
{
    if (limit_abs < 0)
    {
        limit_abs = (int16_t)-limit_abs;
    }

    if (value > limit_abs)
    {
        return limit_abs;
    }

    if (value < (int16_t)-limit_abs)
    {
        return (int16_t)-limit_abs;
    }

    return value;
}

static int16_t Prim_RoundFloatToI16_All(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

static void Prim_RunMoveDistance(uint32_t now)
{
    float progress = Odom_GetForwardAccumMm();
    float target_abs = Prim_AbsFloat(s_move_target_mm);
    float done_threshold = (target_abs > PRIM_DISTANCE_TOLERANCE_MM) ?
                           (target_abs - PRIM_DISTANCE_TOLERANCE_MM) :
                           0.0f;

    if (Prim_AbsFloat(progress) >= done_threshold)
    {
        Motion_Stop();
        s_state = PRIM_DONE;
        MotorSerial_Printf("[PRIM] move done progress=%ldmm\r\n", (long)progress);
        return;
    }

    Prim_UpdateStraightHeadingHold(now);
}

static void Prim_RunMoveDistanceHeading(uint32_t now)
{
    float progress = Odom_GetForwardAccumMm();
    float target_abs = Prim_AbsFloat(s_move_target_mm);
    float done_threshold = (target_abs > PRIM_DISTANCE_TOLERANCE_MM) ?
                           (target_abs - PRIM_DISTANCE_TOLERANCE_MM) :
                           0.0f;

    if (Prim_AbsFloat(progress) >= done_threshold)
    {
        Motion_Stop();
        s_heading_cmd_wz = 0;
        s_state = PRIM_DONE;
        MotorSerial_Printf("[PRIM-HH] done progress=%ldmm\r\n", (long)progress);
        return;
    }

    if ((uint32_t)(now - s_last_control_tick) < PRIM_HEADING_CTRL_PERIOD_MS)
    {
        return;
    }

    s_last_control_tick = now;

#if PRIM_HEADING_HOLD_ENABLE
    if (App_HMC5883L_IsReady() == 0U)
    {
        s_heading_cmd_wz = Prim_RampToI16(s_heading_cmd_wz,
                                          0,
                                          PRIM_HEADING_RAMP_STEP);
        Motion_SetVxWz(s_speed, s_heading_cmd_wz);
        return;
    }

    {
        float heading = App_HMC5883L_GetHeadingDeg();
        float error_deg = Prim_WrapTo180(s_heading_target_deg - heading);
        float abs_err = Prim_AbsFloat(error_deg);
        int16_t target_wz = 0;

        if (abs_err > PRIM_HEADING_DEADBAND_DEG)
        {
            float wz_f = (float)PRIM_HEADING_WZ_SIGN *
                         PRIM_HEADING_KP *
                         error_deg;

            target_wz = Prim_RoundFloatToI16_All(wz_f);
            target_wz = Prim_ClampSignedI16(target_wz, PRIM_HEADING_MAX_WZ);
        }

        s_heading_cmd_wz = Prim_RampToI16(s_heading_cmd_wz,
                                          target_wz,
                                          PRIM_HEADING_RAMP_STEP);
        Motion_SetVxWz(s_speed, s_heading_cmd_wz);

        if ((uint32_t)(now - s_last_heading_log_tick) >= PRIM_HEADING_LOG_PERIOD_MS)
        {
            s_last_heading_log_tick = now;
            MotorSerial_Printf("[PRIM-HH] prog=%ld head=%ld target=%ld err=%ld wz=%d\r\n",
                               (long)progress,
                               (long)heading,
                               (long)s_heading_target_deg,
                               (long)error_deg,
                               (int)s_heading_cmd_wz);
        }
    }
#else
    Motion_SetVxWz(s_speed, 0);
#endif
}

static void Prim_RunTurnAngle(uint32_t now)
{
    float progress = Prim_GetTurnProgressDeg();
    float remaining;

    if (s_state == PRIM_ERROR)
    {
        return;
    }

    remaining = Prim_GetTurnRemainingDeg(progress);
    Prim_DebugTurn(now, progress, remaining);

    switch (s_turn_phase)
    {
    case PRIM_TURN_PHASE_COARSE:
        if (Prim_AbsFloat(remaining) <= PRIM_ANGLE_TOLERANCE_DEG)
        {
            Prim_StartTurnSettle(progress);
        }
        else if (Prim_CoarseShouldEnterFine(remaining) != 0U)
        {
            Prim_StartTurnFine(remaining, progress);
        }
        break;

    case PRIM_TURN_PHASE_FINE:
        if (Prim_AbsFloat(remaining) <= PRIM_ANGLE_TOLERANCE_DEG)
        {
            Prim_StartTurnSettle(progress);
        }
        else if ((uint32_t)(now - s_last_control_tick) >= PRIM_CONTROL_PERIOD_MS)
        {
            s_last_control_tick = now;
            Prim_SetTurnWz(Prim_MakeTurnSpeed(remaining, Prim_GetFineTurnSpeedAbs(remaining)));
        }
        break;

    case PRIM_TURN_PHASE_SETTLE:
        if ((uint32_t)(now - s_phase_tick) < PRIM_TURN_SETTLE_MS)
        {
            break;
        }

        if (Prim_AbsFloat(remaining) <= PRIM_ANGLE_TOLERANCE_DEG)
        {
            Prim_FinishTurn(progress);
        }
        else
        {
            MotorSerial_Printf("[PRIM] turn refine rel_error=%lddeg rel_progress=%lddeg\r\n",
                               (long)remaining,
                               (long)progress);
            Prim_StartTurnFine(remaining, progress);
        }
        break;

    case PRIM_TURN_PHASE_IDLE:
    default:
        if (Prim_AbsFloat(remaining) <= PRIM_ANGLE_TOLERANCE_DEG)
        {
            Prim_StartTurnSettle(progress);
        }
        else
        {
            Prim_StartTurnFine(remaining, progress);
        }
        break;
    }
}

static void Prim_UpdateStraightHeadingHold(uint32_t now)
{
#if APP_ENABLE_HMC5883L_STRAIGHT_HOLD
    float error_deg;
    int16_t correction_wz;

    if (App_HMC5883L_IsReady() == 0U)
    {
        Motion_Stop();
        s_state = PRIM_ERROR;
        MotorSerial_Printf("[PRIM] move error HMC lost\r\n");
        return;
    }

    if ((uint32_t)(now - s_last_control_tick) < PRIM_CONTROL_PERIOD_MS)
    {
        return;
    }
    s_last_control_tick = now;

    error_deg = App_HMC5883L_GetDeltaDeg();
    correction_wz = Prim_ClampI16((int32_t)Prim_RoundFloatToI16(-error_deg * PRIM_STRAIGHT_HEADING_KP),
                                  PRIM_STRAIGHT_WZ_LIMIT);
    Motion_SetVxWz(s_speed, correction_wz);
#else
    (void)now;
#endif
}

static float Prim_GetTurnProgressDeg(void)
{
#if APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP
    if (App_HMC5883L_IsReady() == 0U)
    {
        Motion_Stop();
        s_state = PRIM_ERROR;
        MotorSerial_Printf("[PRIM] turn error HMC lost\r\n");
        return 0.0f;
    }

    Prim_UpdateRelativeTurnProgress();
    return s_turn_progress_deg;
#else
    return Odom_GetTurnAccumDeg();
#endif
}

static float Prim_GetTurnRemainingDeg(float progress)
{
    return s_turn_target_deg - progress;
}

static float Prim_WrapDeltaDeg(float now_deg, float ref_deg)
{
    float d = now_deg - ref_deg;

    while (d > 180.0f)
    {
        d -= 360.0f;
    }

    while (d < -180.0f)
    {
        d += 360.0f;
    }

    return d;
}

static void Prim_ResetRelativeTurnTracker(void)
{
    s_turn_progress_deg = 0.0f;
#if APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP
    s_turn_last_heading_deg = App_HMC5883L_GetHeadingDeg();
#else
    s_turn_last_heading_deg = 0.0f;
#endif
}

static void Prim_UpdateRelativeTurnProgress(void)
{
#if APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP
    float heading = App_HMC5883L_GetHeadingDeg();
    float step = Prim_WrapDeltaDeg(heading, s_turn_last_heading_deg);

    s_turn_progress_deg += step;
    s_turn_last_heading_deg = heading;
#endif
}

static uint8_t Prim_CoarseShouldEnterFine(float remaining)
{
    if (Prim_AbsFloat(remaining) <= PRIM_TURN_FINE_ENTRY_DEG)
    {
        return 1U;
    }

    if ((s_turn_target_deg >= 0.0f) && (remaining < 0.0f))
    {
        return 1U;
    }

    if ((s_turn_target_deg < 0.0f) && (remaining > 0.0f))
    {
        return 1U;
    }

    return 0U;
}

static const char *Prim_TurnPhaseText(PrimTurnPhase_t phase)
{
    switch (phase)
    {
    case PRIM_TURN_PHASE_COARSE:
        return "COARSE";

    case PRIM_TURN_PHASE_FINE:
        return "FINE";

    case PRIM_TURN_PHASE_SETTLE:
        return "SETTLE";

    case PRIM_TURN_PHASE_IDLE:
    default:
        return "IDLE";
    }
}

static void Prim_DebugTurn(uint32_t now, float progress, float remaining)
{
    if (s_turn_debug_enabled == 0U)
    {
        return;
    }

    if ((uint32_t)(now - s_last_turn_debug_tick) < PRIM_TURN_DEBUG_PERIOD_MS)
    {
        return;
    }
    s_last_turn_debug_tick = now;

#if APP_ENABLE_HMC5883L
    {
        const HMC5883L_Info_t *hmc = App_HMC5883L_GetInfo();
        MotorSerial_Printf("[TDBG] phase=%s rel_target=%ld rel_progress=%ld rel_remain=%ld wz=%d head=%ld hdelta=%ld raw=%d,%d,%d\r\n",
                           Prim_TurnPhaseText(s_turn_phase),
                           (long)s_turn_target_deg,
                           (long)progress,
                           (long)remaining,
                           (int)s_turn_cmd_wz,
                           (long)hmc->heading_deg,
                           (long)hmc->delta_deg,
                           (int)hmc->x,
                           (int)hmc->y,
                           (int)hmc->z);
    }
#else
    MotorSerial_Printf("[TDBG] phase=%s rel_target=%ld rel_progress=%ld rel_remain=%ld wz=%d\r\n",
                       Prim_TurnPhaseText(s_turn_phase),
                       (long)s_turn_target_deg,
                       (long)progress,
                       (long)remaining,
                       (int)s_turn_cmd_wz);
#endif
}

static int16_t Prim_GetFineTurnSpeedAbs(float remaining)
{
    int16_t speed_abs = Prim_AbsI16(s_speed);
    int16_t fine_limit = (Prim_AbsFloat(remaining) <= PRIM_TURN_SUPER_FINE_ENTRY_DEG) ?
                         PRIM_TURN_SUPER_FINE_SPEED :
                         PRIM_TURN_FINE_SPEED;

    if (speed_abs > fine_limit)
    {
        return fine_limit;
    }

    return speed_abs;
}

static int16_t Prim_MakeTurnSpeed(float remaining, int16_t speed_abs)
{
    if (speed_abs < 1)
    {
        speed_abs = 1;
    }

    return (remaining >= 0.0f) ? speed_abs : (int16_t)-speed_abs;
}

static void Prim_StartTurnFine(float remaining, float progress)
{
    int16_t fine_speed = Prim_MakeTurnSpeed(remaining, Prim_GetFineTurnSpeedAbs(remaining));

    s_turn_phase = PRIM_TURN_PHASE_FINE;
    s_phase_tick = HAL_GetTick();
    s_last_control_tick = s_phase_tick;
    Prim_SetTurnWz(fine_speed);
    MotorSerial_Printf("[PRIM] turn fine rel_progress=%lddeg rel_remain=%lddeg speed=%d\r\n",
                       (long)progress,
                       (long)remaining,
                       (int)fine_speed);
}

static void Prim_StartTurnSettle(float progress)
{
    Motion_Stop();
    s_turn_cmd_wz = 0;
    s_turn_phase = PRIM_TURN_PHASE_SETTLE;
    s_phase_tick = HAL_GetTick();
    MotorSerial_Printf("[PRIM] turn settle rel_progress=%lddeg\r\n", (long)progress);
}

static void Prim_FinishTurn(float progress)
{
    Motion_Stop();
    s_turn_cmd_wz = 0;
    s_turn_phase = PRIM_TURN_PHASE_IDLE;
    s_state = PRIM_DONE;
    MotorSerial_Printf("[PRIM] turn done rel_progress=%lddeg\r\n", (long)progress);
}

static void Prim_SetTurnWz(int16_t wz)
{
    s_turn_cmd_wz = wz;
    Motion_SetVxWz(0, wz);
}
