#include "app_motion_primitive.h"

#include "app_motion.h"
#include "app_motor_serial.h"
#include "app_odometry.h"
#include "stm32f1xx_hal.h"

#define PRIM_DISTANCE_TOLERANCE_MM  20.0f
#define PRIM_ANGLE_TOLERANCE_DEG    5.0f
#define PRIM_TIMEOUT_MS             20000U

static MotionPrimitiveState_t s_state;
static float s_target;
static int16_t s_speed;
static uint32_t s_start_tick;

static float Prim_AbsFloat(float value);

void MotionPrimitive_Init(void)
{
    s_state = PRIM_IDLE;
    s_target = 0.0f;
    s_speed = 0;
    s_start_tick = 0UL;
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
    s_target = distance_mm;
    s_speed = (distance_mm >= 0.0f) ? speed : (int16_t)-speed;
    s_state = PRIM_MOVE_DISTANCE;
    s_start_tick = HAL_GetTick();
    Motion_SetVxWz(s_speed, 0);
    MotorSerial_Printf("[PRIM] move start target=%ldmm speed=%d\r\n",
                       (long)distance_mm,
                       (int)s_speed);
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
    s_target = angle_deg;
    s_speed = (angle_deg >= 0.0f) ? turn_speed : (int16_t)-turn_speed;
    s_state = PRIM_TURN_ANGLE;
    s_start_tick = HAL_GetTick();
    Motion_SetVxWz(0, s_speed);
    MotorSerial_Printf("[PRIM] turn start target=%lddeg speed=%d\r\n",
                       (long)angle_deg,
                       (int)s_speed);
}

void MotionPrimitive_Task(void)
{
    float progress;
    float target_abs;

    if ((s_state != PRIM_MOVE_DISTANCE) && (s_state != PRIM_TURN_ANGLE))
    {
        return;
    }

    if ((uint32_t)(HAL_GetTick() - s_start_tick) >= PRIM_TIMEOUT_MS)
    {
        Motion_Stop();
        s_state = PRIM_ERROR;
        MotorSerial_Printf("[PRIM] error timeout\r\n");
        return;
    }

    if (s_state == PRIM_MOVE_DISTANCE)
    {
        progress = Odom_GetForwardAccumMm();
        target_abs = Prim_AbsFloat(s_target);
        if (Prim_AbsFloat(progress) >= (target_abs - PRIM_DISTANCE_TOLERANCE_MM))
        {
            Motion_Stop();
            s_state = PRIM_DONE;
            MotorSerial_Printf("[PRIM] move done progress=%ldmm\r\n", (long)progress);
        }
    }
    else
    {
        progress = Odom_GetTurnAccumDeg();
        target_abs = Prim_AbsFloat(s_target);
        if (Prim_AbsFloat(progress) >= (target_abs - PRIM_ANGLE_TOLERANCE_DEG))
        {
            Motion_Stop();
            s_state = PRIM_DONE;
            MotorSerial_Printf("[PRIM] turn done progress=%lddeg\r\n", (long)progress);
        }
    }
}

uint8_t MotionPrimitive_IsBusy(void)
{
    return ((s_state == PRIM_MOVE_DISTANCE) || (s_state == PRIM_TURN_ANGLE)) ? 1U : 0U;
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
    }
}

void MotionPrimitive_Abort(void)
{
    Motion_Stop();
    s_state = PRIM_IDLE;
}

MotionPrimitiveState_t MotionPrimitive_GetState(void)
{
    return s_state;
}

static float Prim_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}
