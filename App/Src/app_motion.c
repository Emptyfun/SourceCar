#include "app_motion.h"

#include "app_config.h"
#include "app_motor_serial.h"
#include "stm32f1xx_hal.h"

#define MOTION_SPEED_LIMIT 300
#define MOTOR_CMD_PERIOD_MS 50U
#define MOTOR_FEEDBACK_TIMEOUT_MS 1000U

static int16_t s_target_m[4];
static uint8_t s_active;
static uint8_t s_dirty;
static uint32_t s_last_cmd_tick;
static uint32_t s_move_start_tick;

static int16_t Motion_ClampSpeed(int32_t value);

void Motion_Init(void)
{
    s_target_m[0] = 0;
    s_target_m[1] = 0;
    s_target_m[2] = 0;
    s_target_m[3] = 0;
    s_active = 0U;
    s_dirty = 0U;
    s_last_cmd_tick = 0UL;
    s_move_start_tick = 0UL;
#if !APP_DEBUG_TURN_OPTIMIZE_ONLY
    Motion_Stop();
#endif
}

void Motion_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (s_active != 0U)
    {
        if (((uint32_t)(now - s_move_start_tick) > MOTOR_FEEDBACK_TIMEOUT_MS) &&
            (MotorSerial_HasRecentFeedback(MOTOR_FEEDBACK_TIMEOUT_MS) == 0U))
        {
#if APP_DEBUG_TURN_OPTIMIZE_ONLY
            MotorSerial_StopOutput();
#else
            MotorSerial_Stop();
#endif
            s_active = 0U;
            s_dirty = 0U;
            MotorSerial_Printf("[MOTION] stop feedback timeout\r\n");
            return;
        }
    }

    if ((s_dirty == 0U) && (s_active == 0U))
    {
        return;
    }

    if ((s_dirty == 0U) && ((uint32_t)(now - s_last_cmd_tick) < MOTOR_CMD_PERIOD_MS))
    {
        return;
    }

    MotorSerial_SendSpeed(s_target_m[0], s_target_m[1], s_target_m[2], s_target_m[3]);
    s_last_cmd_tick = now;
    s_dirty = 0U;
}

void Motion_Stop(void)
{
    s_target_m[0] = 0;
    s_target_m[1] = 0;
    s_target_m[2] = 0;
    s_target_m[3] = 0;
    s_active = 0U;
    s_dirty = 0U;
    MotorSerial_Stop();
}

void Motion_StopOutput(void)
{
    s_target_m[0] = 0;
    s_target_m[1] = 0;
    s_target_m[2] = 0;
    s_target_m[3] = 0;
    s_active = 0U;
    s_dirty = 0U;
    MotorSerial_StopOutput();
}

void Motion_SetWheelSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    s_target_m[0] = Motion_ClampSpeed(m1);
    s_target_m[1] = Motion_ClampSpeed(m2);
    s_target_m[2] = Motion_ClampSpeed(m3);
    s_target_m[3] = Motion_ClampSpeed(m4);
    s_active = ((s_target_m[0] != 0) ||
                (s_target_m[1] != 0) ||
                (s_target_m[2] != 0) ||
                (s_target_m[3] != 0)) ? 1U : 0U;
    s_dirty = 1U;
    s_move_start_tick = HAL_GetTick();
}

void Motion_SetVxWz(int16_t vx, int16_t wz)
{
    int32_t m1 = (int32_t)vx + (int32_t)wz;
    int32_t m2 = -(int32_t)vx + (int32_t)wz;
    int32_t m3 = (int32_t)vx - (int32_t)wz;
    int32_t m4 = -(int32_t)vx - (int32_t)wz;

    Motion_SetWheelSpeed(Motion_ClampSpeed(m1),
                         Motion_ClampSpeed(m2),
                         Motion_ClampSpeed(m3),
                         Motion_ClampSpeed(m4));
}

uint8_t Motion_IsMoving(void)
{
    return s_active;
}

static int16_t Motion_ClampSpeed(int32_t value)
{
    if (value > MOTION_SPEED_LIMIT)
    {
        return MOTION_SPEED_LIMIT;
    }

    if (value < -MOTION_SPEED_LIMIT)
    {
        return -MOTION_SPEED_LIMIT;
    }

    return (int16_t)value;
}
