#include "app_turn_calib.h"

#include "app_config.h"
#include "app_motion.h"
#include "app_motion_primitive.h"
#include "app_motor_serial.h"
#include "app_odometry.h"
#if APP_ENABLE_OLED
#include "app_oled_ui.h"
#endif

#define TURN_CALIB_BASE_DEG       90
#define TURN_CALIB_COMP_LIMIT_DEG 30
#define TURN_CALIB_DEFAULT_SPEED_INDEX 1U

static const int16_t s_speed_table[] = {90, 100, 110, 120};

static TurnCalibState_t s_state;
static TurnCalibAction_t s_action;
static int8_t s_comp_deg;
static uint8_t s_speed_index;
static int8_t s_dir_sign;
static int16_t s_target_deg;
static int16_t s_progress_deg;

static void TurnCalib_Start(void);
static void TurnCalib_UpdateOLED(void);
static const char *TurnCalib_ActionText(TurnCalibAction_t action);
static int16_t TurnCalib_GetSpeed(void);
static int16_t TurnCalib_MakeTargetDeg(void);
static int16_t TurnCalib_RoundFloat(float value);

void App_TurnCalib_Init(void)
{
    s_state = TURN_CALIB_STATE_IDLE;
    s_action = TURN_CALIB_ACTION_START;
    s_comp_deg = 0;
    s_speed_index = TURN_CALIB_DEFAULT_SPEED_INDEX;
    s_dir_sign = 1;
    s_target_deg = TURN_CALIB_BASE_DEG;
    s_progress_deg = 0;
    TurnCalib_UpdateOLED();
}

void App_TurnCalib_Task(void)
{
    if ((s_state == TURN_CALIB_STATE_PENDING) && (MotorSerial_IsInitDone() != 0U))
    {
        TurnCalib_Start();
    }

    if (s_state != TURN_CALIB_STATE_RUNNING)
    {
        TurnCalib_UpdateOLED();
        return;
    }

    s_progress_deg = TurnCalib_RoundFloat(Odom_GetTurnAccumDeg());

    if (MotionPrimitive_IsError() != 0U)
    {
        Motion_Stop();
        s_state = TURN_CALIB_STATE_ERROR;
        MotorSerial_Printf("[TCAL] error target=%d progress=%d\r\n",
                           (int)s_target_deg,
                           (int)s_progress_deg);
        TurnCalib_UpdateOLED();
        return;
    }

    if (MotionPrimitive_IsDone() != 0U)
    {
        MotionPrimitive_ClearDone();
        Motion_Stop();
        s_state = TURN_CALIB_STATE_DONE;
        s_progress_deg = TurnCalib_RoundFloat(Odom_GetTurnAccumDeg());
        MotorSerial_Printf("[TCAL] done target=%d progress=%d comp=%d speed=%d dir=%c\r\n",
                           (int)s_target_deg,
                           (int)s_progress_deg,
                           (int)s_comp_deg,
                           (int)TurnCalib_GetSpeed(),
                           (s_dir_sign > 0) ? 'R' : 'L');
        TurnCalib_UpdateOLED();
        return;
    }

    TurnCalib_UpdateOLED();
}

void App_TurnCalib_Select(void)
{
    s_action = (TurnCalibAction_t)(((uint8_t)s_action + 1U) % (uint8_t)TURN_CALIB_ACTION_COUNT);
    MotorSerial_Printf("[TCAL] select %s\r\n", TurnCalib_ActionText(s_action));
    TurnCalib_UpdateOLED();
}

void App_TurnCalib_Control(void)
{
    if ((s_state == TURN_CALIB_STATE_RUNNING) || (s_state == TURN_CALIB_STATE_PENDING))
    {
        if (s_action == TURN_CALIB_ACTION_START)
        {
            App_TurnCalib_Stop();
        }
        else
        {
            MotorSerial_Printf("[TCAL] busy, select START then K2 to stop\r\n");
        }
        return;
    }

    switch (s_action)
    {
    case TURN_CALIB_ACTION_START:
        if (MotorSerial_IsInitDone() == 0U)
        {
            s_state = TURN_CALIB_STATE_PENDING;
            MotorSerial_Printf("[TCAL] start pending, wait motor init\r\n");
            break;
        }
        TurnCalib_Start();
        break;

    case TURN_CALIB_ACTION_COMP_UP:
        if (s_comp_deg < TURN_CALIB_COMP_LIMIT_DEG)
        {
            s_comp_deg++;
        }
        MotorSerial_Printf("[TCAL] comp=%d\r\n", (int)s_comp_deg);
        break;

    case TURN_CALIB_ACTION_COMP_DOWN:
        if (s_comp_deg > -TURN_CALIB_COMP_LIMIT_DEG)
        {
            s_comp_deg--;
        }
        MotorSerial_Printf("[TCAL] comp=%d\r\n", (int)s_comp_deg);
        break;

    case TURN_CALIB_ACTION_SPEED:
        s_speed_index = (uint8_t)((s_speed_index + 1U) %
                                  (sizeof(s_speed_table) / sizeof(s_speed_table[0])));
        MotorSerial_Printf("[TCAL] speed=%d\r\n", (int)TurnCalib_GetSpeed());
        break;

    case TURN_CALIB_ACTION_DIR:
        s_dir_sign = (s_dir_sign > 0) ? -1 : 1;
        MotorSerial_Printf("[TCAL] dir=%c\r\n", (s_dir_sign > 0) ? 'R' : 'L');
        break;

    case TURN_CALIB_ACTION_RESET:
    default:
        s_comp_deg = 0;
        s_speed_index = TURN_CALIB_DEFAULT_SPEED_INDEX;
        s_dir_sign = 1;
        s_state = TURN_CALIB_STATE_IDLE;
        s_progress_deg = 0;
        MotorSerial_Printf("[TCAL] reset\r\n");
        break;
    }

    TurnCalib_UpdateOLED();
}

void App_TurnCalib_Stop(void)
{
    MotionPrimitive_Abort();
    Motion_Stop();
    s_state = TURN_CALIB_STATE_STOP;
    MotorSerial_Printf("[TCAL] stop\r\n");
    TurnCalib_UpdateOLED();
}

uint8_t App_TurnCalib_IsRunning(void)
{
    return ((s_state == TURN_CALIB_STATE_RUNNING) ||
            (s_state == TURN_CALIB_STATE_PENDING)) ? 1U : 0U;
}

static void TurnCalib_Start(void)
{
    s_target_deg = TurnCalib_MakeTargetDeg();
    s_progress_deg = 0;
    s_state = TURN_CALIB_STATE_RUNNING;
    MotionPrimitive_ClearDone();
    MotionPrimitive_TurnAngle_Start((float)s_target_deg, TurnCalib_GetSpeed());
    MotorSerial_Printf("[TCAL] start target=%d comp=%d speed=%d dir=%c\r\n",
                       (int)s_target_deg,
                       (int)s_comp_deg,
                       (int)TurnCalib_GetSpeed(),
                       (s_dir_sign > 0) ? 'R' : 'L');
    TurnCalib_UpdateOLED();
}

static void TurnCalib_UpdateOLED(void)
{
#if APP_ENABLE_OLED
    App_OLED_SetTurnCalibDebug((uint8_t)s_state,
                               (uint8_t)s_action,
                               (s_dir_sign > 0) ? 1U : 0U,
                               (uint16_t)TurnCalib_GetSpeed(),
                               s_comp_deg,
                               s_target_deg,
                               s_progress_deg);
#endif
}

static const char *TurnCalib_ActionText(TurnCalibAction_t action)
{
    switch (action)
    {
    case TURN_CALIB_ACTION_START:
        return "START";

    case TURN_CALIB_ACTION_COMP_UP:
        return "C+";

    case TURN_CALIB_ACTION_COMP_DOWN:
        return "C-";

    case TURN_CALIB_ACTION_SPEED:
        return "SPD";

    case TURN_CALIB_ACTION_DIR:
        return "DIR";

    case TURN_CALIB_ACTION_RESET:
    default:
        return "RESET";
    }
}

static int16_t TurnCalib_GetSpeed(void)
{
    if (s_speed_index >= (sizeof(s_speed_table) / sizeof(s_speed_table[0])))
    {
        s_speed_index = TURN_CALIB_DEFAULT_SPEED_INDEX;
    }

    return s_speed_table[s_speed_index];
}

static int16_t TurnCalib_MakeTargetDeg(void)
{
    int16_t target = (int16_t)(TURN_CALIB_BASE_DEG + s_comp_deg);

    if (target < 1)
    {
        target = 1;
    }

    return (s_dir_sign > 0) ? target : (int16_t)-target;
}

static int16_t TurnCalib_RoundFloat(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}
