#include "app_coverage.h"

#include "app_config.h"
#if APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP
#include "app_hmc5883l.h"
#endif
#include "app_motion.h"
#include "app_motion_primitive.h"
#include "app_motor_serial.h"
#if APP_ENABLE_OLED
#include "app_oled_ui.h"
#endif

#define COVERAGE_AREA_X_MM          1000.0f
#define COVERAGE_AREA_Y_MM          1000.0f
#define COVERAGE_LINE_SPACING_MM    100.0f

#define COVERAGE_FORWARD_SPEED      120
#define COVERAGE_TURN_SPEED         60
#define COVERAGE_SHIFT_SPEED        100

static CoverageState_t s_state;
static uint8_t s_running;
static uint8_t s_pending_start;
static uint8_t s_row;
static uint8_t s_line_count;
static int8_t s_turn_sign;

static void Coverage_Begin(void);
static void Coverage_Enter(CoverageState_t state);
static void Coverage_UpdateOLED(void);
static void Coverage_StartRunLine(void);
static void Coverage_StartTurnOut(void);
static void Coverage_StartShiftLine(void);
static void Coverage_StartTurnBack(void);
static uint8_t Coverage_IsStartReady(void);
static const char *Coverage_StateText(CoverageState_t state);

void Coverage_Init(void)
{
    s_state = COVERAGE_IDLE;
    s_running = 0U;
    s_pending_start = 0U;
    s_row = 0U;
    s_line_count = (uint8_t)((uint16_t)(COVERAGE_AREA_Y_MM / COVERAGE_LINE_SPACING_MM) + 1U);
    s_turn_sign = 1;
    Coverage_UpdateOLED();
}

void Coverage_Start(void)
{
    if ((s_running != 0U) || (s_pending_start != 0U))
    {
        return;
    }

    if (Coverage_IsStartReady() == 0U)
    {
        s_pending_start = 1U;
        MotorSerial_Printf("[COV] start pending, wait motor/HMC\r\n");
        Coverage_UpdateOLED();
        return;
    }

    Coverage_Begin();
}

void Coverage_Stop(void)
{
    s_pending_start = 0U;

    if (s_state == COVERAGE_IDLE)
    {
        Motion_Stop();
        Coverage_UpdateOLED();
        return;
    }

    MotionPrimitive_Abort();
    Motion_Stop();
    s_running = 0U;
    Coverage_Enter(COVERAGE_ABORT);
}

void Coverage_Task(void)
{
    if ((s_pending_start != 0U) && (Coverage_IsStartReady() != 0U))
    {
        Coverage_Begin();
    }

    if (s_running == 0U)
    {
        return;
    }

    if (MotionPrimitive_IsError() != 0U)
    {
        s_running = 0U;
        Motion_Stop();
        Coverage_Enter(COVERAGE_ERROR);
        return;
    }

    if (MotionPrimitive_IsBusy() != 0U)
    {
        return;
    }

    if (MotionPrimitive_IsDone() == 0U)
    {
        return;
    }

    MotionPrimitive_ClearDone();

    switch (s_state)
    {
    case COVERAGE_RUN_LINE:
        if ((uint8_t)(s_row + 1U) >= s_line_count)
        {
            s_running = 0U;
            Motion_Stop();
            Coverage_Enter(COVERAGE_DONE);
        }
        else
        {
            Coverage_StartTurnOut();
        }
        break;

    case COVERAGE_TURN_OUT:
        Coverage_StartShiftLine();
        break;

    case COVERAGE_SHIFT_LINE:
        Coverage_StartTurnBack();
        break;

    case COVERAGE_TURN_BACK:
        s_row++;
        Coverage_StartRunLine();
        break;

    case COVERAGE_IDLE:
    case COVERAGE_NEXT_LINE:
    case COVERAGE_DONE:
    case COVERAGE_ABORT:
    case COVERAGE_ERROR:
    default:
        break;
    }
}

uint8_t Coverage_IsRunning(void)
{
    return ((s_running != 0U) || (s_pending_start != 0U)) ? 1U : 0U;
}

CoverageState_t Coverage_GetState(void)
{
    return s_state;
}

static void Coverage_Begin(void)
{
    s_pending_start = 0U;
    s_running = 1U;
    s_row = 0U;
    s_line_count = (uint8_t)((uint16_t)(COVERAGE_AREA_Y_MM / COVERAGE_LINE_SPACING_MM) + 1U);
    MotorSerial_Printf("[COV] start area=1000x1000 spacing=100 lines=%u\r\n",
                       (unsigned int)s_line_count);
    Coverage_StartRunLine();
}

static void Coverage_Enter(CoverageState_t state)
{
    s_state = state;
    MotorSerial_Printf("[COV] row=%u state=%s\r\n",
                       (unsigned int)s_row,
                       Coverage_StateText(state));
    Coverage_UpdateOLED();
}

static void Coverage_UpdateOLED(void)
{
#if APP_ENABLE_OLED
    App_OLED_SetCoverageDebug(s_running,
                              s_pending_start,
                              s_row,
                              s_line_count,
                              (uint8_t)s_state);
#endif
}

static void Coverage_StartRunLine(void)
{
    Coverage_Enter(COVERAGE_RUN_LINE);
    MotionPrimitive_MoveDistance_Start(COVERAGE_AREA_X_MM, COVERAGE_FORWARD_SPEED);
}

static void Coverage_StartTurnOut(void)
{
    s_turn_sign = ((s_row % 2U) == 0U) ? 1 : -1;
    Coverage_Enter(COVERAGE_TURN_OUT);
    MotorSerial_Printf("[COV] row=%u turn=%s\r\n",
                       (unsigned int)s_row,
                       (s_turn_sign > 0) ? "right" : "left");
    MotionPrimitive_TurnAngle_Start((float)(90 * s_turn_sign), COVERAGE_TURN_SPEED);
}

static void Coverage_StartShiftLine(void)
{
    Coverage_Enter(COVERAGE_SHIFT_LINE);
    MotionPrimitive_MoveDistance_Start(COVERAGE_LINE_SPACING_MM, COVERAGE_SHIFT_SPEED);
}

static void Coverage_StartTurnBack(void)
{
    Coverage_Enter(COVERAGE_TURN_BACK);
    MotionPrimitive_TurnAngle_Start((float)(90 * s_turn_sign), COVERAGE_TURN_SPEED);
}

static uint8_t Coverage_IsStartReady(void)
{
    if (MotorSerial_IsInitDone() == 0U)
    {
        return 0U;
    }

#if APP_ENABLE_HMC5883L_TURN_CLOSED_LOOP
    if (App_HMC5883L_IsReady() == 0U)
    {
        return 0U;
    }
#endif

    return 1U;
}

static const char *Coverage_StateText(CoverageState_t state)
{
    switch (state)
    {
    case COVERAGE_IDLE:
        return "IDLE";

    case COVERAGE_RUN_LINE:
        return "RUN_LINE";

    case COVERAGE_TURN_OUT:
        return "TURN_OUT";

    case COVERAGE_SHIFT_LINE:
        return "SHIFT_LINE";

    case COVERAGE_TURN_BACK:
        return "TURN_BACK";

    case COVERAGE_NEXT_LINE:
        return "NEXT_LINE";

    case COVERAGE_DONE:
        return "DONE";

    case COVERAGE_ABORT:
        return "ABORT";

    case COVERAGE_ERROR:
    default:
        return "ERROR";
    }
}
