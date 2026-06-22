#ifndef APP_TURN_CALIB_H
#define APP_TURN_CALIB_H

#include <stdint.h>

typedef enum
{
    TURN_CALIB_ACTION_START = 0,
    TURN_CALIB_ACTION_COMP_UP,
    TURN_CALIB_ACTION_COMP_DOWN,
    TURN_CALIB_ACTION_SPEED,
    TURN_CALIB_ACTION_DIR,
    TURN_CALIB_ACTION_RESET,
    TURN_CALIB_ACTION_COUNT
} TurnCalibAction_t;

typedef enum
{
    TURN_CALIB_STATE_IDLE = 0,
    TURN_CALIB_STATE_PENDING,
    TURN_CALIB_STATE_RUNNING,
    TURN_CALIB_STATE_DONE,
    TURN_CALIB_STATE_ERROR,
    TURN_CALIB_STATE_STOP
} TurnCalibState_t;

void App_TurnCalib_Init(void);
void App_TurnCalib_Task(void);
void App_TurnCalib_Select(void);
void App_TurnCalib_Control(void);
void App_TurnCalib_Stop(void);
uint8_t App_TurnCalib_IsRunning(void);

#endif /* APP_TURN_CALIB_H */
