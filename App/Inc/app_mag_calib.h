#ifndef APP_MAG_CALIB_H
#define APP_MAG_CALIB_H

#include <stdint.h>

typedef enum
{
    MAG_CALIB_STATE_IDLE = 0,
    MAG_CALIB_STATE_PENDING,
    MAG_CALIB_STATE_RUNNING,
    MAG_CALIB_STATE_DONE,
    MAG_CALIB_STATE_ERROR,
    MAG_CALIB_STATE_STOP
} MagCalibState_t;

void App_MagCalib_Init(void);
void App_MagCalib_Task(void);
void App_MagCalib_Start(void);
void App_MagCalib_Stop(void);
void App_MagCalib_Toggle(void);
void App_MagCalib_ToggleDirection(void);
void App_MagCalib_SetDirection(int8_t dir_sign);
void App_MagCalib_SetTurnSpeed(int16_t wz_abs);
void App_MagCalib_PrintStatus(void);
uint8_t App_MagCalib_IsRunning(void);

#endif /* APP_MAG_CALIB_H */
