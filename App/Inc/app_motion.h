#ifndef APP_MOTION_H
#define APP_MOTION_H

#include <stdint.h>

void Motion_Init(void);
void Motion_Task(void);
void Motion_Stop(void);
void Motion_StopOutput(void);
void Motion_SetWheelSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4);
void Motion_SetVxWz(int16_t vx, int16_t wz);
uint8_t Motion_IsMoving(void);

#endif /* APP_MOTION_H */
