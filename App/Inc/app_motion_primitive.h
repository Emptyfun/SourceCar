#ifndef APP_MOTION_PRIMITIVE_H
#define APP_MOTION_PRIMITIVE_H

#include <stdint.h>

typedef enum
{
    PRIM_IDLE = 0,
    PRIM_MOVE_DISTANCE,
    PRIM_TURN_ANGLE,
    PRIM_DONE,
    PRIM_ERROR
} MotionPrimitiveState_t;

void MotionPrimitive_Init(void);
void MotionPrimitive_MoveDistance_Start(float distance_mm, int16_t speed);
void MotionPrimitive_TurnAngle_Start(float angle_deg, int16_t turn_speed);
void MotionPrimitive_Task(void);
uint8_t MotionPrimitive_IsBusy(void);
uint8_t MotionPrimitive_IsDone(void);
uint8_t MotionPrimitive_IsError(void);
void MotionPrimitive_ClearDone(void);
void MotionPrimitive_Abort(void);
MotionPrimitiveState_t MotionPrimitive_GetState(void);

#endif /* APP_MOTION_PRIMITIVE_H */
