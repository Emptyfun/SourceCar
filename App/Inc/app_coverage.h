#ifndef APP_COVERAGE_H
#define APP_COVERAGE_H

#include <stdint.h>

typedef enum
{
    COVERAGE_IDLE = 0,
    COVERAGE_RUN_LINE,
    COVERAGE_TURN_OUT,
    COVERAGE_SHIFT_LINE,
    COVERAGE_TURN_BACK,
    COVERAGE_NEXT_LINE,
    COVERAGE_DONE,
    COVERAGE_ABORT,
    COVERAGE_ERROR
} CoverageState_t;

void Coverage_Init(void);
void Coverage_Start(void);
void Coverage_Stop(void);
void Coverage_Task(void);
uint8_t Coverage_IsRunning(void);
CoverageState_t Coverage_GetState(void);

#endif /* APP_COVERAGE_H */
