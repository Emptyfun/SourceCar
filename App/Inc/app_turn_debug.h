#ifndef APP_TURN_DEBUG_H
#define APP_TURN_DEBUG_H

#include <stdint.h>

void App_TurnDebug_Init(void);
void App_TurnDebug_Task(void);
void App_TurnDebug_HandleCommand(const char *cmd);
void App_TurnDebug_Stop(void);

#endif /* APP_TURN_DEBUG_H */
