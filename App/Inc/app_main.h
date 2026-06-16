#ifndef APP_MAIN_H
#define APP_MAIN_H

/**
 * @file app_main.h
 * @brief Public interface for the application entry layer.
 *
 * The App layer separates firmware behavior from board setup so main.c only
 * sequences initialization and repeatedly calls the application loop.
 */

/**
 * @brief Initialize application-level modules after BSP initialization.
 */
void App_Init(void);

/**
 * @brief Run non-blocking application tasks from the main loop.
 */
void App_Loop(void);

#endif /* APP_MAIN_H */
