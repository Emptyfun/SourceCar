#ifndef APP_STATE_H
#define APP_STATE_H

/**
 * @file app_state.h
 * @brief Application state definitions for source car behavior.
 *
 * The enum records high-level firmware modes without placing state-machine
 * logic in the BSP or Core layers.
 */

/**
 * @brief High-level application states.
 */
typedef enum
{
    APP_STATE_INIT = 0,
    APP_STATE_IDLE,
    APP_STATE_MANUAL_TEST,
    APP_STATE_SOURCE_SEARCH,
    APP_STATE_SOURCE_TRACKING,
    APP_STATE_OBSTACLE_AVOID,
    APP_STATE_ARRIVED,
    APP_STATE_ERROR
} AppState_t;

#endif /* APP_STATE_H */
