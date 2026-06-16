#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

/**
 * @file bsp_system.h
 * @brief Public interface for board-level system initialization.
 */

/**
 * @brief Initialize HAL, system clock, GPIO, and UART peripherals.
 */
void BSP_System_Init(void);

#endif /* BSP_SYSTEM_H */
