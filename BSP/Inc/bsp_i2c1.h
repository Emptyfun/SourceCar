#ifndef BSP_I2C1_H
#define BSP_I2C1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

void BSP_I2C1_Init(void);

#ifdef __cplusplus
}
#endif

#endif
