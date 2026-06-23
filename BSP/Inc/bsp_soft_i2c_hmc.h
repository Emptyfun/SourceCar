#ifndef BSP_SOFT_I2C_HMC_H
#define BSP_SOFT_I2C_HMC_H

#include "main.h"
#include <stdint.h>

#define HMC_SCL_GPIO_PORT    GPIOA
#define HMC_SCL_GPIO_PIN     GPIO_PIN_0

#define HMC_SDA_GPIO_PORT    GPIOA
#define HMC_SDA_GPIO_PIN     GPIO_PIN_1

#define HMC_DRDY_GPIO_PORT   GPIOA
#define HMC_DRDY_GPIO_PIN    GPIO_PIN_6

void SoftI2C_HMC_Init(void);

void SoftI2C_HMC_Start(void);
void SoftI2C_HMC_Stop(void);

uint8_t SoftI2C_HMC_WriteByte(uint8_t data);
uint8_t SoftI2C_HMC_ReadByte(uint8_t ack);

uint8_t SoftI2C_HMC_ReadSDA(void);
uint8_t SoftI2C_HMC_ReadDRDY(void);

#endif /* BSP_SOFT_I2C_HMC_H */
