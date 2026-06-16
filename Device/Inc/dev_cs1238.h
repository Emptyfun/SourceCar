#ifndef DEV_CS1238_H
#define DEV_CS1238_H

#include "stm32f1xx_hal.h"

typedef struct
{
    GPIO_TypeDef *clk_port;
    uint16_t clk_pin;

    GPIO_TypeDef *data_port;
    uint16_t data_pin;
} CS1238_Handle;

extern CS1238_Handle hcs1238_1;
extern CS1238_Handle hcs1238_2;

void CS1238_InitAll(void);
uint8_t CS1238_IsDataReady(CS1238_Handle *h);
HAL_StatusTypeDef CS1238_ReadRaw(CS1238_Handle *h, int32_t *raw, uint32_t timeout_ms);
HAL_StatusTypeDef CS1238_WriteConfig(CS1238_Handle *h, uint8_t cfg);
uint8_t CS1238_MakeConfig(uint8_t refo_off, uint8_t speed, uint8_t pga, uint8_t ch);
float CS1238_RawToVoltage(int32_t raw, float vref, uint8_t pga);

#endif /* DEV_CS1238_H */
