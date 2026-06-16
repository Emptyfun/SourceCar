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

typedef enum
{
    CS1238_SPEED_10HZ = 0U,
    CS1238_SPEED_40HZ = 1U,
    CS1238_SPEED_640HZ = 2U,
    CS1238_SPEED_1280HZ = 3U
} CS1238_SpeedSel;

typedef enum
{
    CS1238_PGA_SEL_1 = 0U,
    CS1238_PGA_SEL_2 = 1U,
    CS1238_PGA_SEL_64 = 2U,
    CS1238_PGA_SEL_128 = 3U
} CS1238_PgaSel;

typedef enum
{
    CS1238_CH_A = 0U,
    CS1238_CH_B = 1U,
    CS1238_CH_TEMPERATURE = 2U,
    CS1238_CH_SHORT = 3U
} CS1238_ChannelSel;

extern CS1238_Handle hcs1238_1;
extern CS1238_Handle hcs1238_2;

void CS1238_Init(CS1238_Handle *h);
void CS1238_InitAll(void);
uint8_t CS1238_IsDataReady(CS1238_Handle *h);
HAL_StatusTypeDef CS1238_ReadRaw(CS1238_Handle *h, int32_t *raw, uint32_t timeout_ms);
HAL_StatusTypeDef CS1238_WriteConfig(CS1238_Handle *h, uint8_t cfg);
uint8_t CS1238_MakeConfig(uint8_t refo_off, uint8_t speed, uint8_t pga, uint8_t ch);
uint8_t CS1238_PgaGainFromSel(uint8_t pga);
float CS1238_RawToVoltage(int32_t raw, float vref, uint8_t pga);

#endif /* DEV_CS1238_H */
