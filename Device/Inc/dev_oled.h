#ifndef DEV_OLED_H
#define DEV_OLED_H

#include "stm32f1xx_hal.h"

#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define OLED_WIDTH 128U
#define OLED_HEIGHT 64U

#define OLED_FONT_6X8 8U
#define OLED_FONT_10X14 16U

#define OLED_CMD 0U
#define OLED_DATA 1U

#define OLED_SCL_Clr() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)
#define OLED_SCL_Set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)

#define OLED_SDA_Clr() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define OLED_SDA_Set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)

#define OLED_RES_Clr() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define OLED_RES_Set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)

#define OLED_DC_Clr() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define OLED_DC_Set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)

#define OLED_CS_Clr() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define OLED_CS_Set() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode);
void OLED_ShowString(uint8_t x, uint8_t y, const uint8_t *str, uint8_t size, uint8_t mode);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size, uint8_t mode);

#endif /* DEV_OLED_H */
