#ifndef DEV_TLV493D_A1B6_H
#define DEV_TLV493D_A1B6_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

#define TLV493D_A1B6_ADDR1_7BIT    0x5E
#define TLV493D_A1B6_ADDR2_7BIT    0x1F
#define TLV493D_A1B6_MTLSB         0.098f

typedef enum
{
    TLV493D_A1B6_OK = 0,
    TLV493D_A1B6_ERROR_NULL,
    TLV493D_A1B6_ERROR_I2C_NOT_READY,
    TLV493D_A1B6_ERROR_I2C_READ,
    TLV493D_A1B6_ERROR_I2C_WRITE,
    TLV493D_A1B6_ERROR_NO_DEVICE
} TLV493D_A1B6_Status_t;

typedef enum
{
    TLV493D_A1B6_MODE_POWER_DOWN = 0,
    TLV493D_A1B6_MODE_ULTRA_LOW_POWER,
    TLV493D_A1B6_MODE_LOW_POWER,
    TLV493D_A1B6_MODE_MASTER_CONTROLLED,
    TLV493D_A1B6_MODE_FAST
} TLV493D_A1B6_Mode_t;

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr7;

    uint8_t raw[10];
    uint8_t write_reg[4];

    int16_t bx_raw;
    int16_t by_raw;
    int16_t bz_raw;
    int16_t temp_raw;

    float bx_mT;
    float by_mT;
    float bz_mT;
    float b_abs_mT;
    float temperature_C;

    uint32_t last_hal_error;
    uint32_t sample_count;
    TLV493D_A1B6_Mode_t mode;
} TLV493D_A1B6_Handle_t;

TLV493D_A1B6_Status_t TLV493D_A1B6_Init(TLV493D_A1B6_Handle_t *dev,
                                        I2C_HandleTypeDef *hi2c,
                                        uint8_t addr7);

TLV493D_A1B6_Status_t TLV493D_A1B6_AutoDetect(TLV493D_A1B6_Handle_t *dev,
                                              I2C_HandleTypeDef *hi2c);

TLV493D_A1B6_Status_t TLV493D_A1B6_ReadRaw(TLV493D_A1B6_Handle_t *dev);

TLV493D_A1B6_Status_t TLV493D_A1B6_Update(TLV493D_A1B6_Handle_t *dev);

TLV493D_A1B6_Status_t TLV493D_A1B6_ConfigureMode(TLV493D_A1B6_Handle_t *dev,
                                                 TLV493D_A1B6_Mode_t mode);

TLV493D_A1B6_Status_t TLV493D_A1B6_StartLowPowerMeasurement(TLV493D_A1B6_Handle_t *dev);

TLV493D_A1B6_Status_t TLV493D_A1B6_StartFastMeasurement(TLV493D_A1B6_Handle_t *dev);

const char *TLV493D_A1B6_StatusToString(TLV493D_A1B6_Status_t status);

#ifdef __cplusplus
}
#endif

#endif
