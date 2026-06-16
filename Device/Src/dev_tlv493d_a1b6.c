#include "dev_tlv493d_a1b6.h"

#include <math.h>
#include <string.h>

#define TLV493D_A1B6_I2C_TRIALS      2U
#define TLV493D_A1B6_READY_TIMEOUT   20U
#define TLV493D_A1B6_READ_TIMEOUT    50U
#define TLV493D_A1B6_WRITE_TIMEOUT   50U
#define TLV493D_A1B6_CONVERSION_MS   20U

#define TLV493D_A1B6_W_FAST          0x02U
#define TLV493D_A1B6_W_LOWPOWER      0x01U
#define TLV493D_A1B6_W_LP_PERIOD     0x40U
#define TLV493D_A1B6_W_PARITY_EN     0x20U
#define TLV493D_A1B6_W_TEMP_NEN      0x80U
#define TLV493D_A1B6_W_PARITY        0x80U

static uint16_t TLV493D_A1B6_HALAddr(uint8_t addr7)
{
    return (uint16_t)(addr7 << 1);
}

static int16_t TLV493D_A1B6_SignExtend12(uint16_t raw12)
{
    raw12 &= 0x0FFFu;

    if (raw12 & 0x0800u)
    {
        return (int16_t)(raw12 | 0xF000u);
    }

    return (int16_t)raw12;
}

static void TLV493D_A1B6_CalcParity(uint8_t *w)
{
    uint8_t y;

    if (w == NULL)
    {
        return;
    }

    /*
     * This follows Infineon's TLV493D-A1B6 Arduino library calcParity():
     * set W_PARITY to 1, XOR all four write bytes, fold that XOR to one bit,
     * then write the folded LSB back to W_PARITY. The final write block has
     * odd parity.
     */
    w[1] |= TLV493D_A1B6_W_PARITY;

    y = (uint8_t)(w[0] ^ w[1] ^ w[2] ^ w[3]);
    y = (uint8_t)(y ^ (y >> 1));
    y = (uint8_t)(y ^ (y >> 2));
    y = (uint8_t)(y ^ (y >> 4));

    if (y & 0x01U)
    {
        w[1] |= TLV493D_A1B6_W_PARITY;
    }
    else
    {
        w[1] &= (uint8_t)~TLV493D_A1B6_W_PARITY;
    }
}

static void TLV493D_A1B6_BuildWriteRegs(TLV493D_A1B6_Handle_t *dev)
{
    dev->write_reg[0] = 0x00U;
    dev->write_reg[1] = (uint8_t)(dev->raw[7] & 0x18U);
    dev->write_reg[2] = dev->raw[8];
    dev->write_reg[3] = (uint8_t)(dev->raw[9] & 0x1FU);
}

static void TLV493D_A1B6_ApplyMode(TLV493D_A1B6_Handle_t *dev,
                                   TLV493D_A1B6_Mode_t mode)
{
    dev->write_reg[1] &= (uint8_t)~(TLV493D_A1B6_W_PARITY |
                                    TLV493D_A1B6_W_FAST |
                                    TLV493D_A1B6_W_LOWPOWER);
    dev->write_reg[3] &= (uint8_t)~(TLV493D_A1B6_W_TEMP_NEN |
                                    TLV493D_A1B6_W_LP_PERIOD |
                                    TLV493D_A1B6_W_PARITY_EN);

    dev->write_reg[3] |= TLV493D_A1B6_W_PARITY_EN;

    switch (mode)
    {
    case TLV493D_A1B6_MODE_FAST:
        dev->write_reg[1] |= TLV493D_A1B6_W_FAST;
        break;

    case TLV493D_A1B6_MODE_ULTRA_LOW_POWER:
        dev->write_reg[1] |= TLV493D_A1B6_W_LOWPOWER;
        break;

    case TLV493D_A1B6_MODE_LOW_POWER:
        dev->write_reg[1] |= TLV493D_A1B6_W_LOWPOWER;
        dev->write_reg[3] |= TLV493D_A1B6_W_LP_PERIOD;
        break;

    case TLV493D_A1B6_MODE_MASTER_CONTROLLED:
        dev->write_reg[1] |= (TLV493D_A1B6_W_FAST | TLV493D_A1B6_W_LOWPOWER);
        dev->write_reg[3] |= TLV493D_A1B6_W_LP_PERIOD;
        break;

    case TLV493D_A1B6_MODE_POWER_DOWN:
    default:
        break;
    }
}

static void TLV493D_A1B6_Parse(TLV493D_A1B6_Handle_t *dev)
{
    uint16_t bx = ((uint16_t)dev->raw[0] << 4) | ((uint16_t)dev->raw[4] >> 4);
    uint16_t by = ((uint16_t)dev->raw[1] << 4) | ((uint16_t)dev->raw[4] & 0x0Fu);
    uint16_t bz = ((uint16_t)dev->raw[2] << 4) | ((uint16_t)dev->raw[5] & 0x0Fu);
    uint16_t temp = (((uint16_t)dev->raw[3] & 0xF0u) << 4) | dev->raw[6];

    dev->bx_raw = TLV493D_A1B6_SignExtend12(bx);
    dev->by_raw = TLV493D_A1B6_SignExtend12(by);
    dev->bz_raw = TLV493D_A1B6_SignExtend12(bz);
    dev->temp_raw = TLV493D_A1B6_SignExtend12(temp);

    dev->bx_mT = (float)dev->bx_raw * TLV493D_A1B6_MTLSB;
    dev->by_mT = (float)dev->by_raw * TLV493D_A1B6_MTLSB;
    dev->bz_mT = (float)dev->bz_raw * TLV493D_A1B6_MTLSB;
    dev->b_abs_mT = sqrtf((dev->bx_mT * dev->bx_mT) +
                          (dev->by_mT * dev->by_mT) +
                          (dev->bz_mT * dev->bz_mT));

    /* Temperature is secondary debug data and should be verified against the official library/datasheet. */
    dev->temperature_C = ((float)dev->temp_raw - 315.0f) * 1.1f;
}

TLV493D_A1B6_Status_t TLV493D_A1B6_Init(TLV493D_A1B6_Handle_t *dev,
                                        I2C_HandleTypeDef *hi2c,
                                        uint8_t addr7)
{
    if ((dev == NULL) || (hi2c == NULL))
    {
        return TLV493D_A1B6_ERROR_NULL;
    }

    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;
    dev->addr7 = addr7;

    if (HAL_I2C_IsDeviceReady(hi2c,
                              TLV493D_A1B6_HALAddr(addr7),
                              TLV493D_A1B6_I2C_TRIALS,
                              TLV493D_A1B6_READY_TIMEOUT) != HAL_OK)
    {
        dev->last_hal_error = HAL_I2C_GetError(hi2c);
        return TLV493D_A1B6_ERROR_I2C_NOT_READY;
    }

    dev->last_hal_error = HAL_I2C_GetError(hi2c);
    return TLV493D_A1B6_OK;
}

TLV493D_A1B6_Status_t TLV493D_A1B6_AutoDetect(TLV493D_A1B6_Handle_t *dev,
                                              I2C_HandleTypeDef *hi2c)
{
    TLV493D_A1B6_Status_t status;

    if ((dev == NULL) || (hi2c == NULL))
    {
        return TLV493D_A1B6_ERROR_NULL;
    }

    status = TLV493D_A1B6_Init(dev, hi2c, TLV493D_A1B6_ADDR1_7BIT);
    if (status == TLV493D_A1B6_OK)
    {
        return status;
    }

    status = TLV493D_A1B6_Init(dev, hi2c, TLV493D_A1B6_ADDR2_7BIT);
    if (status == TLV493D_A1B6_OK)
    {
        return status;
    }

    return TLV493D_A1B6_ERROR_NO_DEVICE;
}

TLV493D_A1B6_Status_t TLV493D_A1B6_ReadRaw(TLV493D_A1B6_Handle_t *dev)
{
    if ((dev == NULL) || (dev->hi2c == NULL))
    {
        return TLV493D_A1B6_ERROR_NULL;
    }

    if (HAL_I2C_Master_Receive(dev->hi2c,
                               TLV493D_A1B6_HALAddr(dev->addr7),
                               dev->raw,
                               sizeof(dev->raw),
                               TLV493D_A1B6_READ_TIMEOUT) != HAL_OK)
    {
        dev->last_hal_error = HAL_I2C_GetError(dev->hi2c);
        return TLV493D_A1B6_ERROR_I2C_READ;
    }

    dev->last_hal_error = HAL_I2C_GetError(dev->hi2c);
    return TLV493D_A1B6_OK;
}

TLV493D_A1B6_Status_t TLV493D_A1B6_Update(TLV493D_A1B6_Handle_t *dev)
{
    TLV493D_A1B6_Status_t status = TLV493D_A1B6_ReadRaw(dev);

    if (status != TLV493D_A1B6_OK)
    {
        return status;
    }

    TLV493D_A1B6_Parse(dev);
    dev->sample_count++;

    return TLV493D_A1B6_OK;
}

TLV493D_A1B6_Status_t TLV493D_A1B6_ConfigureMode(TLV493D_A1B6_Handle_t *dev,
                                                 TLV493D_A1B6_Mode_t mode)
{
    TLV493D_A1B6_Status_t status;

    if ((dev == NULL) || (dev->hi2c == NULL))
    {
        return TLV493D_A1B6_ERROR_NULL;
    }

    status = TLV493D_A1B6_ReadRaw(dev);
    if (status != TLV493D_A1B6_OK)
    {
        return status;
    }

    TLV493D_A1B6_BuildWriteRegs(dev);
    TLV493D_A1B6_ApplyMode(dev, mode);
    TLV493D_A1B6_CalcParity(dev->write_reg);

    if (HAL_I2C_Master_Transmit(dev->hi2c,
                                TLV493D_A1B6_HALAddr(dev->addr7),
                                dev->write_reg,
                                sizeof(dev->write_reg),
                                TLV493D_A1B6_WRITE_TIMEOUT) != HAL_OK)
    {
        dev->last_hal_error = HAL_I2C_GetError(dev->hi2c);
        return TLV493D_A1B6_ERROR_I2C_WRITE;
    }

    dev->last_hal_error = HAL_I2C_GetError(dev->hi2c);
    dev->mode = mode;

    HAL_Delay(TLV493D_A1B6_CONVERSION_MS);
    return TLV493D_A1B6_Update(dev);
}

TLV493D_A1B6_Status_t TLV493D_A1B6_StartLowPowerMeasurement(TLV493D_A1B6_Handle_t *dev)
{
    return TLV493D_A1B6_ConfigureMode(dev, TLV493D_A1B6_MODE_LOW_POWER);
}

TLV493D_A1B6_Status_t TLV493D_A1B6_StartFastMeasurement(TLV493D_A1B6_Handle_t *dev)
{
    return TLV493D_A1B6_ConfigureMode(dev, TLV493D_A1B6_MODE_FAST);
}

const char *TLV493D_A1B6_StatusToString(TLV493D_A1B6_Status_t status)
{
    switch (status)
    {
    case TLV493D_A1B6_OK:
        return "OK";

    case TLV493D_A1B6_ERROR_NULL:
        return "ERROR_NULL";

    case TLV493D_A1B6_ERROR_I2C_NOT_READY:
        return "ERROR_I2C_NOT_READY";

    case TLV493D_A1B6_ERROR_I2C_READ:
        return "ERROR_I2C_READ";

    case TLV493D_A1B6_ERROR_I2C_WRITE:
        return "ERROR_I2C_WRITE";

    case TLV493D_A1B6_ERROR_NO_DEVICE:
    default:
        return "ERROR_NO_DEVICE";
    }
}
