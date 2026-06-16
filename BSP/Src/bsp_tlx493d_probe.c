#include "bsp_tlx493d_probe.h"
#include "bsp_i2c1.h"
#include <string.h>

#define BSP_TLX493D_I2C_TRIALS      2U
#define BSP_TLX493D_READY_TIMEOUT   20U
#define BSP_TLX493D_IO_TIMEOUT      50U

static uint16_t BSP_TLX493D_HALAddr(uint8_t addr7)
{
    return (uint16_t)(addr7 << 1);
}

static uint8_t BSP_TLX493D_IsReady(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    if (hi2c == NULL)
    {
        return 0U;
    }

    return (HAL_I2C_IsDeviceReady(hi2c,
                                  BSP_TLX493D_HALAddr(addr7),
                                  BSP_TLX493D_I2C_TRIALS,
                                  BSP_TLX493D_READY_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

static uint8_t BSP_TLX493D_ReadDirect(I2C_HandleTypeDef *hi2c,
                                      uint8_t addr7,
                                      uint8_t *buf,
                                      uint16_t len)
{
    if ((hi2c == NULL) || (buf == NULL) || (len == 0U))
    {
        return 0U;
    }

    return (HAL_I2C_Master_Receive(hi2c,
                                   BSP_TLX493D_HALAddr(addr7),
                                   buf,
                                   len,
                                   BSP_TLX493D_IO_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

static uint8_t BSP_TLX493D_ReadReg(I2C_HandleTypeDef *hi2c,
                                   uint8_t addr7,
                                   uint8_t reg,
                                   uint8_t *buf,
                                   uint16_t len)
{
    if ((hi2c == NULL) || (buf == NULL) || (len == 0U))
    {
        return 0U;
    }

    if (HAL_I2C_Master_Transmit(hi2c,
                                BSP_TLX493D_HALAddr(addr7),
                                &reg,
                                1U,
                                BSP_TLX493D_IO_TIMEOUT) != HAL_OK)
    {
        return 0U;
    }

    return (HAL_I2C_Master_Receive(hi2c,
                                   BSP_TLX493D_HALAddr(addr7),
                                   buf,
                                   len,
                                   BSP_TLX493D_IO_TIMEOUT) == HAL_OK) ? 1U : 0U;
}

static uint8_t BSP_TLX493D_ProbeA1B6(I2C_HandleTypeDef *hi2c,
                                     uint8_t addr7,
                                     BSP_TLX493D_ProbeInfo_t *info)
{
    uint8_t buf[10] = {0};

    if ((info == NULL) || !BSP_TLX493D_ReadDirect(hi2c, addr7, buf, sizeof(buf)))
    {
        return 0U;
    }

    memcpy(info->extra, buf, sizeof(info->extra));
    return 1U;
}

static uint8_t BSP_TLX493D_ProbeW2Version(I2C_HandleTypeDef *hi2c,
                                          uint8_t addr7,
                                          BSP_TLX493D_ProbeInfo_t *info)
{
    uint8_t ver = 0U;

    if ((info == NULL) || !BSP_TLX493D_ReadReg(hi2c, addr7, 0x16U, &ver, 1U))
    {
        return 0U;
    }

    info->extra[0] = ver;

    if ((ver == 0xC9U) || (ver == 0xD9U) || (ver == 0xE9U))
    {
        return 1U;
    }

    return 0U;
}

static uint8_t BSP_TLX493D_ProbeP3W3Id(I2C_HandleTypeDef *hi2c,
                                       uint8_t addr7,
                                       BSP_TLX493D_ProbeInfo_t *info)
{
    uint8_t id[6] = {0};

    if ((info == NULL) || !BSP_TLX493D_ReadReg(hi2c, addr7, 0x15U, id, sizeof(id)))
    {
        return 0U;
    }

    memset(info->extra, 0, sizeof(info->extra));
    memcpy(info->extra, id, sizeof(id));

    return 1U;
}

BSP_TLX493D_ProbeInfo_t BSP_TLX493D_Probe(I2C_HandleTypeDef *hi2c)
{
    static const uint8_t candidate_addrs[] =
    {
        0x5EU, 0x1FU,
        0x35U,
        0x5DU, 0x13U, 0x29U, 0x46U
    };

    BSP_TLX493D_ProbeInfo_t info;
    memset(&info, 0, sizeof(info));
    info.result = BSP_TLX493D_PROBE_NONE;

    for (uint32_t i = 0U; i < (sizeof(candidate_addrs) / sizeof(candidate_addrs[0])); i++)
    {
        uint8_t addr = candidate_addrs[i];

        if (!BSP_TLX493D_IsReady(hi2c, addr))
        {
            continue;
        }

        info.addr7 = addr;
        info.result = BSP_TLX493D_PROBE_UNKNOWN_I2C_DEVICE;

        if ((addr == 0x5EU) || (addr == 0x1FU))
        {
            if (BSP_TLX493D_ProbeA1B6(hi2c, addr, &info))
            {
                info.result = BSP_TLX493D_PROBE_A1B6_LIKELY;
                return info;
            }

            return info;
        }

        if (addr == 0x35U)
        {
            if (BSP_TLX493D_ProbeW2Version(hi2c, addr, &info))
            {
                info.result = BSP_TLX493D_PROBE_W2B6_LIKELY;
                return info;
            }

            info.result = BSP_TLX493D_PROBE_A2B6_OR_W2B6;
            return info;
        }

        if ((addr == 0x5DU) || (addr == 0x13U) || (addr == 0x29U) || (addr == 0x46U))
        {
            if (BSP_TLX493D_ProbeP3W3Id(hi2c, addr, &info))
            {
                info.result = BSP_TLX493D_PROBE_P3B6_OR_W3B6;
                return info;
            }

            info.result = BSP_TLX493D_PROBE_P3B6_OR_W3B6;
            return info;
        }

        return info;
    }

    return info;
}

BSP_TLX493D_ProbeInfo_t BSP_TLX493D_ProbeI2C1(void)
{
    return BSP_TLX493D_Probe(&hi2c1);
}

const char *BSP_TLX493D_ProbeResultToString(BSP_TLX493D_ProbeResult_t result)
{
    switch (result)
    {
    case BSP_TLX493D_PROBE_NONE:
        return "No known TLx493D device found";

    case BSP_TLX493D_PROBE_A1B6_LIKELY:
        return "TLV493D-A1B6 likely";

    case BSP_TLX493D_PROBE_A2B6_OR_W2B6:
        return "TLE493D-A2B6 or TLE493D-W2B6";

    case BSP_TLX493D_PROBE_W2B6_LIKELY:
        return "TLE493D-W2B6 likely";

    case BSP_TLX493D_PROBE_P3B6_OR_W3B6:
        return "TLE493D-P3B6 or TLE493D-W3B6-Bx";

    case BSP_TLX493D_PROBE_UNKNOWN_I2C_DEVICE:
    default:
        return "Unknown I2C device at TLx493D candidate address";
    }
}
