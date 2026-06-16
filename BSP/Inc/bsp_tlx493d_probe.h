#ifndef BSP_TLX493D_PROBE_H
#define BSP_TLX493D_PROBE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum
{
    BSP_TLX493D_PROBE_NONE = 0,
    BSP_TLX493D_PROBE_A1B6_LIKELY,
    BSP_TLX493D_PROBE_A2B6_OR_W2B6,
    BSP_TLX493D_PROBE_W2B6_LIKELY,
    BSP_TLX493D_PROBE_P3B6_OR_W3B6,
    BSP_TLX493D_PROBE_UNKNOWN_I2C_DEVICE
} BSP_TLX493D_ProbeResult_t;

typedef struct
{
    uint8_t addr7;
    BSP_TLX493D_ProbeResult_t result;
    uint8_t extra[8];
} BSP_TLX493D_ProbeInfo_t;

BSP_TLX493D_ProbeInfo_t BSP_TLX493D_Probe(I2C_HandleTypeDef *hi2c);
BSP_TLX493D_ProbeInfo_t BSP_TLX493D_ProbeI2C1(void);

const char *BSP_TLX493D_ProbeResultToString(BSP_TLX493D_ProbeResult_t result);

/*
 * Example:
 *
 * #include "bsp_i2c1.h"
 * #include "bsp_tlx493d_probe.h"
 *
 * void App_MagProbeTest(void)
 * {
 *     BSP_I2C1_Init();
 *
 *     BSP_TLX493D_ProbeInfo_t info = BSP_TLX493D_ProbeI2C1();
 *
 *     info.addr7;
 *     info.result;
 *     info.extra;
 * }
 */

#ifdef __cplusplus
}
#endif

#endif
