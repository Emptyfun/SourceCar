#include "app_mag_probe_debug.h"

#include "bsp_i2c1.h"
#include "bsp_tlx493d_probe.h"
#include "bsp_uart1.h"

#include "stm32f1xx_hal.h"

#include <stdio.h>
#include <string.h>

static void MagDebug_Write(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    HAL_UART_Transmit(BSP_UART1_GetHandle(),
                      (uint8_t *)s,
                      (uint16_t)strlen(s),
                      200);
}

void App_MagProbeDebug_RunOnce(void)
{
    static const uint8_t addrs[] = {0x5EU, 0x1FU, 0x35U, 0x5DU, 0x13U, 0x29U, 0x46U};
    char msg[192];

    MagDebug_Write("\r\n[MAG] probe start\r\n");

    BSP_I2C1_Init();

    for (uint32_t i = 0U; i < (sizeof(addrs) / sizeof(addrs[0])); i++)
    {
        uint8_t addr7 = addrs[i];
        uint16_t hal_addr = (uint16_t)(addr7 << 1);
        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, hal_addr, 2, 20);
        uint32_t err = HAL_I2C_GetError(&hi2c1);

        snprintf(msg, sizeof(msg),
                 "[MAG] scan addr7=0x%02X hal=0x%02X : %s err=0x%08lX\r\n",
                 (unsigned int)addr7,
                 (unsigned int)hal_addr,
                 (st == HAL_OK) ? "ACK" : "NACK",
                 (unsigned long)err);

        MagDebug_Write(msg);
    }

    BSP_TLX493D_ProbeInfo_t info = BSP_TLX493D_ProbeI2C1();

    snprintf(msg, sizeof(msg),
             "[MAG] final addr7=0x%02X hal=0x%02X result=%s\r\n",
             (unsigned int)info.addr7,
             (unsigned int)(info.addr7 << 1),
             BSP_TLX493D_ProbeResultToString(info.result));
    MagDebug_Write(msg);

    snprintf(msg, sizeof(msg),
             "[MAG] extra=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
             (unsigned int)info.extra[0], (unsigned int)info.extra[1],
             (unsigned int)info.extra[2], (unsigned int)info.extra[3],
             (unsigned int)info.extra[4], (unsigned int)info.extra[5],
             (unsigned int)info.extra[6], (unsigned int)info.extra[7]);
    MagDebug_Write(msg);

    if (info.result == BSP_TLX493D_PROBE_NONE)
    {
        MagDebug_Write("[MAG] no known TLx493D found. Check 3.3V, GND, PB6=SCL, PB7=SDA, pull-ups, and sensor board ownership.\r\n");
    }

    MagDebug_Write("[MAG] probe end\r\n");
}
