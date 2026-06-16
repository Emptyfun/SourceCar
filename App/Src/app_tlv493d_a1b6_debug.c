#include "app_tlv493d_a1b6_debug.h"

#include "bsp_i2c1.h"
#include "dev_tlv493d_a1b6.h"
#include "stm32f1xx_hal.h"
#include "usart.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define APP_TLV493D_DEBUG_INTERVAL_MS 500U
#define TLV_A1B6_DEBUG_VERBOSE_RAW       0
#define TLV_A1B6_DEBUG_VERBOSE_STATUS    0
#define TLV_A1B6_DEBUG_VERBOSE_WARNING   0

static TLV493D_A1B6_Handle_t g_tlv493d;
static uint8_t g_tlv493d_detected;

#if TLV_A1B6_DEBUG_VERBOSE_WARNING
static const uint8_t g_powerdown_raw_pattern[10] =
{
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x30U, 0x00U, 0x80U, 0x08U, 0x20U
};
#endif

static void MagPrint(const char *s)
{
    if (s == NULL)
    {
        return;
    }

    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 200);
}

static void MagFormatFloat2(char *buf, size_t len, float value)
{
    int32_t scaled;
    const char *sign = "";

    if ((buf == NULL) || (len == 0U))
    {
        return;
    }

    if (value >= 0.0f)
    {
        scaled = (int32_t)((value * 100.0f) + 0.5f);
    }
    else
    {
        scaled = (int32_t)((value * 100.0f) - 0.5f);
    }

    if (scaled < 0)
    {
        sign = "-";
        scaled = -scaled;
    }

    snprintf(buf, len, "%s%ld.%02ld", sign, (long)(scaled / 100), (long)(scaled % 100));
}

#if TLV_A1B6_DEBUG_VERBOSE_WARNING
static uint8_t MagRawEquals(const uint8_t *a, const uint8_t *b)
{
    if ((a == NULL) || (b == NULL))
    {
        return 0U;
    }

    return (memcmp(a, b, sizeof(g_tlv493d.raw)) == 0) ? 1U : 0U;
}
#endif

static void MagPrintAddressCheck(uint8_t addr7)
{
    char msg[128];
    uint16_t hal_addr = (uint16_t)(addr7 << 1);
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, hal_addr, 2, 20);
    uint32_t err = HAL_I2C_GetError(&hi2c1);

    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] check addr7=0x%02X hal=0x%02X %s err=0x%08lX\r\n",
             (unsigned int)addr7,
             (unsigned int)hal_addr,
             (st == HAL_OK) ? "ACK" : "NACK",
             (unsigned long)err);
    MagPrint(msg);
}

static void MagPrintRawLine(const TLV493D_A1B6_Handle_t *dev, const char *label)
{
#if TLV_A1B6_DEBUG_VERBOSE_RAW
    char msg[144];

    if (dev == NULL)
    {
        return;
    }

    if (label == NULL)
    {
        label = "raw";
    }

    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] %s=%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
             label,
             (unsigned int)dev->raw[0], (unsigned int)dev->raw[1],
             (unsigned int)dev->raw[2], (unsigned int)dev->raw[3],
             (unsigned int)dev->raw[4], (unsigned int)dev->raw[5],
             (unsigned int)dev->raw[6], (unsigned int)dev->raw[7],
             (unsigned int)dev->raw[8], (unsigned int)dev->raw[9]);
    MagPrint(msg);
#else
    (void)dev;
    (void)label;
#endif
}

static void MagPrintStatusDetailLines(const TLV493D_A1B6_Handle_t *dev)
{
#if TLV_A1B6_DEBUG_VERBOSE_STATUS
    char msg[128];
    uint8_t pdown;
    uint8_t frame;
    uint8_t channel;
    uint8_t bz_lsb;

    if (dev == NULL)
    {
        return;
    }

    pdown = (dev->raw[5] & 0x10U) ? 1U : 0U;
    bz_lsb = (uint8_t)(dev->raw[5] & 0x0FU);
    frame = (uint8_t)((dev->raw[3] & 0x0CU) >> 2);
    channel = (uint8_t)(dev->raw[3] & 0x03U);

    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] status byte raw[5]=0x%02X pdown=%u bz_lsb=0x%X\r\n",
             (unsigned int)dev->raw[5],
             (unsigned int)pdown,
             (unsigned int)bz_lsb);
    MagPrint(msg);

    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] frame_channel raw[3]=0x%02X frame=%u channel=%u\r\n",
             (unsigned int)dev->raw[3],
             (unsigned int)frame,
             (unsigned int)channel);
    MagPrint(msg);
#else
    (void)dev;
#endif
}

static void MagPrintPeriodicStatusLine(const TLV493D_A1B6_Handle_t *dev)
{
#if TLV_A1B6_DEBUG_VERBOSE_STATUS
    char msg[128];
    uint8_t pdown;
    uint8_t frame;
    uint8_t channel;

    if (dev == NULL)
    {
        return;
    }

    pdown = (dev->raw[5] & 0x10U) ? 1U : 0U;
    frame = (uint8_t)((dev->raw[3] & 0x0CU) >> 2);
    channel = (uint8_t)(dev->raw[3] & 0x03U);

    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] status raw3=0x%02X raw5=0x%02X pdown=%u frame=%u channel=%u\r\n",
             (unsigned int)dev->raw[3],
             (unsigned int)dev->raw[5],
             (unsigned int)pdown,
             (unsigned int)frame,
             (unsigned int)channel);
    MagPrint(msg);
#else
    (void)dev;
#endif
}

static void MagPrintRawValuesLine(const TLV493D_A1B6_Handle_t *dev)
{
#if TLV_A1B6_DEBUG_VERBOSE_RAW
    char msg[128];

    if (dev == NULL)
    {
        return;
    }

    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] raw_xyz bx=%d by=%d bz=%d temp=%d\r\n",
             (int)dev->bx_raw,
             (int)dev->by_raw,
             (int)dev->bz_raw,
             (int)dev->temp_raw);
    MagPrint(msg);
#else
    (void)dev;
#endif
}

static void MagPrintConvertedLine(const TLV493D_A1B6_Handle_t *dev)
{
    char bx[20];
    char by[20];
    char bz[20];
    char b_abs[20];
    char temp[20];
    char msg[160];

    if (dev == NULL)
    {
        return;
    }

    MagFormatFloat2(bx, sizeof(bx), dev->bx_mT);
    MagFormatFloat2(by, sizeof(by), dev->by_mT);
    MagFormatFloat2(bz, sizeof(bz), dev->bz_mT);
    MagFormatFloat2(b_abs, sizeof(b_abs), dev->b_abs_mT);
    MagFormatFloat2(temp, sizeof(temp), dev->temperature_C);

    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] mT bx=%s by=%s bz=%s abs=%s tempC=%s\r\n",
             bx, by, bz, b_abs, temp);
    MagPrint(msg);
}

static void MagPrintConfigLine(const TLV493D_A1B6_Handle_t *dev)
{
    char msg[96];

    if (dev == NULL)
    {
        return;
    }

    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] config write=%02X %02X %02X %02X\r\n",
             (unsigned int)dev->write_reg[0],
             (unsigned int)dev->write_reg[1],
             (unsigned int)dev->write_reg[2],
             (unsigned int)dev->write_reg[3]);
    MagPrint(msg);
}

static void MagPrintPowerdownWarning(const TLV493D_A1B6_Handle_t *dev)
{
#if TLV_A1B6_DEBUG_VERBOSE_WARNING
    if ((dev != NULL) && ((dev->raw[5] & 0x10U) != 0U))
    {
        MagPrint("[TLV-A1B6] WARNING: power-down/status bit still set.\r\n");
    }
#else
    (void)dev;
#endif
}

void App_TLV493D_A1B6_DebugRunOnce(void)
{
    char msg[160];
    uint8_t raw_before[10] = {0};
    TLV493D_A1B6_Status_t status;

    MagPrint("\r\n[TLV-A1B6] debug start\r\n");

    BSP_I2C1_Init();
    MagPrint("[TLV-A1B6] I2C1 init done PB6=SCL PB7=SDA speed=100k\r\n");

    MagPrintAddressCheck(TLV493D_A1B6_ADDR1_7BIT);
    MagPrintAddressCheck(TLV493D_A1B6_ADDR2_7BIT);

    status = TLV493D_A1B6_AutoDetect(&g_tlv493d, &hi2c1);
    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] autodetect status=%s selected addr7=0x%02X\r\n",
             TLV493D_A1B6_StatusToString(status),
             (unsigned int)g_tlv493d.addr7);
    MagPrint(msg);

    if (status != TLV493D_A1B6_OK)
    {
        g_tlv493d_detected = 0U;
        MagPrint("[TLV-A1B6] no A1B6 detected. Check VDD=3.3V, GND, PB6/PB7, pull-ups, and whether the sensor board MCU is occupying I2C.\r\n");
        return;
    }

    status = TLV493D_A1B6_Update(&g_tlv493d);
    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] raw before config status=%s halerr=0x%08lX\r\n",
             TLV493D_A1B6_StatusToString(status),
             (unsigned long)g_tlv493d.last_hal_error);
    MagPrint(msg);
    MagPrintRawLine(&g_tlv493d, "raw before config");
    MagPrintStatusDetailLines(&g_tlv493d);
    MagPrintRawValuesLine(&g_tlv493d);
    MagPrintConvertedLine(&g_tlv493d);
    memcpy(raw_before, g_tlv493d.raw, sizeof(raw_before));

    MagPrint("[TLV-A1B6] start mode=FAST\r\n");
    status = TLV493D_A1B6_StartFastMeasurement(&g_tlv493d);
    MagPrintConfigLine(&g_tlv493d);
    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] config status=%s halerr=0x%08lX\r\n",
             TLV493D_A1B6_StatusToString(status),
             (unsigned long)g_tlv493d.last_hal_error);
    MagPrint(msg);

    if (status == TLV493D_A1B6_ERROR_I2C_WRITE)
    {
        snprintf(msg, sizeof(msg),
                 "[TLV-A1B6] ERROR: config write failed halerr=0x%08lX\r\n",
                 (unsigned long)g_tlv493d.last_hal_error);
        MagPrint(msg);
    }

    MagPrintRawLine(&g_tlv493d, "raw after config");
    MagPrintStatusDetailLines(&g_tlv493d);
    MagPrintRawValuesLine(&g_tlv493d);
    MagPrintConvertedLine(&g_tlv493d);

#if TLV_A1B6_DEBUG_VERBOSE_WARNING
    if ((status == TLV493D_A1B6_OK) &&
        (MagRawEquals(raw_before, g_tlv493d.raw) ||
         MagRawEquals(g_tlv493d.raw, g_powerdown_raw_pattern)))
    {
        MagPrint("[TLV-A1B6] WARNING: raw data unchanged after mode config. Sensor may still be in power-down, config write failed, another MCU may own the bus, or this may not be TLV493D-A1B6.\r\n");
    }
#else
    (void)raw_before;
#endif

    MagPrintPowerdownWarning(&g_tlv493d);

    g_tlv493d_detected = (status == TLV493D_A1B6_OK) ? 1U : 0U;
}

void App_TLV493D_A1B6_DebugLoopTick(void)
{
    static uint32_t last_sample_tick = 0U;
    uint32_t now = HAL_GetTick();
    TLV493D_A1B6_Status_t status;

    if (!g_tlv493d_detected)
    {
        return;
    }

    if ((uint32_t)(now - last_sample_tick) < APP_TLV493D_DEBUG_INTERVAL_MS)
    {
        return;
    }

    last_sample_tick = now;
    status = TLV493D_A1B6_Update(&g_tlv493d);

#if TLV_A1B6_DEBUG_VERBOSE_STATUS
    char msg[160];
    snprintf(msg, sizeof(msg),
             "[TLV-A1B6] sample=%lu addr7=0x%02X status=%s halerr=0x%08lX\r\n",
             (unsigned long)g_tlv493d.sample_count,
             (unsigned int)g_tlv493d.addr7,
             TLV493D_A1B6_StatusToString(status),
             (unsigned long)g_tlv493d.last_hal_error);
    MagPrint(msg);
#else
    (void)status;
#endif

    MagPrintRawLine(&g_tlv493d, "raw");
    MagPrintPeriodicStatusLine(&g_tlv493d);
    MagPrintRawValuesLine(&g_tlv493d);
    MagPrintConvertedLine(&g_tlv493d);
    MagPrintPowerdownWarning(&g_tlv493d);
}
