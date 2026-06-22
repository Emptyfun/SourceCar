#include "app_button_test.h"

#include "app_oled_ui.h"
#include "stm32f1xx_hal.h"

#define APP_BUTTON_SCAN_INTERVAL_MS 5U
#define APP_BUTTON_DEBOUNCE_MS 25U
#define APP_BUTTON_LONG_PRESS_MS 1000U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t raw_level;
    uint8_t raw_down;
    uint8_t last_raw_down;
    uint8_t stable_down;
    uint8_t long_reported;
    uint32_t raw_changed_tick;
    uint32_t press_tick;
    uint32_t hold_ms;
    uint32_t press_count;
} AppButton_State;

static AppButton_State s_k1 = {GPIOB, GPIO_PIN_8, 1U, 0U, 0U, 0U, 0U, 0UL, 0UL, 0UL, 0UL};
static AppButton_State s_k2 = {GPIOB, GPIO_PIN_9, 1U, 0U, 0U, 0U, 0U, 0UL, 0UL, 0UL, 0UL};
static uint8_t s_run_enabled;
static const char *s_last_event = "BOOT";

static void App_ButtonTest_GPIOInit(void);
static void App_ButtonTest_InitButton(AppButton_State *button, uint32_t now);
static void App_ButtonTest_UpdateButton(AppButton_State *button, uint8_t index, uint32_t now);
static void App_ButtonTest_HandleShortPress(uint8_t index);
static void App_ButtonTest_HandleLongPress(uint8_t index);
static void App_ButtonTest_ClearCounts(const char *event_text);
static void App_ButtonTest_UpdateOLED(void);
static uint8_t App_ButtonTest_ReadRawLevel(const AppButton_State *button);

void App_ButtonTest_Init(void)
{
    uint32_t now;

    App_ButtonTest_GPIOInit();
    now = HAL_GetTick();

    App_ButtonTest_InitButton(&s_k1, now);
    App_ButtonTest_InitButton(&s_k2, now);

    s_run_enabled = 0U;
    s_last_event = "BOOT";
    App_OLED_SetPage(APP_OLED_PAGE_KEY_TEST);
    App_ButtonTest_UpdateOLED();
}

void App_ButtonTest_Task(void)
{
    static uint32_t last_scan_tick = 0UL;
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_scan_tick) < APP_BUTTON_SCAN_INTERVAL_MS)
    {
        return;
    }
    last_scan_tick = now;

    App_ButtonTest_UpdateButton(&s_k1, 1U, now);
    App_ButtonTest_UpdateButton(&s_k2, 2U, now);
    App_ButtonTest_UpdateOLED();
}

static void App_ButtonTest_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

static void App_ButtonTest_InitButton(AppButton_State *button, uint32_t now)
{
    button->raw_level = App_ButtonTest_ReadRawLevel(button);
    button->raw_down = (button->raw_level == 0U) ? 1U : 0U;
    button->last_raw_down = button->raw_down;
    button->stable_down = button->raw_down;
    button->long_reported = 0U;
    button->raw_changed_tick = now;
    button->press_tick = button->stable_down ? now : 0UL;
    button->hold_ms = 0UL;
    button->press_count = 0UL;
}

static void App_ButtonTest_UpdateButton(AppButton_State *button, uint8_t index, uint32_t now)
{
    button->raw_level = App_ButtonTest_ReadRawLevel(button);
    button->raw_down = (button->raw_level == 0U) ? 1U : 0U;

    if (button->raw_down != button->last_raw_down)
    {
        button->last_raw_down = button->raw_down;
        button->raw_changed_tick = now;
    }

    if ((button->raw_down != button->stable_down) &&
        ((uint32_t)(now - button->raw_changed_tick) >= APP_BUTTON_DEBOUNCE_MS))
    {
        button->stable_down = button->raw_down;
        if (button->stable_down != 0U)
        {
            button->press_tick = now;
            button->hold_ms = 0UL;
            button->long_reported = 0U;
            button->press_count++;
            s_last_event = (index == 1U) ? "K1 DOWN" : "K2 DOWN";
        }
        else
        {
            if (button->long_reported == 0U)
            {
                App_ButtonTest_HandleShortPress(index);
            }
            button->hold_ms = 0UL;
        }
    }

    if (button->stable_down != 0U)
    {
        button->hold_ms = (uint32_t)(now - button->press_tick);
        if ((button->long_reported == 0U) && (button->hold_ms >= APP_BUTTON_LONG_PRESS_MS))
        {
            button->long_reported = 1U;
            App_ButtonTest_HandleLongPress(index);
        }
    }
}

static void App_ButtonTest_HandleShortPress(uint8_t index)
{
    if (index == 1U)
    {
        s_run_enabled = (s_run_enabled == 0U) ? 1U : 0U;
        s_last_event = (s_run_enabled != 0U) ? "K1 RUN ON" : "K1 RUN OFF";
    }
    else
    {
        App_OLED_TogglePage();
        s_last_event = (App_OLED_GetPage() == APP_OLED_PAGE_KEY_TEST) ? "K2 KEY" : "K2 CS";
    }
}

static void App_ButtonTest_HandleLongPress(uint8_t index)
{
    App_ButtonTest_ClearCounts((index == 1U) ? "K1 LONG CLR" : "K2 LONG CLR");
}

static void App_ButtonTest_ClearCounts(const char *event_text)
{
    s_k1.press_count = 0UL;
    s_k2.press_count = 0UL;
    s_run_enabled = 0U;
    s_last_event = event_text;
    App_OLED_SetPage(APP_OLED_PAGE_KEY_TEST);
}

static void App_ButtonTest_UpdateOLED(void)
{
    App_OLED_SetButtonDebug(s_k1.raw_level,
                            s_k2.raw_level,
                            s_k1.stable_down,
                            s_k2.stable_down,
                            s_k1.press_count,
                            s_k2.press_count,
                            s_k1.hold_ms,
                            s_k2.hold_ms,
                            s_run_enabled,
                            s_last_event);
}

static uint8_t App_ButtonTest_ReadRawLevel(const AppButton_State *button)
{
    return (HAL_GPIO_ReadPin(button->port, button->pin) == GPIO_PIN_SET) ? 1U : 0U;
}
