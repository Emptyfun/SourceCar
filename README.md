# SourceCar_HAL

STM32F103-based SourceCar firmware built with STM32 HAL, CMake, and an
application/BSP/device-driver split. The repository is intended to keep the
generated STM32Cube project code and the hand-written firmware logic in one
place while making the current bring-up target easy to understand.

## Current Focus

The default firmware configuration is a CS1238 ADC bring-up build:

- USART1 stays enabled as the debug console.
- The LED heartbeat stays enabled so the main loop activity is visible.
- CS1238 sampling is enabled.
- The older 3-UART car-control router, ESP8266 path, USART2, and USART3 are
  compiled in where needed but not started by default.

The active feature switches live in `App/Inc/app_config.h`:

```c
#define APP_ENABLE_CS1238          1
#define APP_ENABLE_LED_HEARTBEAT   1
#define APP_ENABLE_SERIAL_BRIDGE   0
#define APP_ENABLE_ESP8266         0
#define APP_ENABLE_UART2           0
#define APP_ENABLE_UART3           0
```

## Project Layout

```text
App/       Application-level tasks and feature orchestration
BSP/       Board support wrappers for clock, GPIO, UART, LED, and ESP8266
Common/    Shared utilities, currently a ring buffer implementation
Core/      STM32Cube/HAL startup code, handles, interrupts, and generated glue
Device/    Device drivers, currently including CS1238
Docs/      Notes that describe the current firmware behavior
Protocol/  Protocol-layer placeholders and motor/telemetry helpers
cmake/     Toolchain and STM32CubeMX CMake integration
```

`Core/Src/main.c` keeps the runtime flow small:

```c
BSP_System_Init();
App_Init();

while (1)
{
    App_Loop();
}
```

Most project behavior is selected and run from the `App/` layer.

## CS1238 ADC Bring-Up

The CS1238 driver is implemented in:

- `Device/Inc/dev_cs1238.h`
- `Device/Src/dev_cs1238.c`
- `App/Src/app_cs1238_task.c`

Current CS1238 pin mapping:

```text
ADC1 / hcs1238_1:
  SCLK = PB0
  DOUT = PB1

ADC2 / hcs1238_2:
  SCLK = PA0
  DOUT = PA6
```

Current app-level sampling settings:

```text
Speed:       10 Hz
PGA:         2
Reference:   REFO off
Channels:    A and B
Print rate:  1000 ms
Sample view: recent samples from the last 1200 ms
```

The app alternates CS1238 channel A/B on the active ADC device. After every
channel switch it discards a few conversions before accepting a sample, which
reduces stale or settling data after register changes. Debug output is sent on
USART1 and currently prints compact raw averages such as:

```text
[CS1238] t=12345ms
ADC1-A raw_avg=...
ADC1-B raw_avg=...
```

ADC2 is initialized by the current constants, but polling and printing for ADC2
are disabled in `app_cs1238_task.c` so ADC1 timing can be verified first:

```c
#define APP_CS1238_POLL_ADC2  0
#define APP_CS1238_PRINT_ADC2 0
```

## Optional UART Router

The project still contains the earlier 3-UART router in
`App/Src/app_serial_bridge.c`. It supports:

- PC to car-controller routing over USART1 and USART2
- ESP8266 network-side car control over USART3
- PC to ESP8266 AT-command debug mode
- Local `#` commands such as `#pc_car`, `#wifi_car`, `#esp_debug`, `#esp_on`,
  `#esp_off`, and `#status`

That router is intentionally disabled in the current CS1238 bring-up build with
`APP_ENABLE_SERIAL_BRIDGE 0`. Re-enable the related switches in
`App/Inc/app_config.h` when returning to car-control or ESP8266 routing tests.

## Build

Prerequisites:

- CMake 3.22 or newer
- Ninja
- `arm-none-eabi-gcc` toolchain available to CMake

Configure and build:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

The generated ELF and HEX files are placed under `build/Debug/`.

## Notes For Contributors

- Keep generated STM32Cube/HAL code separate from hand-written application
  logic where possible.
- Prefer adding feature gates in `App/Inc/app_config.h` when a bring-up target
  needs a smaller runtime surface.
- `HAL_UART_RxCpltCallback()` is owned by the serial bridge module. Avoid adding
  a second callback implementation when the router is enabled.
- The repository currently has no explicit license file. Choose and add a
  license before relying on public reuse terms.
