cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME SourceCar_HAL)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    App/Src/app_button_test.c
    App/Src/app_cs1238_task.c
    App/Src/app_main.c
    App/Src/app_oled_ui.c
    App/Src/app_serial_bridge.c
    BSP/Src/bsp_system.c
    BSP/Src/bsp_clock.c
    BSP/Src/bsp_esp8266.c
    BSP/Src/bsp_gpio.c
    BSP/Src/bsp_led.c
    BSP/Src/bsp_uart1.c
    BSP/Src/bsp_uart2.c
    BSP/Src/bsp_uart3.c
    Common/Src/ring_buffer.c
    Device/Src/dev_cs1238.c
    Device/Src/dev_oled.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # User include paths
    ${CMAKE_SOURCE_DIR}/App/Inc
    ${CMAKE_SOURCE_DIR}/BSP/Inc
    ${CMAKE_SOURCE_DIR}/Common/Inc
    ${CMAKE_SOURCE_DIR}/Device/Inc

    # STM32CubeMX / HAL / CMSIS include paths
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

if(NOT CMAKE_OBJCOPY OR NOT EXISTS ${CMAKE_OBJCOPY})
    get_filename_component(TOOLCHAIN_BIN_DIR ${CMAKE_C_COMPILER} DIRECTORY)
    set(CMAKE_OBJCOPY
        "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-objcopy.exe"
        CACHE FILEPATH "Objcopy executable" FORCE
    )
    set(CMAKE_OBJCOPY "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-objcopy.exe")
endif()

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
            ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating HEX file"
)
