# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 1.5.1)
set(toolchainVersion 13_2_Rel1)
set(picotoolVersion 2.0.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(hipsafe_v1 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add subdirectory for SD card library
# add_subdirectory(drivers/sdcard/lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)

# Add executable with C++ main file
add_executable(hipsafe_v1 
    analise_postural.cpp
    analise_postural_sensor.c
    drivers/button/button.c
    drivers/buzzer/buzzer.c
    # drivers/sdcard/SDCard.c
    # drivers/sdcard/hw_config.c
    drivers/mpu6050/mpu6050_i2c.c
    drivers/filter/MadgwickAHRS.c
    evento/evento.cpp
)

pico_set_program_name(hipsafe_v1 "hipsafe_v1")
pico_set_program_version(hipsafe_v1 "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(hipsafe_v1 0)
pico_enable_stdio_usb(hipsafe_v1 1)

# Add include directories
target_include_directories(hipsafe_v1 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/drivers
    ${CMAKE_CURRENT_LIST_DIR}/drivers/button
    ${CMAKE_CURRENT_LIST_DIR}/drivers/buzzer
    ${CMAKE_CURRENT_LIST_DIR}/drivers/sdcard
    ${CMAKE_CURRENT_LIST_DIR}/drivers/mpu6050
    ${CMAKE_CURRENT_LIST_DIR}/drivers/filter
    ${CMAKE_CURRENT_LIST_DIR}/evento
)

# Add the standard library and required libraries
target_link_libraries(hipsafe_v1
    pico_stdlib
    pico_multicore
    hardware_spi
    hardware_i2c
    hardware_pwm
    hardware_rtc
    hardware_dma
    hardware_timer
    # FatFs_SPI
)

pico_add_extra_outputs(hipsafe_v1)