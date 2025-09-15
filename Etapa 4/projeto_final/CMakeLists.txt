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

project(projeto_final C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add subdirectory for SD card library
add_subdirectory(drivers/sdcard/lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)

# Add executable. Default name is the project name, version 0.1

add_executable(projeto_final 
    main.cpp
    src/analise_postural.cpp
    src/evento.cpp
    drivers/button/button.c
    drivers/buzzer/buzzer.c
    drivers/mpu9250/mpu9250_i2c.c
    drivers/madgwick/MadgwickAHRS.c
    drivers/postura/algoritmo_postura.c
    drivers/sdcard/SDCard.c
    drivers/sdcard/hw_config.c
    drivers/rtc/ds3231.c
    drivers/rtc/rtc_utils.c
    drivers/watchdog/sensor_watchdog.c
)

pico_set_program_name(projeto_final "projeto_final")
pico_set_program_version(projeto_final "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(projeto_final 0)
pico_enable_stdio_usb(projeto_final 1)

# Add the standard library to the build
target_link_libraries(projeto_final
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(projeto_final PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${CMAKE_CURRENT_LIST_DIR}/drivers/button
        ${CMAKE_CURRENT_LIST_DIR}/drivers/buzzer
        ${CMAKE_CURRENT_LIST_DIR}/drivers/mpu9250
        ${CMAKE_CURRENT_LIST_DIR}/drivers/madgwick
        ${CMAKE_CURRENT_LIST_DIR}/drivers/postura
        ${CMAKE_CURRENT_LIST_DIR}/drivers/sdcard
        ${CMAKE_CURRENT_LIST_DIR}/drivers/rtc
        ${CMAKE_CURRENT_LIST_DIR}/drivers/watchdog
        ${CMAKE_CURRENT_LIST_DIR}/evento
)

# Add any user requested libraries
target_link_libraries(projeto_final 
        hardware_spi
        hardware_dma
        hardware_i2c
        hardware_gpio
        hardware_pwm
        hardware_timer
        hardware_rtc
        hardware_watchdog
        pico_multicore
        pico_sync
        pico_time
        pico_util
        FatFs_SPI
)

pico_add_extra_outputs(projeto_final)

