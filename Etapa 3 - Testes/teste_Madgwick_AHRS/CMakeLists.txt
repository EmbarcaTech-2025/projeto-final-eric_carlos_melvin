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
set(PICO_BOARD pico CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(ACELEROMETRO)

# Initialize the SDK
pico_sdk_init()

add_executable(ACELEROMETRO
    main.c
    mpu6050_i2c.c
    MadgwickAHRS.c
)

# Pull in our pico_stdlib which aggregates commonly used features
target_link_libraries(ACELEROMETRO pico_stdlib hardware_i2c)

# enable usb output, disable uart output
pico_enable_stdio_usb(ACELEROMETRO 1)
pico_enable_stdio_uart(ACELEROMETRO 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(ACELEROMETRO)
