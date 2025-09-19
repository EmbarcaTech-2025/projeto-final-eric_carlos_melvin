# Arquivo de configuração do projeto CMake para Raspberry Pi Pico
# Todos os comentários foram traduzidos e detalhados em português.

cmake_minimum_required(VERSION 3.13)

# Define o padrão das linguagens C e C++
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Gera compile_commands.json para análise de ferramentas

# =================================================================================================
# Inicialização do pico_sdk a partir do local instalado
# (pode ser definido por variável de ambiente, cache do CMake, etc)
#
# == NÃO ALTERE AS LINHAS ABAIXO para garantir o funcionamento da extensão VS Code do Raspberry Pi Pico ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE}) # Diretório home no Windows
else()
    set(USERHOME $ENV{HOME})        # Diretório home em outros sistemas
endif()
set(sdkVersion 1.5.1)              # Versão do SDK Pico
set(toolchainVersion 13_2_Rel1)    # Versão do toolchain
set(picotoolVersion 2.0.0)         # Versão do picotool
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake) # Caminho do arquivo de integração VS Code
if (EXISTS ${picoVscode})
    include(${picoVscode})          # Inclui integração com VS Code, se existir
endif()
# =================================================================================================

# Define a placa utilizada (pico_w por padrão)
set(PICO_BOARD pico_w CACHE STRING "Tipo de placa")

# Importa o SDK do Raspberry Pi Pico (deve ser antes do comando project)
include(pico_sdk_import.cmake)

# Define o projeto e as linguagens utilizadas
project(projeto_final C CXX ASM)

# Inicializa o SDK do Raspberry Pi Pico
pico_sdk_init()

# Adiciona subdiretório da biblioteca de cartão SD (FatFs_SPI)
add_subdirectory(drivers/sdcard/lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI)

# Cria o executável principal do projeto, incluindo todos os arquivos fonte necessários
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

# Define o nome e a versão do programa
pico_set_program_name(projeto_final "projeto_final")
pico_set_program_version(projeto_final "0.1")

# Habilita/desabilita saída padrão via UART e USB
# (0 = desabilitado, 1 = habilitado)
pico_enable_stdio_uart(projeto_final 0)
pico_enable_stdio_usb(projeto_final 1)

# Adiciona a biblioteca padrão do Pico ao projeto
target_link_libraries(projeto_final pico_stdlib)

# Adiciona diretórios de inclusão (headers) ao projeto
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
)

# Adiciona bibliotecas extras necessárias ao projeto
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

# Gera arquivos binários, UF2, etc, para upload na placa
pico_add_extra_outputs(projeto_final)

