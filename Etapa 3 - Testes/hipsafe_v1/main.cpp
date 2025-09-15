/*
    Arquivo principal. 
    Este arquivo:
        Inicia todos os periféricos do sistema
        Inicia as interrupções dos botões (tem que implementar o callback)
        Entra no loop principal do programa que consiste em:
            while(1) {
                pega a posição;
                verifica posição;
            }
    Em 'pega a posição' ( = "getPosition();" ), internamente, ele lê os valores brutos 
    dos MPU9350 e já transforma em nos valores angulares de rotação, abdução e flexão.
    Em 'verifica posição' já está incluido:
        (1) Verificar se as posições são perigosas
        (2) Gerenciar os eventos
        (3) Ligar e desligar o alarme
        (4) Gravar a informação dos eventos concluidos no SDcard.

*/

#include "analise_postural.h"
#include "evento.h"
#include "estruturas_de_dados.hpp"  // exceto a estrutura de evento, que está em evento.h
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "button.h"
#include "mpu9250_i2c.h"
#include "SDCard.h"
#include "buzzer.h"
#include "rtc_utils.h"  // Para usar as funções do RTC DS3231

 // I2C addresses for MPU6050
#define MPU6050_ADDR_0 0x68 // Default I2C address
#define MPU6050_ADDR_1 0x69 // Alternate I2C address
// GPIO pins for interfacing I2C
#define I2C0_SDA 0 // Default SDA pin
#define I2C0_SCL 1 // Default SCL pin
#define I2C1_SDA 2 // Alternate SDA pin
#define I2C1_SCL 3 // Alternate SCL pin

// Variáveis globais
Alarme alarme;  // Struct em analise_postural.h
std::vector<Evento> eventosAbertos;   // Classe em analise_postural.h
static int contador_prints = 0; // Contador para reduzir frequência dos prints (100Hz seria muito output)
bool mpu_flags[3] = {false, false, false}; // Flags para cada MPU9250

int main() 
{

    // ***************************************************************
    // INICIALIZAÇÃO DO SISTEMA
    
    stdio_init_all(); // Inicializa UART/USB para debug
    sleep_ms(1000); // Aguarda 1 segundo para estabilizar a conexão serial
    printf("=== HIPSAFE v1 - Sistema de Monitoramento Postural ===\n");
    printf("Iniciando sistema...\n");
    
    // Estrutura de dados para cada MPU9250
    mpu9250_t mpu_0 = {
        .i2c = i2c1, // I2C instance
        .sda_gpio = I2C1_SDA, // SDA GPIO pin
        .scl_gpio = I2C1_SCL, // SCL GPIO pin
        .addr = MPU6050_ADDR_0, // Endereço I2C do MPU9250
        .id = 0 // ID do MPU9250
    };

    mpu9250_t mpu_1 = {
        .i2c = i2c1,
        .sda_gpio = I2C1_SDA,
        .scl_gpio = I2C1_SCL,
        .addr = MPU6050_ADDR_1,
        .id = 1
    };

    // Configuração padrão para o MPU9250
    mpu9250_config_t config = {
        .accel_range = MPU9250_ACCEL_RANGE_2G,      // ±2g
        .gyro_range = MPU9250_GYRO_RANGE_250DPS,    // ±250°/s
        .dlpf_filter = MPU9250_DLPF_41HZ,           // 41Hz low-pass filter
        .sample_rate_divider = 9,                    // 100Hz sample rate (1000/(1+9))
        .enable_magnetometer = true                  // Enable magnetometer
    };

    mpu9250_t mpu_list[] = {mpu_0, mpu_1};
    
    // Inicializar botões
    printf("Inicializando botões...\n");
    setup_buttons();
    
    // Inicializar buzzer
    printf("Inicializando buzzer...\n");
    buzzer_init();

    printf("Inicializando RTC DS3231...\n");
    rtc_ds3231_init();  // Usa a função de rtc_utils.c que configura automaticamente
    
    // Inicializar SD Card
    printf("Inicializando SD Card...\n");
    sd_card_init();

    // Inicializar I2C para os MPUs
    printf("Configurando cada sensor MPU9250...\n");
    for(auto& mpu : mpu_list) 
    {
        mpu9250_init(&mpu, &config);
        sleep_ms(1000);
    }
    printf("MPU9250s configurados: ±2g, ±250°/s\n");
    
    printf("Sistema inicializado com sucesso!\n");
    printf("Configuração: Taxa de amostragem 100Hz (período = 10ms)\n");
    printf("Iniciando monitoramento postural...\n\n");

    // FIM DA INICIALIZAÇÃO DO SISTEMA
    // ***************************************************************

    // AQUI PODERIAM ENTRAR CÓDIGOS DE AJUSTES DE PARÂMETROS, WATCH DOG DENTRE OUTROS.

    // INÍCIO DO PROGRAMA PROPRIAMENTE DITO:
    while (true) 
    {
        // REQUISITA POSIÇÕES
        Orientacao orientacao = getPosition(mpu_list);

        // CHECA E GRAVA POSIÇÃO PERIGOSA
        dangerCheck(orientacao);
    }

    std::cout << "Nunca alcança essa linha\n";
    return 0;
}
