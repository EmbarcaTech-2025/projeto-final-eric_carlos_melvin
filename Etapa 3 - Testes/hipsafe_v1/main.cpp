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

#include "bogus.h"
#include "analise_postural.h"
#include "evento.h"
#include "estruturas_de_dados.hpp"  // exceto a estrutura de evento, que está em evento.h
#include <thread>
#include <iostream>

// DESCOMENTAR ESSAS LINHAS quando for implementando os recurosos e tirando do mock
// #include "buzzer.h"
// #include "mpu9350_i2c.h"
// #include "SDCard.h"
// #include "buzzer.h"

// Variáveis globais
Alarme alarme;  // Struct em analise_postural.h
std::vector<Evento> eventosAbertos;   // Classe em analise_postural.h
static int contador_prints = 0; // Contador para reduzir frequência dos prints (100Hz seria muito output)
bool mpu_flags[3] = {false, false, false}; // Flags para cada MPU6050

int main() {

    // ***************************************************************
    // INICIALIZAÇÃO DO SISTEMA
    
    stdio_init_all(); // Inicializa UART/USB para debug
    std::this_thread::sleep_for(std::chrono::seconds(1)); // Aguarda 1 segundos para estabilizar a conexão serial
    printf("=== HIPSAFE v1 - Sistema de Monitoramento Postural ===\n");
    printf("Iniciando sistema...\n");
    
    // Estrutura de dados para cada MPU6050
    mpu6050_t mpu_0 = {
        .i2c = i2c1, // I2C instance
        .sda_gpio = I2C1_SDA, // SDA GPIO pin
        .scl_gpio = I2C1_SCL, // SCL GPIO pin
        .addr = MPU6050_ADDR_0, // Endereço I2C do MPU6050
        .id = 0 // ID do MPU6050
    };

    mpu6050_t mpu_1 = {
        .i2c = i2c1,
        .sda_gpio = I2C1_SDA,
        .scl_gpio = I2C1_SCL,
        .addr = MPU6050_ADDR_1,
        .id = 1
    };

    mpu6050_t mpu_2 = {
        .i2c = i2c0,
        .sda_gpio = I2C0_SDA,
        .scl_gpio = I2C0_SCL,
        .addr = MPU6050_ADDR_1,
        .id = 2
    };

    mpu6050_t mpu_list[] = {mpu_0, mpu_1, mpu_2};

    // Inicializar I2C para os MPUs
    printf("Configurando cada sensor MPU6050...\n");
    for(auto& mpu : mpu_list) {
        mpu6050_setup_i2c(&mpu);
        mpu6050_reset(&mpu); // Reinicia o sensor

        // Configura o sensor para ±2g (mais sensível) e ±250°/s
        mpu6050_set_accel_range(&mpu, 0); // 0 = ±2g
        mpu6050_set_gyro_range(&mpu, 0);  // 0 = ±250°/s
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    printf("MPU6050s configurados: ±2g, ±250°/s\n");
    
    // Inicializar botões
    printf("Inicializando botões...\n");
    setup_buttons();
    
    // Inicializar buzzer
    printf("Inicializando buzzer...\n");
    buzzer_init();
    
    // Inicializar SD Card
    printf("Inicializando SD Card...\n");
    sd_card_init();
    
    printf("Sistema inicializado com sucesso!\n");
    printf("Configuração: Taxa de amostragem 100Hz (período = 10ms)\n");
    printf("Iniciando monitoramento postural...\n\n");

    // FIM DA INICIALIZAÇÃO DO SISTEMA
    // ***************************************************************

    // AQUI PODERIAM ENTRAR CÓDIGOS DE AJUSTES DE PARÂMETROS, WATCH DOG DENTRE OUTROS.

    // INÍCIO DO PROGRAMA PROPRIAMENTE DITO:
    while (true) {
        // REQUISITA POSIÇÕES
        Orientacao orientacao = getPosition(mpu_list);

        // CHECA E GRAVA POSIÇÃO PERIGOSA
        dangerCheck(orientacao);
    }

    std::cout << "Nunca alcança essa linha\n";
    return 0;
}
