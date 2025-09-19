
/*
 * Projeto: Sistema de Monitoramento Ativo para Reabilitação de Quadril
 * Autores: Eric Senne Roma, Carlos Fernando Mattos do Amaral e Melvin Gustavo Maradiaga Elvir
 * Data de Criação: 19/09/2025
 *
 * Descrição Geral:
 *   Código principal do sistema embarcado para monitoramento postural utilizando a placa BitDogLab (Raspberry Pi Pico W).
 *   O sistema lê sensores inerciais (MPU9250) para monitorar, em tempo real, os ângulos do quadril do paciente.
 *   Principais funcionalidades:
 *     - Detecção automática de posturas perigosas
 *     - Gerenciamento e registro de eventos em cartão SD
 *     - Acionamento de alarme sonoro (buzzer)
 *     - Interface com botões físicos para interação do usuário
 *     - Marcação temporal de eventos via RTC
 *     - Sistema watchdog para robustez e segurança
 *   Objetivo: Auxiliar o processo de reabilitação, fornecendo feedback imediato e histórico de dados para profissionais de saúde.
 *
 * Lógica do Programa:
 *   1. Inicializa todos os periféricos do sistema (sensores, botões, buzzer, RTC, SD Card, watchdog).
 *   2. Entra no loop principal, onde:
 *      - Lê a orientação dos sensores inerciais (getPosition)
 *      - Verifica se a posição é perigosa (dangerCheck)
 *      - Gerencia eventos e alarme
 *      - Atualiza o watchdog
 */


// ====== INCLUDES DE BIBLIOTECAS E COMPONENTES ======

#include "analise_postural.h"      // Funções e estruturas para análise postural e controle do alarme
#include "evento.h"                // Definição e manipulação de eventos do sistema
#include "estruturas_de_dados.hpp" // Estruturas de dados auxiliares (ex: Orientacao, Evento)
#include <iostream>                // Biblioteca padrão C++ para entrada/saída (usada para debug)
#include "pico/stdlib.h"           // Funções utilitárias da Raspberry Pi Pico (delay, inicialização, etc)
#include "hardware/i2c.h"          // Controle do barramento I2C (comunicação com sensores)
#include "hardware/gpio.h"         // Controle dos pinos GPIO (botões, buzzer, etc)

// Includes de componentes escritos em C (necessário extern "C" para linkage correto)
extern "C" {
    #include "button.h"            // Controle e leitura dos botões físicos
    #include "mpu9250_i2c.h"       // Driver para comunicação com sensores MPU9250 via I2C
    #include "SDCard.h"            // Driver para acesso e gravação no cartão SD
    #include "buzzer.h"            // Driver para controle do buzzer (alarme sonoro)
    #include "rtc_utils.h"         // Driver para o RTC DS3231 (relógio de tempo real)
    #include "sensor_watchdog.h"   // Driver do sistema watchdog (monitoramento de travamentos)
}


// ================== DEFINIÇÕES E VARIÁVEIS GLOBAIS ==================

// Variável global para flag do botão A (definida em button.c)
extern volatile bool button_a_pressed;

// Endereços I2C dos sensores MPU6050/MPU9250
#define MPU6050_ADDR_0 0x68 // Endereço padrão do MPU9250
#define MPU6050_ADDR_1 0x69 // Endereço alternativo do MPU9250 (AD0 conectado ao VCC)

// Pinos GPIO para I2C
#define I2C0_SDA 0 // SDA da I2C0 (RTC)
#define I2C0_SCL 1 // SCL da I2C0 (RTC)
#define I2C1_SDA 2 // SDA da I2C1 (MPU9250)
#define I2C1_SCL 3 // SCL da I2C1 (MPU9250)

// Estruturas e variáveis globais do sistema
Alarme alarme;                        // Estrutura de controle do alarme
std::vector<Evento> eventosAbertos;   // Lista de eventos abertos
static int contador_prints = 0;       // Contador para limitar prints no loop principal
bool mpu_flags[3] = {false, false, false}; // Flags de status para cada MPU9250


int main() 
{
    // ================== INICIALIZAÇÃO DO SISTEMA ==================

    stdio_init_all(); // Inicializa UART/USB para debug
    sleep_ms(1000);   // Aguarda estabilização da conexão serial
    printf("=== HIPSAFE v1 - Sistema de Monitoramento Postural ===\n");
    printf("Iniciando sistema...\n");

    // --- Configuração dos sensores MPU9250 ---
    // Cada estrutura representa um sensor inercial conectado ao sistema

    //MPU_0 - Sensor no tronco (pelve) - Referência
    mpu9250_t mpu_0 = {
        .i2c = i2c1,                // I2C1
        .sda_gpio = I2C1_SDA,       // Pino SDA da I2C1
        .scl_gpio = I2C1_SCL,       // Pino SCL da I2C1
        .addr = MPU6050_ADDR_0,     // Endereço I2C 0X68 do sensor
        .id = 0                     // Sensor com ID = 0
    };
    mpu9250_t mpu_1 = {
        .i2c = i2c1,                // I2C1
        .sda_gpio = I2C1_SDA,       // Pino SDA da I2C1
        .scl_gpio = I2C1_SCL,       // Pino SCL da I2C1
        .addr = MPU6050_ADDR_1,     // Endereço I2C 0X69 do sensor
        .id = 1                     // Sensor com ID = 1
    };

    // Parâmetros de configuração padrão para os sensores MPU9250
    mpu9250_config_t config = {
        .accel_range = MPU9250_ACCEL_RANGE_2G,      // Faixa do acelerômetro: ±2g
        .gyro_range = MPU9250_GYRO_RANGE_250DPS,    // Faixa do giroscópio: ±250°/s
        .dlpf_filter = MPU9250_DLPF_41HZ,           // Filtro passa-baixa: 41Hz
        .sample_rate_divider = 9,                   // Taxa de amostragem: 100Hz (1000/(1+9))
        .enable_magnetometer = true                 // Habilita magnetômetro
    };

    mpu9250_t mpu_list[] = {mpu_0, mpu_1}; // Lista de sensores conectados

    // --- Inicialização dos periféricos ---
    printf("Inicializando botões...\n");
    setup_buttons(); // Configura interrupções dos botões

    printf("Inicializando buzzer...\n");
    buzzer_init(); // Inicializa o buzzer para alarmes sonoros

    printf("Inicializando RTC DS3231...\n");
    rtc_ds3231_init(); // Inicializa o relógio de tempo real

    printf("Inicializando SD Card...\n");
    sd_card_init(); // Inicializa o cartão SD para registro de eventos

    // --- Inicialização dos sensores inerciais ---
    printf("Configurando cada sensor MPU9250...\n");
    for(auto& mpu : mpu_list) 
    {
        mpu9250_init(&mpu, &config); // Inicializa cada sensor com a configuração padrão
        sleep_ms(1000);              // Aguarda estabilização
    }
    printf("MPU9250s configurados: ±2g, ±250°/s\n");

    printf("Sistema inicializado com sucesso!\n");
    printf("Configuração: Taxa de amostragem 100Hz (período = 10ms)\n");
    printf("Iniciando monitoramento postural...\n\n");

    // ================== WATCHDOG ==================

    printf("\n=== CONFIGURAÇÃO DO SISTEMA DE WATCHDOG ===\n");
    printf("Aguardando estabilização dos sensores antes de ativar watchdog...\n");

    // Aguarda 5 segundos para garantir estabilização dos sensores
    for (int i = 5; i > 0; i--) 
    {
        printf("Aguardando %d segundos...\n", i);
        sleep_ms(1000);
    }

    printf("Inicializando sistema de watchdog...\n");
    sensor_watchdog_init(); // Inicializa o watchdog para monitorar travamentos

    // Aguarda mais 3 segundos antes de ativar o watchdog
    printf("Aguardando mais 3 segundos antes de ativar watchdog...\n");
    for (int i = 3; i > 0; i--) 
    {
        printf("Ativando watchdog em %d segundos...\n", i);
        sleep_ms(1000);
    }

    sensor_watchdog_enable(); // Ativa o watchdog
    printf("=== WATCHDOG ATIVADO - Sistema monitorado ===\n\n");

    // ================== LOOP PRINCIPAL ==================

    while (true) 
    {
        // --- Gerenciamento do botão A ---
        // Permite ao usuário silenciar/desilenciar o alarme via botão físico
        if (button_a_pressed) 
        {
            printf("Botão A pressionado\n");

            if (alarme_esta_ligado()) 
            {
                if (alarme_esta_silenciado()) 
                {
                    printf("  -> Desilenciando alarme\n");
                    desilenciar_alarme();
                } 
                else 
                {
                    printf("  -> Silenciando alarme\n");
                    silenciar_alarme();
                }
            } 
            else 
            {
                printf("  -> Alarme não está ativo no momento\n");
            }

            button_a_pressed = false; // Reseta a flag do botão
        }

        // --- Aquisição da orientação postural ---
        // Lê os sensores e retorna os ângulos de rotação, abdução e flexão
        Orientacao orientacao = getPosition(mpu_list);

        // --- Verificação de postura perigosa ---
        // Analisa a orientação e executa ações: gera eventos, ativa/desativa alarme, grava no SD
        dangerCheck(orientacao);

        // --- Atualização do watchdog ---
        // Garante que o sistema não travou; reinicia o temporizador do watchdog
        sensor_watchdog_update();
    }

    // Esta linha nunca será alcançada devido ao loop infinito
    std::cout << "Nunca alcança essa linha\n";
    return 0;
}
