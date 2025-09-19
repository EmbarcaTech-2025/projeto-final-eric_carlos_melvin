
// ======================================================================
//  Arquivo: sensor_watchdog.c
//  Descrição: Implementação do sistema de watchdog para monitoramento de sensores inerciais
// ======================================================================

#include "sensor_watchdog.h"      // Header do watchdog de sensores
#include "hardware/watchdog.h"    // API do watchdog do hardware Pico
#include "hardware/gpio.h"        // Controle de GPIO para reset de barramento
#include <stdio.h>                 // Funções de entrada/saída padrão
#include <string.h>                // Manipulação de memória
#include <math.h>                  // Funções matemáticas
#include <stdlib.h>                // Funções utilitárias

// ----------------------------------------------------------------------
// Instância global do watchdog dos sensores
// ----------------------------------------------------------------------
static sensor_watchdog_t g_watchdog; ///< Estrutura global de controle do watchdog

/**
 * @brief Inicializa o sistema de watchdog dos sensores inerciais.
 *
 * Limpa a estrutura global, inicializa os dados de cada sensor e verifica se o sistema
 * foi reiniciado por travamento anterior. O watchdog de hardware só é habilitado após estabilização.
 */
void sensor_watchdog_init(void)
{
    // Zera toda a estrutura de controle do watchdog
    memset(&g_watchdog, 0, sizeof(sensor_watchdog_t));

    // Inicializa os campos de cada sensor monitorado
    for (int i = 0; i < MAX_SENSORS; i++) 
    {
        g_watchdog.sensors[i].sensor_id = i;
        g_watchdog.sensors[i].is_initialized = false;
        g_watchdog.sensors[i].is_frozen = false;
        g_watchdog.sensors[i].sample_count = 0;
        g_watchdog.sensors[i].history_index = 0;
    }

    // Detecta se o último reset foi causado pelo watchdog (travamento)
    if (watchdog_caused_reboot()) 
    {
        printf("*** ATENÇÃO: Sistema foi reiniciado pelo watchdog! ***\n");
        printf("*** Motivo: Travamento de sensor detectado ***\n");
        printf("*** Sistema reinicializado com sucesso ***\n");
        sleep_ms(1000); // Pausa para destacar a mensagem
    }

    // Apenas inicializa a estrutura, não habilita o watchdog ainda
    g_watchdog.watchdog_enabled = false;
    g_watchdog.last_update_time = to_ms_since_boot(get_absolute_time());

    printf("Watchdog dos sensores inicializado (timeout: %d ms).\n", WATCHDOG_TIMEOUT_MS);
    printf("Watchdog hardware será habilitado após estabilização dos sensores.\n");
}

/**
 * @brief Habilita o monitoramento do watchdog de sensores e ativa o watchdog de hardware.
 *
 * Após chamada, o sistema passa a monitorar travamentos e pode reiniciar automaticamente.
 */
void sensor_watchdog_enable(void)
{
    // Habilita o watchdog de hardware do Pico
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);

    g_watchdog.watchdog_enabled = true;
    g_watchdog.last_update_time = to_ms_since_boot(get_absolute_time());
    printf("*** WATCHDOG DOS SENSORES HABILITADO ***\n");
    printf("Watchdog hardware ativado com timeout de %d ms\n", WATCHDOG_TIMEOUT_MS);
    printf("Monitorando %d sensores para travamentos...\n", MAX_SENSORS);
    printf("Limiar de detecção: %d amostras idênticas\n", SENSOR_FREEZE_THRESHOLD);
}

/**
 * @brief Desabilita o monitoramento do watchdog de sensores.
 *
 * O sistema deixa de monitorar travamentos e não reiniciará automaticamente.
 */
void sensor_watchdog_disable(void)
{
    g_watchdog.watchdog_enabled = false;
    printf("Watchdog dos sensores desabilitado.\n");
}

/**
 * @brief Verifica se os dados do sensor mudaram significativamente em relação à última amostra.
 *
 * @param sensor Ponteiro para os dados do sensor
 * @param new_data Novos dados brutos do sensor
 * @return true se os dados mudaram, false caso contrário
 */
static bool sensor_data_changed(sensor_watchdog_data_t *sensor, mpu9250_raw_data_t *new_data)
{
    if (sensor->sample_count == 0) 
    {
        return true; // Primeira amostra sempre é considerada mudança
    }

    // Obtém o índice da última amostra registrada
    uint32_t last_index = (sensor->history_index > 0) ?
                         (sensor->history_index - 1) :
                         (SENSOR_FREEZE_THRESHOLD - 1);

    // Calcula a diferença absoluta entre a nova amostra e a última
    int16_t diff_ax = abs(new_data->accel[0] - sensor->accel_x_history[last_index]);
    int16_t diff_ay = abs(new_data->accel[1] - sensor->accel_y_history[last_index]);
    int16_t diff_az = abs(new_data->accel[2] - sensor->accel_z_history[last_index]);
    int16_t diff_gx = abs(new_data->gyro[0] - sensor->gyro_x_history[last_index]);
    int16_t diff_gy = abs(new_data->gyro[1] - sensor->gyro_y_history[last_index]);
    int16_t diff_gz = abs(new_data->gyro[2] - sensor->gyro_z_history[last_index]);

    // Considera que houve mudança se qualquer diferença for maior que 1 LSB
    return (diff_ax > 1 || diff_ay > 1 || diff_az > 1 ||
            diff_gx > 1 || diff_gy > 1 || diff_gz > 1);
}

/**
 * @brief Verifica se o sensor está travado com base no histórico de amostras.
 *
 * Considera travado se as últimas SENSOR_FREEZE_THRESHOLD amostras forem idênticas.
 * @param sensor Ponteiro para os dados do sensor
 * @return true se o sensor está travado, false caso contrário
 */
static bool check_sensor_freeze(sensor_watchdog_data_t *sensor)
{
    if (sensor->sample_count < SENSOR_FREEZE_THRESHOLD) 
    {
        return false; // Ainda não há amostras suficientes para análise
    }

    // Compara todas as amostras do histórico circular
    for (uint32_t i = 1; i < SENSOR_FREEZE_THRESHOLD; i++) 
    {
        if (sensor->accel_x_history[i] != sensor->accel_x_history[0] ||
            sensor->accel_y_history[i] != sensor->accel_y_history[0] ||
            sensor->accel_z_history[i] != sensor->accel_z_history[0] ||
            sensor->gyro_x_history[i] != sensor->gyro_x_history[0] ||
            sensor->gyro_y_history[i] != sensor->gyro_y_history[0] ||
            sensor->gyro_z_history[i] != sensor->gyro_z_history[0]) 
        {
            return false; // Encontrou diferença, não está travado
        }
    }
    return true; // Todas as amostras são idênticas
}

/**
 * @brief Alimenta o watchdog com uma nova amostra de dados do sensor.
 *
 * Atualiza o histórico circular, verifica travamento e marca o sensor como inicializado.
 * @param sensor_id ID do sensor (0 ou 1)
 * @param raw_data Ponteiro para os dados brutos do sensor
 */
void sensor_watchdog_feed(uint8_t sensor_id, mpu9250_raw_data_t *raw_data)
{
    if (sensor_id >= MAX_SENSORS || !g_watchdog.watchdog_enabled) 
    {
        return;
    }

    sensor_watchdog_data_t *sensor = &g_watchdog.sensors[sensor_id];

    // Armazena os dados no histórico circular
    uint32_t idx = sensor->history_index;
    sensor->accel_x_history[idx] = raw_data->accel[0];
    sensor->accel_y_history[idx] = raw_data->accel[1];
    sensor->accel_z_history[idx] = raw_data->accel[2];
    sensor->gyro_x_history[idx] = raw_data->gyro[0];
    sensor->gyro_y_history[idx] = raw_data->gyro[1];
    sensor->gyro_z_history[idx] = raw_data->gyro[2];

    // Atualiza os contadores de histórico e amostras
    sensor->history_index = (sensor->history_index + 1) % SENSOR_FREEZE_THRESHOLD;
    sensor->sample_count++;

    // Verifica se o sensor está travado
    sensor->is_frozen = check_sensor_freeze(sensor);

    if (!sensor->is_initialized) 
    {
        sensor->is_initialized = true;
    }

    // Atualiza o tempo da última atualização global
    g_watchdog.last_update_time = to_ms_since_boot(get_absolute_time());
}

/**
 * @brief Verifica se o sensor está realmente travado ou apenas desconectado.
 *
 * (Ponto de extensão: pode-se implementar leitura de registrador WHO_AM_I para distinguir travamento de desconexão.)
 * @param sensor_id ID do sensor
 * @return true se confirmadamente travado, false se pode ser desconexão
 */
static bool verify_sensor_freeze(uint8_t sensor_id)
{
    // Ponto de extensão: implementar verificação real de conectividade se necessário
    return true; // Assume travamento para fins de segurança
}

/**
 * @brief Atualiza o estado do watchdog, verifica travamentos e executa reset se necessário.
 *
 * Deve ser chamada periodicamente no loop principal do sistema.
 */
void sensor_watchdog_update(void)
{
    if (!g_watchdog.watchdog_enabled) 
    {
        return;
    }

    uint32_t current_time = to_ms_since_boot(get_absolute_time());

    // Verifica se algum sensor está travado
    bool any_frozen = false;
    for (int i = 0; i < MAX_SENSORS; i++) {
        if (g_watchdog.sensors[i].is_initialized && g_watchdog.sensors[i].is_frozen) 
        {
            // Confirma travamento (pode ser expandido para checar desconexão)
            if (verify_sensor_freeze(i)) 
            {
                any_frozen = true;
                break;
            }
        }
    }

    if (any_frozen) 
    {
        printf("\n");
        printf("************************************************\n");
        printf("*** ALERTA: SENSOR(ES) TRAVADO(S) DETECTADO(S)! ***\n");
        printf("************************************************\n");

        // Mostra quais sensores estão travados
        for (int i = 0; i < MAX_SENSORS; i++) 
        {
            if (g_watchdog.sensors[i].is_initialized && g_watchdog.sensors[i].is_frozen) 
            {
                printf("*** SENSOR %d TRAVADO! ***\n", i);
            }
        }

        sensor_watchdog_print_status();

        printf("Iniciando procedimento de reset do sistema...\n");
        sleep_ms(2000); // Tempo para ver as mensagens
        sensor_watchdog_reset_system();
    } 
    else 
    {
        // Se não há travamento, alimenta o watchdog do hardware
        watchdog_update();
    }
}

/**
 * @brief Consulta se um sensor específico está travado.
 *
 * @param sensor_id ID do sensor (0 ou 1)
 * @return true se o sensor está travado, false caso contrário
 */
bool sensor_watchdog_is_sensor_frozen(uint8_t sensor_id)
{
    if (sensor_id >= MAX_SENSORS) 
    {
        return false;
    }
    return g_watchdog.sensors[sensor_id].is_frozen;
}

/**
 * @brief Consulta se algum sensor monitorado está travado.
 *
 * @return true se algum sensor está travado, false caso contrário
 */
bool sensor_watchdog_any_sensor_frozen(void)
{
    for (int i = 0; i < MAX_SENSORS; i++) 
    {
        if (g_watchdog.sensors[i].is_initialized && g_watchdog.sensors[i].is_frozen) 
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Força o reset do barramento I2C antes do reset do sistema.
 *
 * Ajuda a destravar sensores que possam estar travados no barramento I2C.
 */
static void force_i2c_bus_reset(void)
{
    printf("Forçando reset do barramento I2C...\n");

    // GPIOs usados para I2C1 (ajustar conforme hardware)
    const uint sda_gpio = 2;
    const uint scl_gpio = 3;

    // Salva as funções atuais dos pinos (pode ser expandido para restaurar depois)
    uint32_t sda_func = gpio_get_function(sda_gpio);
    uint32_t scl_func = gpio_get_function(scl_gpio);

    // Configura como GPIO de saída
    gpio_init(sda_gpio);
    gpio_init(scl_gpio);
    gpio_set_dir(sda_gpio, GPIO_OUT);
    gpio_set_dir(scl_gpio, GPIO_OUT);

    // Força ambas as linhas em nível baixo por um momento
    gpio_put(sda_gpio, 0);
    gpio_put(scl_gpio, 0);
    sleep_ms(10);

    // Depois força ambas em nível alto para destravar o barramento
    gpio_put(sda_gpio, 1);
    gpio_put(scl_gpio, 1);
    sleep_ms(50);

    // Gera alguns pulsos de clock para limpar o barramento
    for (int i = 0; i < 9; i++) 
    {
        gpio_put(scl_gpio, 0);
        sleep_us(10);
        gpio_put(scl_gpio, 1);
        sleep_us(10);
    }

    // Mantém ambas as linhas em nível alto
    gpio_put(sda_gpio, 1);
    gpio_put(scl_gpio, 1);
    sleep_ms(10);

    printf("Reset do barramento I2C concluído.\n");
}

/**
 * @brief Força o reset do sistema via watchdog, após reset do barramento I2C.
 *
 * Utilizado para recuperar o sistema em caso de travamento de sensores.
 */
void sensor_watchdog_reset_system(void)
{
    printf("SISTEMA TRAVADO DETECTADO - Forçando reset via watchdog...\n");
    printf("Aguarde o reinício do sistema...\n");

    // Força reset do barramento I2C antes do reset do sistema
    force_i2c_bus_reset();

    printf("Reset em 3 segundos...\n");

    // Aguarda tempo suficiente para garantir envio das mensagens
    for (int i = 3; i > 0; i--) {
        printf("Reset em %d segundos...\n", i);
        sleep_ms(1000);
    }

    printf("RESETANDO AGORA!\n");
    sleep_ms(500); // Tempo adicional para enviar a última mensagem

    // Para de alimentar o watchdog, forçando o reset do sistema
    while (1) 
    {
        sleep_ms(100);
    }
}

/**
 * @brief Imprime o status atual do watchdog e dos sensores monitorados.
 *
 * Exibe informações detalhadas para depuração e acompanhamento do sistema.
 */
void sensor_watchdog_print_status(void)
{
    printf("=== Status do Watchdog dos Sensores ===\n");
    printf("Watchdog habilitado: %s\n", g_watchdog.watchdog_enabled ? "SIM" : "NAO");

    for (int i = 0; i < MAX_SENSORS; i++) 
    {
        sensor_watchdog_data_t *sensor = &g_watchdog.sensors[i];

        printf("Sensor %d:\n", i);
        printf("  Inicializado: %s\n", sensor->is_initialized ? "SIM" : "NAO");
        printf("  Travado: %s\n", sensor->is_frozen ? "SIM" : "NAO");
        printf("  Amostras coletadas: %lu\n", sensor->sample_count);

        if (sensor->sample_count > 0) 
        {
            uint32_t last_idx = (sensor->history_index > 0) ?
                               (sensor->history_index - 1) :
                               (SENSOR_FREEZE_THRESHOLD - 1);

            printf("  Última amostra - Accel: [%d, %d, %d], Gyro: [%d, %d, %d], Mag: [%d, %d, %d]\n",
                   sensor->accel_x_history[last_idx],
                   sensor->accel_y_history[last_idx],
                   sensor->accel_z_history[last_idx],
                   sensor->gyro_x_history[last_idx],
                   sensor->gyro_y_history[last_idx],
                   sensor->gyro_z_history[last_idx],
                   sensor->mag_x_history[last_idx],
                   sensor->mag_y_history[last_idx],
                   sensor->mag_z_history[last_idx]);
        }
    }
    printf("=======================================\n");
}
