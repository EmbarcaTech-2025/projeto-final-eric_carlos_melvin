#include "sensor_watchdog.h"
#include "hardware/watchdog.h"
#include "hardware/gpio.h" // Adicionado para controle de GPIO
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

// Instância global do watchdog
static sensor_watchdog_t g_watchdog;

/**
 * @brief Inicializa o sistema de watchdog dos sensores
 */
void sensor_watchdog_init(void) 
{
    // Limpa a estrutura do watchdog
    memset(&g_watchdog, 0, sizeof(sensor_watchdog_t));
    
    // Inicializa os dados dos sensores
    for (int i = 0; i < MAX_SENSORS; i++) 
    {
        g_watchdog.sensors[i].sensor_id = i;
        g_watchdog.sensors[i].is_initialized = false;
        g_watchdog.sensors[i].is_frozen = false;
        g_watchdog.sensors[i].sample_count = 0;
        g_watchdog.sensors[i].history_index = 0;
    }
    
    // Verifica se o sistema foi reiniciado pelo watchdog
    if (watchdog_caused_reboot()) 
    {
        printf("*** ATENCAO: Sistema foi reiniciado pelo watchdog! ***\n");
        printf("*** Motivo: Travamento de sensor detectado ***\n");
        printf("*** Sistema reinicializado com sucesso ***\n");
        sleep_ms(1000); // Pausa para destacar a mensagem
    }
    
    // NÃO habilita o watchdog ainda - apenas inicializa a estrutura
    g_watchdog.watchdog_enabled = false;
    g_watchdog.last_update_time = to_ms_since_boot(get_absolute_time());
    
    printf("Watchdog dos sensores inicializado (timeout: %d ms).\n", WATCHDOG_TIMEOUT_MS);
    printf("Watchdog hardware sera habilitado apos estabilizacao.\n");
}

/**
 * @brief Habilita o monitoramento do watchdog
 */
void sensor_watchdog_enable(void) 
{
    // Agora sim habilita o watchdog hardware
    watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);
    
    g_watchdog.watchdog_enabled = true;
    g_watchdog.last_update_time = to_ms_since_boot(get_absolute_time());
    printf("*** WATCHDOG DOS SENSORES HABILITADO ***\n");
    printf("Watchdog hardware ativado com timeout de %d ms\n", WATCHDOG_TIMEOUT_MS);
    printf("Monitorando %d sensores para travamentos...\n", MAX_SENSORS);
    printf("Limiar de detecção: %d amostras idênticas\n", SENSOR_FREEZE_THRESHOLD);
}

/**
 * @brief Desabilita o monitoramento do watchdog
 */
void sensor_watchdog_disable(void) 
{
    g_watchdog.watchdog_enabled = false;
    printf("Watchdog dos sensores desabilitado.\n");
}

/**
 * @brief Verifica se os dados do sensor mudaram significativamente
 * @param sensor Ponteiro para os dados do sensor
 * @param new_data Novos dados brutos do sensor
 * @return true se os dados mudaram, false caso contrário
 */
static bool sensor_data_changed(sensor_watchdog_data_t *sensor, mpu9250_raw_data_t *new_data) 
{
    if (sensor->sample_count == 0) {
        return true; // Primeira amostra sempre é considerada mudança
    }
    
    // Obtém o índice da última amostra
    uint32_t last_index = (sensor->history_index > 0) ? 
                         (sensor->history_index - 1) : 
                         (SENSOR_FREEZE_THRESHOLD - 1);
    
    // Verifica se há mudança significativa em qualquer eixo
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
 * @brief Verifica se o sensor está travado baseado no histórico
 * @param sensor Ponteiro para os dados do sensor
 * @return true se o sensor está travado, false caso contrário
 */
static bool check_sensor_freeze(sensor_watchdog_data_t *sensor) 
{
    if (sensor->sample_count < SENSOR_FREEZE_THRESHOLD) 
    {
        return false; // Ainda não temos amostras suficientes
    }
    
    // Verifica se todas as últimas SENSOR_FREEZE_THRESHOLD amostras são idênticas
    for (uint32_t i = 1; i < SENSOR_FREEZE_THRESHOLD; i++) 
    {
        if (sensor->accel_x_history[i] != sensor->accel_x_history[0] ||
            sensor->accel_y_history[i] != sensor->accel_y_history[0] ||
            sensor->accel_z_history[i] != sensor->accel_z_history[0] ||
            sensor->gyro_x_history[i] != sensor->gyro_x_history[0] ||
            sensor->gyro_y_history[i] != sensor->gyro_y_history[0] ||
            sensor->gyro_z_history[i] != sensor->gyro_z_history[0]) {
            return false; // Encontrou diferença, não está travado
        }
    }
    
    return true; // Todas as amostras são idênticas
}

/**
 * @brief Alimenta o watchdog com dados do sensor
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
    
    // Atualiza os contadores
    sensor->history_index = (sensor->history_index + 1) % SENSOR_FREEZE_THRESHOLD;
    sensor->sample_count++;
    
    // Verifica se o sensor está travado
    sensor->is_frozen = check_sensor_freeze(sensor);
    
    if (!sensor->is_initialized) {
        sensor->is_initialized = true;
    }
    
    // Atualiza o tempo da última atualização
    g_watchdog.last_update_time = to_ms_since_boot(get_absolute_time());
}

/**
 * @brief Verifica se o sensor está realmente travado ou apenas desconectado
 * @param sensor_id ID do sensor
 * @return true se confirmadamente travado, false se pode ser desconexão
 */
static bool verify_sensor_freeze(uint8_t sensor_id) 
{
    // Aqui você pode adicionar uma verificação de conectividade
    // Por exemplo, tentar ler o registrador WHO_AM_I
    // Se a leitura falhar, pode ser desconexão, não travamento
    
    // Por enquanto, assume que está travado se chegou até aqui
    return true;
}

/**
 * @brief Atualiza o estado do watchdog e verifica por travamentos
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
        if (g_watchdog.sensors[i].is_initialized && g_watchdog.sensors[i].is_frozen) {
            // Verifica se é realmente travamento ou desconexão
            if (verify_sensor_freeze(i)) {
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
        for (int i = 0; i < MAX_SENSORS; i++) {
            if (g_watchdog.sensors[i].is_initialized && g_watchdog.sensors[i].is_frozen) {
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
 * @brief Verifica se um sensor específico está travado
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
 * @brief Verifica se algum sensor está travado
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
 * @brief Força reset do barramento I2C antes do reset do sistema
 * Isso ajuda a destravar sensores que possam estar travados no barramento
 */
static void force_i2c_bus_reset(void) 
{
    printf("Forçando reset do barramento I2C...\n");
    
    // GPIOs usados para I2C1 (baseado no main.c: SDA=GPIO2, SCL=GPIO3)
    const uint sda_gpio = 2;
    const uint scl_gpio = 3;
    
    // Salva as funções atuais dos pinos
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
    for (int i = 0; i < 9; i++) {
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
 * @brief Força o reset do sistema via watchdog
 */
void sensor_watchdog_reset_system(void) 
{
    printf("SISTEMA TRAVADO DETECTADO - Forçando reset via watchdog...\n");
    printf("Aguarde o reinicio do sistema...\n");
    
    // NOVO: Força reset do barramento I2C antes do reset do sistema
    force_i2c_bus_reset();
    
    printf("Reset em 3 segundos...\n");
    
    // Aguarda um tempo maior para garantir que as mensagens sejam enviadas
    for (int i = 3; i > 0; i--) {
        printf("Reset em %d segundos...\n", i);
        sleep_ms(1000);
    }
    
    printf("RESETANDO AGORA!\n");
    sleep_ms(500); // Tempo adicional para enviar a última mensagem
    
    // Para de alimentar o watchdog, forçando o reset
    while (1) 
    {
        // Loop infinito sem alimentar o watchdog
        sleep_ms(100);
    }
}

/**
 * @brief Imprime o status atual do watchdog
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
