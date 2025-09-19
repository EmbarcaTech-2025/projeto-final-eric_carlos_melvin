// ======================================================================
//  Arquivo: sensor_watchdog.h
//  Descrição: Interface do sistema de watchdog para monitoramento de sensores inerciais
// ======================================================================

#ifndef SENSOR_WATCHDOG_H_
#define SENSOR_WATCHDOG_H_

#include <stdint.h>         // Tipos inteiros padrão
#include <stdbool.h>        // Tipo booleano padrão
#include "mpu9250_i2c.h"   // Definição dos tipos de dados do sensor MPU9250
#include "pico/time.h"     // Funções de tempo do Pico SDK

// ----------------------------------------------------------------------
// Definições de Constantes
// ----------------------------------------------------------------------

#define MAX_SENSORS 2                ///< Número máximo de sensores monitorados
#define SENSOR_FREEZE_THRESHOLD 10    ///< Número de amostras idênticas para detectar travamento
#define WATCHDOG_TIMEOUT_MS 3000      ///< Timeout do watchdog de hardware (ms)

// ----------------------------------------------------------------------
// Estrutura: sensor_watchdog_data_t
// ----------------------------------------------------------------------
/**
 * @brief Estrutura para armazenar o histórico e o estado de cada sensor monitorado.
 *
 * - sensor_id: identificador lógico do sensor
 * - is_initialized: indica se o sensor já recebeu dados
 * - is_frozen: indica se o sensor está travado
 * - sample_count: número total de amostras recebidas
 * - history_index: índice circular do histórico
 * - *_history: históricos circulares das últimas amostras de cada eixo
 */
typedef struct {
    uint8_t sensor_id;                                   ///< ID lógico do sensor
    bool is_initialized;                                 ///< true se já recebeu dados
    bool is_frozen;                                      ///< true se travado
    uint32_t sample_count;                               ///< Total de amostras recebidas
    uint32_t history_index;                              ///< Índice do histórico circular
    int16_t accel_x_history[SENSOR_FREEZE_THRESHOLD];    ///< Histórico do acelerômetro X
    int16_t accel_y_history[SENSOR_FREEZE_THRESHOLD];    ///< Histórico do acelerômetro Y
    int16_t accel_z_history[SENSOR_FREEZE_THRESHOLD];    ///< Histórico do acelerômetro Z
    int16_t gyro_x_history[SENSOR_FREEZE_THRESHOLD];     ///< Histórico do giroscópio X
    int16_t gyro_y_history[SENSOR_FREEZE_THRESHOLD];     ///< Histórico do giroscópio Y
    int16_t gyro_z_history[SENSOR_FREEZE_THRESHOLD];     ///< Histórico do giroscópio Z
    int16_t mag_x_history[SENSOR_FREEZE_THRESHOLD];      ///< Histórico do magnetômetro X
    int16_t mag_y_history[SENSOR_FREEZE_THRESHOLD];      ///< Histórico do magnetômetro Y
    int16_t mag_z_history[SENSOR_FREEZE_THRESHOLD];      ///< Histórico do magnetômetro Z
} sensor_watchdog_data_t;

// ----------------------------------------------------------------------
// Estrutura: sensor_watchdog_t
// ----------------------------------------------------------------------
/**
 * @brief Estrutura global de controle do sistema de watchdog dos sensores.
 *
 * - sensors: array de estruturas de cada sensor monitorado
 * - watchdog_enabled: indica se o monitoramento está ativo
 * - last_update_time: timestamp da última atualização
 */
typedef struct {
    sensor_watchdog_data_t sensors[MAX_SENSORS]; ///< Estado de cada sensor
    bool watchdog_enabled;                        ///< true se monitoramento ativo
    uint32_t last_update_time;                    ///< Timestamp da última atualização
} sensor_watchdog_t;

// ----------------------------------------------------------------------
// Protótipos das Funções do Watchdog
// ----------------------------------------------------------------------

/** @brief Inicializa o sistema de watchdog dos sensores. */
void sensor_watchdog_init(void);

/** @brief Habilita o monitoramento do watchdog e ativa o watchdog de hardware. */
void sensor_watchdog_enable(void);

/** @brief Desabilita o monitoramento do watchdog. */
void sensor_watchdog_disable(void);

/**
 * @brief Alimenta o watchdog com uma nova amostra de dados do sensor.
 * @param sensor_id ID do sensor (0 ou 1)
 * @param raw_data Ponteiro para os dados brutos do sensor
 */
void sensor_watchdog_feed(uint8_t sensor_id, mpu9250_raw_data_t *raw_data);

/** @brief Atualiza o estado do watchdog, verifica travamentos e executa reset se necessário. */
void sensor_watchdog_update(void);

/**
 * @brief Consulta se um sensor específico está travado.
 * @param sensor_id ID do sensor (0 ou 1)
 * @return true se o sensor está travado, false caso contrário
 */
bool sensor_watchdog_is_sensor_frozen(uint8_t sensor_id);

/** @brief Consulta se algum sensor monitorado está travado. */
bool sensor_watchdog_any_sensor_frozen(void);

/** @brief Força o reset do sistema via watchdog, após reset do barramento I2C. */
void sensor_watchdog_reset_system(void);

/** @brief Imprime o status atual do watchdog e dos sensores monitorados. */
void sensor_watchdog_print_status(void);

#endif /* SENSOR_WATCHDOG_H_ */
