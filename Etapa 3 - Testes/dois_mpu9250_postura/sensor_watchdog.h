#ifndef SENSOR_WATCHDOG_H
#define SENSOR_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>
#include "mpu9250_i2c.h"
#include "pico/stdlib.h"

// Definições
#define WATCHDOG_TIMEOUT_MS         8000    // Timeout do watchdog (8 segundos)
#define SENSOR_FREEZE_THRESHOLD     50      // Número de amostras para detectar travamento
#define MAX_SENSORS                 2       // Número máximo de sensores
#define SENSOR_DATA_TOLERANCE       0.001f  // Tolerância para mudança nos dados

// Estrutura para armazenar histórico de dados do sensor
typedef struct {
    int16_t accel_x_history[SENSOR_FREEZE_THRESHOLD];
    int16_t accel_y_history[SENSOR_FREEZE_THRESHOLD];
    int16_t accel_z_history[SENSOR_FREEZE_THRESHOLD];
    int16_t gyro_x_history[SENSOR_FREEZE_THRESHOLD];
    int16_t gyro_y_history[SENSOR_FREEZE_THRESHOLD];
    int16_t gyro_z_history[SENSOR_FREEZE_THRESHOLD];
    int16_t mag_x_history[SENSOR_FREEZE_THRESHOLD];
    int16_t mag_y_history[SENSOR_FREEZE_THRESHOLD];
    int16_t mag_z_history[SENSOR_FREEZE_THRESHOLD];
    uint32_t sample_count;
    uint32_t history_index;
    bool is_frozen;
    bool is_initialized;
    uint8_t sensor_id;
} sensor_watchdog_data_t;

// Estrutura principal do watchdog
typedef struct {
    sensor_watchdog_data_t sensors[MAX_SENSORS];
    bool watchdog_enabled;
    uint32_t last_update_time;
    uint32_t freeze_detection_start_time;
} sensor_watchdog_t;

// Funções públicas
void sensor_watchdog_init(void);
void sensor_watchdog_enable(void);
void sensor_watchdog_disable(void);
void sensor_watchdog_feed(uint8_t sensor_id, mpu9250_raw_data_t *raw_data);
void sensor_watchdog_update(void);
bool sensor_watchdog_is_sensor_frozen(uint8_t sensor_id);
bool sensor_watchdog_any_sensor_frozen(void);
void sensor_watchdog_reset_system(void);
void sensor_watchdog_print_status(void);

#endif // SENSOR_WATCHDOG_H
