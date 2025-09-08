#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdbool.h>
#include "pico/util/datetime.h"
#include "mpu9250_i2c.h"
#include "sensor_watchdog.h"
#include "MadgwickAHRS.h"

/* Constantes do programa */
#define SAMPLE_PERIOD_US 10000.0f // Período de amostragem dos MPUs em microssegundos (10us -> 0,01 ms)
#define PRINT_PERIOD_US 10000.0f // Período de impressão dos dados em microssegundos (1s -> 1000000us)
#define SAMPLE_FREQ_HZ 1.0f/(SAMPLE_PERIOD_US / 1000000.0f) // Frequência de amostragem dos MPUs em Hz

/* Estruturas de dados */
typedef struct {
    // Dados usados e obtidos pelo Madgwick AHRS
    AHRS_data_t ahrs;
    float roll, pitch, yaw;
    uint32_t sample_count;
    bool initialized;
} sensor_data_t;

/* Funções públicas */
void imu_init(mpu9250_t *mpu, mpu9250_config_t *config);
void imu_start(mpu9250_t *mpu);
void imu_start_dual_sensors(mpu9250_t *mpu0, mpu9250_t *mpu1);
void imu_stop(void);
void imu_print_data(void);
void imu_print_hip_angles(void);

#endif // IMU_DRIVER_H
