#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdbool.h>
#include "pico/util/datetime.h"
#include "mpu9250_i2c.h"
#include "sensor_watchdog.h"

// Estrutura para armazenar os dados do IMU
typedef struct {
    float roll;
    float pitch;
    float yaw;
    bool data_ready;
} imu_data_t;

// Funções públicas
void imu_init(mpu9250_t *mpu, mpu9250_config_t *config);
void imu_start(mpu9250_t *mpu);
void imu_start_dual_sensors(mpu9250_t *mpu0, mpu9250_t *mpu1);
void imu_stop(void);
imu_data_t imu_get_data(void);
void imu_print_data(void);
void imu_print_hip_angles(void);

#endif // IMU_DRIVER_H
