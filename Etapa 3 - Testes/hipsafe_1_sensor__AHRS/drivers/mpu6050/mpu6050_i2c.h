#ifndef MPU6050_I2C_H // include guard
#define MPU6050_I2C_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Accelerometer sensitivity constants
#define ACCEL_SENS_2G  16384.0f
#define ACCEL_SENS_4G  8192.0f
#define ACCEL_SENS_8G  4096.0f
#define ACCEL_SENS_16G 2048.0f

// Gyroscope sensitivity constants (LSB/°/s)
#define GYRO_SENS_250DPS  131.0f
#define GYRO_SENS_500DPS  65.5f
#define GYRO_SENS_1000DPS 32.8f
#define GYRO_SENS_2000DPS 16.4f

void mpu6050_setup_i2c(void);
void mpu6050_reset(void);
uint8_t mpu6050_get_accel_range(void); // Returns 0=±2g, 1=±4g, 2=±8g, 3=±16g
void mpu6050_set_accel_range(uint8_t range) ; // 0=±2g, 1=±4g, 2=±8g, 3=±16g
uint8_t mpu6050_get_gyro_range(void); // Returns 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
void mpu6050_set_gyro_range(uint8_t range); // 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);

#endif // MPU6050_I2C_H