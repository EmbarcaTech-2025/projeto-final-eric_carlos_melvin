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

// I2C addresses for MPU6050
#define MPU6050_ADDR_0 0x68 // Default I2C address
#define MPU6050_ADDR_1 0x69 // Alternate I2C address

// GPIO pins for interfacing I2C
#define I2C0_SDA 0 // Default SDA pin
#define I2C0_SCL 1 // Default SCL pin
#define I2C1_SDA 2 // Alternate SDA pin
#define I2C1_SCL 3 // Alternate SCL pin


typedef struct{
    i2c_inst_t *i2c; // I2C instance
    uint sda_gpio;   // SDA GPIO pin
    uint scl_gpio;   // SCL GPIO pin
    uint8_t addr;   // MPU6050 I2C address (0x68 or 0x69)
    uint8_t id;     // MPU6050 Number ID
} mpu6050_t;

// Array de sensores MPU6050
extern mpu6050_t mpu_sensors[3];

// Funções para um sensor específico
void mpu6050_setup_i2c(mpu6050_t *mpu);
void mpu6050_reset(mpu6050_t *mpu);
uint8_t mpu6050_get_accel_range(mpu6050_t *mpu); // Returns 0=±2g, 1=±4g, 2=±8g, 3=±16g
void mpu6050_set_accel_range(mpu6050_t *mpu, uint8_t range); // 0=±2g, 1=±4g, 2=±8g, 3=±16g
uint8_t mpu6050_get_gyro_range(mpu6050_t *mpu); // Returns 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
void mpu6050_set_gyro_range(mpu6050_t *mpu, uint8_t range); // 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
void mpu6050_read_raw(mpu6050_t *mpu, int16_t accel[3], int16_t gyro[3], int16_t *temp);

// Funções para múltiplos sensores
void mpu6050_init_all_sensors(void);
void mpu6050_setup_all_sensors(void);
void mpu6050_reset_all_sensors(void);
void mpu6050_configure_all_sensors(uint8_t accel_range, uint8_t gyro_range);
bool mpu6050_read_all_sensors(int16_t accel_data[3][3], int16_t gyro_data[3][3], int16_t temp_data[3]);

#endif // MPU6050_I2C_H