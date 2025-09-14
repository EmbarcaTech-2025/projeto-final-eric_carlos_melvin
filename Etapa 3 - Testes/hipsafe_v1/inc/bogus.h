#ifndef BOGUS_H_
#define BOGUS_H_

#include "estruturas_de_dados.hpp"
// #include "analise_postural.h"

// I2C addresses for MPU6050
#define MPU6050_ADDR_0 0x68 // Default I2C address
#define MPU6050_ADDR_1 0x69 // Alternate I2C address
// GPIO pins for interfacing I2C
#define I2C0_SDA 0 // Default SDA pin
#define I2C0_SCL 1 // Default SCL pin
#define I2C1_SDA 2 // Alternate SDA pin
#define I2C1_SCL 3 // Alternate SCL pin
#define i2c1 1
#define i2c0 0

// Funções de mock
void buzzer_init();
void sd_card_init();
void setup_buttons();

void stdio_init_all(); // Inicializa UART/USB para debug

// Funções (mock) de configuração do mpu6050 ---> trocar pelo mpu9350
    void mpu6050_setup_i2c(mpu6050_t *mpu);
    void mpu6050_reset(mpu6050_t *mpu);
    uint8_t mpu6050_get_accel_range(mpu6050_t *mpu); // Returns 0=±2g, 1=±4g, 2=±8g, 3=±16g
    void mpu6050_set_accel_range(mpu6050_t *mpu, uint8_t range); // 0=±2g, 1=±4g, 2=±8g, 3=±16g
    uint8_t mpu6050_get_gyro_range(mpu6050_t *mpu); // Returns 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
    void mpu6050_set_gyro_range(mpu6050_t *mpu, uint8_t range); // 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
    void mpu6050_read_raw(mpu6050_t *mpu, int16_t accel[3], int16_t gyro[3], int16_t *temp);

#endif