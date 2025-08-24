#include "./mpu6050_i2c.h"
#include <stdio.h>

// Array de estruturas para os 3 sensores MPU6050
mpu6050_t mpu_sensors[3];

void mpu6050_setup_i2c(mpu6050_t *mpu)
{
    i2c_init(mpu->i2c, 400*1000); // common options: 100*1000 (100 kHz) or 400*1000 (400 kHz)
    gpio_set_function(mpu->sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(mpu->scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(mpu->sda_gpio);
    gpio_pull_up(mpu->scl_gpio);
}

void mpu6050_reset(mpu6050_t *mpu) 
{
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(mpu->i2c, mpu->addr, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(mpu->i2c, mpu->addr, buf, 2, false);
    sleep_ms(10);
}

uint8_t mpu6050_get_accel_range(mpu6050_t *mpu) 
{
    uint8_t reg = 0x1C;
    uint8_t val;
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, &val, 1, false);
    return (val >> 3) & 0x03; // bits 4:3
}

// 0=±2g, 1=±4g, 2=±8g, 3=±16g
void mpu6050_set_accel_range(mpu6050_t *mpu, uint8_t range) 
{
    uint8_t buf[2];
    buf[0] = 0x1C; // ACCEL_CONFIG register
    buf[1] = range << 3; // bits 3 e 4
    i2c_write_blocking(mpu->i2c, mpu->addr, buf, 2, false);
}

uint8_t mpu6050_get_gyro_range(mpu6050_t *mpu) 
{
    uint8_t reg = 0x1B;
    uint8_t val;
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, &val, 1, false);
    return (val >> 3) & 0x03; // bits 4:3
}

// 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
void mpu6050_set_gyro_range(mpu6050_t *mpu, uint8_t range) 
{
    uint8_t buf[2];
    buf[0] = 0x1B; // GYRO_CONFIG register
    buf[1] = range << 3; // bits 3 e 4
    i2c_write_blocking(mpu->i2c, mpu->addr, buf, 2, false);
}

void mpu6050_read_raw(mpu6050_t *mpu, int16_t accel[3], int16_t gyro[3], int16_t *temp) 
{
    uint8_t buffer[6];
    uint8_t reg = 0x3B; //MPU6050_REG_ACCEL_XOUT_H
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, buffer, 6, false);
    for (int i=0; i<3; i++)
        accel[i] = (buffer[2*i]<<8) | buffer[2*i+1];

    reg = 0x43; //MPU6050_REG_GYRO_XOUT_H
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, buffer, 6, false);
    for (int i=0; i<3; i++)
        gyro[i] = (buffer[2*i]<<8) | buffer[2*i+1];

    reg = 0x41; //MPU6050_REG_TEMP_OUT_H
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, buffer, 2, false);
    *temp = (buffer[0]<<8) | buffer[1];
}

// Inicializa as estruturas dos 3 sensores MPU6050
void mpu6050_init_all_sensors(void) 
{
    // Sensor 0 - I2C0, endereço 0x68
    mpu_sensors[0].i2c = i2c1;
    mpu_sensors[0].sda_gpio = I2C1_SDA;
    mpu_sensors[0].scl_gpio = I2C1_SCL;
    mpu_sensors[0].addr = MPU6050_ADDR_0;
    mpu_sensors[0].id = 0;

    // Sensor 1 - I2C0, endereço 0x69
    mpu_sensors[1].i2c = i2c1;
    mpu_sensors[1].sda_gpio = I2C1_SDA;
    mpu_sensors[1].scl_gpio = I2C1_SCL;
    mpu_sensors[1].addr = MPU6050_ADDR_1;
    mpu_sensors[1].id = 1;

    // Sensor 2 - I2C1, endereço 0x69
    mpu_sensors[2].i2c = i2c0;
    mpu_sensors[2].sda_gpio = I2C0_SDA;
    mpu_sensors[2].scl_gpio = I2C0_SCL;
    mpu_sensors[2].addr = MPU6050_ADDR_0;
    mpu_sensors[2].id = 2;
}

// Configura I2C para todos os sensores
void mpu6050_setup_all_sensors(void) 
{
    printf("Configurando I2C para todos os sensores MPU6050...\n");
    
    // Configurar I2C0 (sensores 0 e 1)
    mpu6050_setup_i2c(&mpu_sensors[0]);
    printf("I2C0 configurado (sensores 0 e 1)\n");
    
    // Configurar I2C1 (sensor 2)
    mpu6050_setup_i2c(&mpu_sensors[2]);
    printf("I2C1 configurado (sensor 2)\n");
}

// Reinicia todos os sensores
void mpu6050_reset_all_sensors(void) 
{
    printf("Reiniciando todos os sensores MPU6050...\n");
    for (int i = 0; i < 3; i++) {
        printf("Reiniciando sensor %d...\n", i);
        mpu6050_reset(&mpu_sensors[i]);
        sleep_ms(50); // Aguardar entre resets
    }
    printf("Todos os sensores reiniciados\n");
}

// Configura o range de todos os sensores
void mpu6050_configure_all_sensors(uint8_t accel_range, uint8_t gyro_range) 
{
    printf("Configurando range de todos os sensores (±%dg, ±%d°/s)...\n", 
           (2 << accel_range), (250 << gyro_range));
    
    for (int i = 0; i < 3; i++) {
        printf("Configurando sensor %d...\n", i);
        mpu6050_set_accel_range(&mpu_sensors[i], accel_range);
        mpu6050_set_gyro_range(&mpu_sensors[i], gyro_range);
        sleep_ms(10); // Pequena pausa entre configurações
    }
    printf("Todos os sensores configurados\n");
}

// Lê dados de todos os sensores
bool mpu6050_read_all_sensors(int16_t accel_data[3][3], int16_t gyro_data[3][3], int16_t temp_data[3]) 
{
    for (int i = 0; i < 3; i++) {
        mpu6050_read_raw(&mpu_sensors[i], accel_data[i], gyro_data[i], &temp_data[i]);
    }
    return true; // Assumindo sucesso por enquanto
}