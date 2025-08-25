#include "./mpu6050_i2c.h"

#define I2C_PORT i2c1
#define I2C_SDA 2
#define I2C_SCL 3
#define MPU6050_ADDR_0 0x68
#define MPU6050_ADDR_1 0x69

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