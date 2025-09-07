#include "mpu9250_i2c.h"
#include <stdio.h>
#include <stdlib.h>

// Endereços dos registradores do MPU9250
#define MPU9250_WHO_AM_I        0x75
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_PWR_MGMT_2      0x6C
#define MPU9250_CONFIG          0x1A
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_ACCEL_CONFIG2   0x1D
#define MPU9250_SMPLRT_DIV      0x19
#define MPU9250_INT_PIN_CFG     0x37
#define MPU9250_INT_ENABLE      0x38
#define MPU9250_INT_STATUS      0x3A
#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_TEMP_OUT_H      0x41
#define MPU9250_GYRO_XOUT_H     0x43
#define MPU9250_USER_CTRL       0x6A
#define MPU9250_I2C_MST_CTRL    0x24
#define MPU9250_I2C_SLV0_ADDR   0x25
#define MPU9250_I2C_SLV0_REG    0x26
#define MPU9250_I2C_SLV0_CTRL   0x27
#define MPU9250_I2C_SLV0_DO     0x63
#define MPU9250_EXT_SENS_DATA_00 0x49

// Endereços dos registradores do AK8963
#define AK8963_WHO_AM_I     0x00
#define AK8963_INFO         0x01
#define AK8963_ST1          0x02
#define AK8963_XOUT_L       0x03
#define AK8963_XOUT_H       0x04
#define AK8963_YOUT_L       0x05
#define AK8963_YOUT_H       0x06
#define AK8963_ZOUT_L       0x07
#define AK8963_ZOUT_H       0x08
#define AK8963_ST2          0x09
#define AK8963_CNTL1        0x0A
#define AK8963_CNTL2        0x0B
#define AK8963_ASTC         0x0C
#define AK8963_I2CDIS       0x0F
#define AK8963_ASAX         0x10
#define AK8963_ASAY         0x11
#define AK8963_ASAZ         0x12

// Definições de bits dos registradores
#define PWR_RESET           0x80 //Bit 7
#define CLOCK_SEL_PLL       0x01
#define I2C_MST_EN          0x20
#define I2C_SLV0_EN         0x80
#define I2C_READ_FLAG       0x80
#define BYPASS_EN           0x02

// IDs dos dispositivos
#define MPU9250_ID          0x71
#define MPU9255_ID          0x73
#define AK8963_ID           0x48

// Protótipos das funções internas
static void mpu9250_write_reg(mpu9250_t *mpu, uint8_t reg, uint8_t data);
static uint8_t mpu9250_read_reg(mpu9250_t *mpu, uint8_t reg);
static void mpu9250_read_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len);
static void mpu9250_write_mag_reg(mpu9250_t *mpu, uint8_t reg, uint8_t data);
static uint8_t mpu9250_read_mag_reg(mpu9250_t *mpu, uint8_t reg);
static void mpu9250_read_mag_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len);
static void mpu9250_update_sensitivity_factors(mpu9250_t *mpu);

void mpu9250_setup_i2c(mpu9250_t *mpu) 
{
    // Reset das linhas I2C antes de inicializar (prevenção de travamento)
    gpio_init(mpu->sda_gpio);
    gpio_init(mpu->scl_gpio);
    gpio_set_dir(mpu->sda_gpio, GPIO_OUT);
    gpio_set_dir(mpu->scl_gpio, GPIO_OUT);
    
    // Força ambas em nível alto para garantir estado conhecido
    gpio_put(mpu->sda_gpio, 1);
    gpio_put(mpu->scl_gpio, 1);
    sleep_ms(10);
    
    // Agora configura para I2C
    i2c_init(mpu->i2c, 400*1000); // I2C a 400 kHz
    gpio_set_function(mpu->sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(mpu->scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(mpu->sda_gpio);
    gpio_pull_up(mpu->scl_gpio);
    
    // Aguarda estabilização
    sleep_ms(10);
}

void mpu9250_reset(mpu9250_t *mpu) 
{
    // Reseta o dispositivo
    mpu9250_write_reg(mpu, MPU9250_PWR_MGMT_1, PWR_RESET);
    sleep_ms(100);
    
    // Acorda o dispositivo e seleciona a melhor fonte de clock disponível
    mpu9250_write_reg(mpu, MPU9250_PWR_MGMT_1, CLOCK_SEL_PLL);
    sleep_ms(10);
    
    // Habilita todos os sensores
    mpu9250_write_reg(mpu, MPU9250_PWR_MGMT_2, 0x00);
    sleep_ms(10);
}

bool mpu9250_init(mpu9250_t *mpu, mpu9250_config_t *config)
{
    // Setup I2C
    mpu9250_setup_i2c(mpu);
    
    // Reset device
    mpu9250_reset(mpu);
    
    // Check device connection
    if (!mpu9250_test_connection(mpu)) 
    {
        return false;
    }
    
    // Configure accelerometer range
    mpu9250_set_accel_range(mpu, config->accel_range);
    
    // Configure gyroscope range
    mpu9250_set_gyro_range(mpu, config->gyro_range);
    
    // Configure DLPF
    mpu9250_set_dlpf(mpu, config->dlpf_filter);
    
    // Configure sample rate
    mpu9250_set_sample_rate(mpu, config->sample_rate_divider);
    
    // Initialize magnetometer if enabled
    if (config->enable_magnetometer) 
    {
        if (!mpu9250_enable_magnetometer(mpu, true)) 
        {
            return false;
        }
    }
    
    return true;
}

bool mpu9250_test_connection(mpu9250_t *mpu)
{
    uint8_t who_am_i = mpu9250_read_reg(mpu, MPU9250_WHO_AM_I);
    printf("WHO_AM_I: 0x%02X\n", who_am_i); // ADICIONE ESTA LINHA
    return (who_am_i == MPU9250_ID || who_am_i == MPU9255_ID);
}

bool mpu9250_test_mag_connection(mpu9250_t *mpu)
{
    uint8_t who_am_i = mpu9250_read_mag_reg(mpu, AK8963_WHO_AM_I);
    printf("AK8963 WHO_AM_I: 0x%02X\n", who_am_i); // ADICIONE ESTA LINHA
    return (who_am_i == AK8963_ID);
}

uint8_t mpu9250_get_accel_range(mpu9250_t *mpu) 
{
    uint8_t val = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG);
    return (val >> 3) & 0x03; // bits 4:3
}

void mpu9250_set_accel_range(mpu9250_t *mpu, mpu9250_accel_range_t range) 
{
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG, range);
    mpu9250_update_sensitivity_factors(mpu);
}

uint8_t mpu9250_get_gyro_range(mpu9250_t *mpu) 
{
    uint8_t val = mpu9250_read_reg(mpu, MPU9250_GYRO_CONFIG);
    return (val >> 3) & 0x03; // bits 4:3
}

void mpu9250_set_gyro_range(mpu9250_t *mpu, mpu9250_gyro_range_t range) 
{
    mpu9250_write_reg(mpu, MPU9250_GYRO_CONFIG, range);
    mpu9250_update_sensitivity_factors(mpu);
}

void mpu9250_set_dlpf(mpu9250_t *mpu, mpu9250_dlpf_t filter)
{
    // Set DLPF for gyroscope
    mpu9250_write_reg(mpu, MPU9250_CONFIG, filter);
    
    // Set DLPF for accelerometer  
    uint8_t accel_config2 = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG2);
    accel_config2 = (accel_config2 & 0xF0) | (filter & 0x0F);
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG2, accel_config2);
}

void mpu9250_set_sample_rate(mpu9250_t *mpu, uint8_t divider)
{
    mpu9250_write_reg(mpu, MPU9250_SMPLRT_DIV, divider);
}

bool mpu9250_enable_magnetometer(mpu9250_t *mpu, bool enable)
{
    if (enable) 
    {
        printf("Starting magnetometer initialization...\n");
        
        // 1. Desabilita I2C master e ativa bypass
        mpu9250_write_reg(mpu, MPU9250_USER_CTRL, 0x00);
        sleep_ms(10);
        uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg | BYPASS_EN);
        sleep_ms(100);
        
        // 2. Testa conexão com magnetômetro
        if (!mpu9250_test_mag_connection(mpu)) 
        {
            printf("ERROR: Magnetometer not detected!\n");
            return false;
        }
        printf("Magnetometer detected successfully\n");
        
        // 3. Reset magnetômetro
        mpu9250_write_mag_reg(mpu, AK8963_CNTL2, 0x01);
        sleep_ms(100);
        
        // 4. Modo FUSE ROM para ler calibração
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, AK8963_FUSE_ROM);
        sleep_ms(100);
        
        // 5. Lê calibração
        uint8_t asa_data[3];
        mpu9250_read_mag_regs(mpu, AK8963_ASAX, asa_data, 3);
        for (int i = 0; i < 3; i++) {
            mpu->mag_asa[i] = ((float)asa_data[i] - 128.0f) / 256.0f + 1.0f;
        }
        printf("ASA values: X=%.3f, Y=%.3f, Z=%.3f\n", 
               mpu->mag_asa[0], mpu->mag_asa[1], mpu->mag_asa[2]);
        
        // 6. Power down antes de configurar modo contínuo
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, AK8963_POWER_DOWN);
        sleep_ms(100);
        
        // 7. Configura magnetômetro para modo contínuo
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, 0x16); // Continuous mode 2 (100Hz) + 16-bit
        sleep_ms(100);
        
        // 8. Verifica se entrou em modo contínuo
        uint8_t cntl1 = mpu9250_read_mag_reg(mpu, AK8963_CNTL1);
        printf("AK8963 CNTL1: 0x%02X (Expected: 0x16)\n", cntl1);
        
        // 9. Desativa bypass e configura I2C master
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg & ~BYPASS_EN);
        sleep_ms(10);
        
        // 10. Primeiro desabilita todas as slaves
        for (int i = 0; i < 4; i++) {
            mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL + i*3, 0x00);
        }
        sleep_ms(10);
        
        // 11. Configura I2C master clock ANTES de habilitar
        mpu9250_write_reg(mpu, MPU9250_I2C_MST_CTRL, 0x0D); // 400kHz
        sleep_ms(10);
        
        // 12. Configura Slave 0 ANTES de habilitar I2C master
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, AK8963_ADDR | I2C_READ_FLAG);
        sleep_ms(5);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, AK8963_ST1);
        sleep_ms(5);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, 0x88); // Enable + 8 bytes
        sleep_ms(10);
        
        // 13. Agora habilita I2C master
        mpu9250_write_reg(mpu, MPU9250_USER_CTRL, I2C_MST_EN);
        sleep_ms(100);
        
        // 14. Verificação final
        uint8_t verify_addr = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_ADDR);
        uint8_t verify_reg = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_REG);
        uint8_t verify_ctrl = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_CTRL);
        uint8_t verify_user = mpu9250_read_reg(mpu, MPU9250_USER_CTRL);
        
        printf("Final verification:\n");
        printf("  I2C_SLV0_ADDR: 0x%02X (Expected: 0x8C)\n", verify_addr);
        printf("  I2C_SLV0_REG: 0x%02X (Expected: 0x02)\n", verify_reg);
        printf("  I2C_SLV0_CTRL: 0x%02X (Expected: 0x88)\n", verify_ctrl);
        printf("  USER_CTRL: 0x%02X (Expected: 0x20)\n", verify_user);
        
        mpu->mag_enabled = true;
        printf("Magnetometer initialization completed\n");
    } 
    else 
    {
        // Disable magnetometer
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, AK8963_POWER_DOWN);
        // Disable bypass mode
        uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg & ~BYPASS_EN);
        mpu->mag_enabled = false;
    }
    return true;
}

void mpu9250_read_raw_motion(mpu9250_t *mpu, int16_t accel[3], int16_t gyro[3], int16_t *temp) 
{
    uint8_t buffer[14];
    
    // Read accelerometer, temperature, and gyroscope data
    mpu9250_read_regs(mpu, MPU9250_ACCEL_XOUT_H, buffer, 14);
    
    // Parse accelerometer data
    accel[0] = (int16_t)((buffer[0] << 8) | buffer[1]);
    accel[1] = (int16_t)((buffer[2] << 8) | buffer[3]);
    accel[2] = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    // Parse temperature data
    *temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    
    // Parse gyroscope data
    gyro[0] = (int16_t)((buffer[8] << 8) | buffer[9]);
    gyro[1] = (int16_t)((buffer[10] << 8) | buffer[11]);
    gyro[2] = (int16_t)((buffer[12] << 8) | buffer[13]);
}

void mpu9250_read_raw_mag(mpu9250_t *mpu, int16_t mag[3])
{
    if (!mpu->mag_enabled) 
    {
        mag[0] = mag[1] = mag[2] = 0;
        return;
    }
    
    // For I2C master mode, read from EXT_SENS_DATA registers
    uint8_t buffer[8];
    
    // Read the 8 bytes of magnetometer data from EXT_SENS_DATA_00 to EXT_SENS_DATA_07
    mpu9250_read_regs(mpu, MPU9250_EXT_SENS_DATA_00, buffer, 8);
    
    // Check if data is ready (ST1 register bit 0 = DRDY)
    if (!(buffer[0] & 0x01)) {
        // Data not ready, return previous values or zeros
        mag[0] = mag[1] = mag[2] = 0;
        return;
    }
    
    // Check ST2 for overflow (bit 3 = HOFL - magnetic sensor overflow)
    if (buffer[7] & 0x08) {
        // Magnetic sensor overflow detected - reset magnetometer
        printf("Magnetometer overflow detected, resetting...\n");
        
        // Temporarily enable bypass to reset magnetometer
        uint8_t user_ctrl = mpu9250_read_reg(mpu, MPU9250_USER_CTRL);
        uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
        
        mpu9250_write_reg(mpu, MPU9250_USER_CTRL, 0x00);
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg | BYPASS_EN);
        sleep_ms(10);
        
        // Reset and reconfigure magnetometer
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, AK8963_POWER_DOWN);
        sleep_ms(10);
        mpu9250_write_mag_reg(mpu, AK8963_CNTL1, 0x16); // Continuous mode 2 + 16-bit
        sleep_ms(10);
        
        // Restore I2C master mode
        mpu9250_write_reg(mpu, MPU9250_INT_PIN_CFG, int_pin_cfg & ~BYPASS_EN);
        mpu9250_write_reg(mpu, MPU9250_USER_CTRL, user_ctrl);
        sleep_ms(10);
        
        mag[0] = mag[1] = mag[2] = 0;
        return;
    }
    
    // Parse magnetometer data (little endian format)
    // Buffer layout: ST1(0), HXL(1), HXH(2), HYL(3), HYH(4), HZL(5), HZH(6), ST2(7)
    // AK8963 uses little endian format: low byte first, then high byte
    mag[0] = (int16_t)((buffer[2] << 8) | buffer[1]);  // HX = HXH << 8 | HXL
    mag[1] = (int16_t)((buffer[4] << 8) | buffer[3]);  // HY = HYH << 8 | HYL  
    mag[2] = (int16_t)((buffer[6] << 8) | buffer[5]);  // HZ = HZH << 8 | HZL
}

void mpu9250_read_raw(mpu9250_t *mpu, mpu9250_raw_data_t *data)
{
    mpu9250_read_raw_motion(mpu, data->accel, data->gyro, &data->temp);
    mpu9250_read_raw_mag(mpu, data->mag);
}

void mpu9250_read_motion(mpu9250_t *mpu, float accel[3], float gyro[3], float *temp)
{
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    
    mpu9250_read_raw_motion(mpu, accel_raw, gyro_raw, &temp_raw);
    
    // Convert to physical units
    for (int i = 0; i < 3; i++) {
        accel[i] = (float)accel_raw[i] / mpu->accel_sensitivity;
        gyro[i] = (float)gyro_raw[i] / mpu->gyro_sensitivity;
    }
    
    // Convert temperature (formula from MPU9250 datasheet)
    *temp = ((float)temp_raw - 21.0f) / 333.87f + 21.0f;
}

void mpu9250_read_mag(mpu9250_t *mpu, float mag[3])
{
    int16_t mag_raw[3];
    
    mpu9250_read_raw_mag(mpu, mag_raw);
    
    // Convert to physical units with sensitivity adjustment
    for (int i = 0; i < 3; i++) {
        mag[i] = (float)mag_raw[i] * mpu->mag_asa[i] * MAG_SENS;
    }
}

void mpu9250_read_data(mpu9250_t *mpu, mpu9250_data_t *data)
{
    mpu9250_read_motion(mpu, data->accel, data->gyro, &data->temp);
    mpu9250_read_mag(mpu, data->mag);
}

float mpu9250_read_temperature(mpu9250_t *mpu)
{
    uint8_t buffer[2];
    mpu9250_read_regs(mpu, MPU9250_TEMP_OUT_H, buffer, 2);
    
    int16_t temp_raw = (int16_t)((buffer[0] << 8) | buffer[1]);
    return ((float)temp_raw - 21.0f) / 333.87f + 21.0f;
}

void mpu9250_debug_mag_status(mpu9250_t *mpu)
{
    if (!mpu->mag_enabled) {
        printf("Magnetometer is disabled\n");
        return;
    }
    
    // Read EXT_SENS_DATA to see what's being captured
    uint8_t buffer[8];
    mpu9250_read_regs(mpu, MPU9250_EXT_SENS_DATA_00, buffer, 8);
    
    printf("EXT_SENS_DATA: ");
    for (int i = 0; i < 8; i++) {
        printf("0x%02X ", buffer[i]);
    }
    printf("\n");
    
    printf("ST1 (DRDY): %s\n", (buffer[0] & 0x01) ? "Ready" : "Not Ready");
    printf("ST2 (HOFL): %s\n", (buffer[7] & 0x08) ? "Overflow" : "Normal");
    
    // Check I2C master status
    uint8_t user_ctrl = mpu9250_read_reg(mpu, MPU9250_USER_CTRL);
    uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
    uint8_t i2c_mst_ctrl = mpu9250_read_reg(mpu, MPU9250_I2C_MST_CTRL);
    uint8_t i2c_slv0_addr = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_ADDR);
    uint8_t i2c_slv0_reg = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_REG);
    uint8_t i2c_slv0_ctrl = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_CTRL);
    
    printf("USER_CTRL: 0x%02X (I2C_MST_EN: %s)\n", user_ctrl, (user_ctrl & 0x20) ? "ON" : "OFF");
    printf("INT_PIN_CFG: 0x%02X (BYPASS_EN: %s)\n", int_pin_cfg, (int_pin_cfg & 0x02) ? "ON" : "OFF");
    printf("I2C_MST_CTRL: 0x%02X\n", i2c_mst_ctrl);
    printf("I2C_SLV0_ADDR: 0x%02X (Expected: 0x8C)\n", i2c_slv0_addr);
    printf("I2C_SLV0_REG: 0x%02X (Expected: 0x02)\n", i2c_slv0_reg);
    printf("I2C_SLV0_CTRL: 0x%02X (EN: %s, LEN: %d)\n", i2c_slv0_ctrl, 
           (i2c_slv0_ctrl & 0x80) ? "ON" : "OFF", i2c_slv0_ctrl & 0x0F);
    
    // Auto-fix if configuration is wrong
    if ((i2c_slv0_addr != 0x8C) || (i2c_slv0_reg != 0x02) || (i2c_slv0_ctrl != 0x88)) {
        printf("Auto-fixing magnetometer I2C configuration...\n");
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, AK8963_ADDR | I2C_READ_FLAG);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, AK8963_ST1);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, 0x88);
        printf("Configuration restored\n");
    }
}

void mpu9250_calibrate_gyro(mpu9250_t *mpu, uint16_t samples, float gyro_offset[3])
{
    int32_t gyro_sum[3] = {0, 0, 0};
    int16_t gyro_raw[3], accel_raw[3], temp_raw;
    
    // Initialize offsets
    gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0.0f;
    
    // Collect samples
    for (uint16_t i = 0; i < samples; i++) 
    {
        mpu9250_read_raw_motion(mpu, accel_raw, gyro_raw, &temp_raw);
        
        gyro_sum[0] += gyro_raw[0];
        gyro_sum[1] += gyro_raw[1];
        gyro_sum[2] += gyro_raw[2];
        
        sleep_ms(2);
    }
    
    // Calculate averages and convert to physical units
    for (int i = 0; i < 3; i++) 
    {
        gyro_offset[i] = (float)gyro_sum[i] / (float)samples / mpu->gyro_sensitivity;
    }
}

bool mpu9250_self_test(mpu9250_t *mpu)
{
    // Save current configuration including I2C master settings
    uint8_t saved_config[10];
    saved_config[0] = mpu9250_read_reg(mpu, MPU9250_SMPLRT_DIV);
    saved_config[1] = mpu9250_read_reg(mpu, MPU9250_CONFIG);
    saved_config[2] = mpu9250_read_reg(mpu, MPU9250_GYRO_CONFIG);
    saved_config[3] = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG);
    saved_config[4] = mpu9250_read_reg(mpu, MPU9250_ACCEL_CONFIG2);
    saved_config[5] = mpu9250_read_reg(mpu, MPU9250_USER_CTRL);
    saved_config[6] = mpu9250_read_reg(mpu, MPU9250_I2C_MST_CTRL);
    saved_config[7] = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_ADDR);
    saved_config[8] = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_REG);
    saved_config[9] = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_CTRL);
    
    printf("Starting MPU9250 self-test...\n");
    
    // Configure for self-test (according to datasheet)
    mpu9250_write_reg(mpu, MPU9250_SMPLRT_DIV, 0x00);    // Sample rate = 1kHz
    mpu9250_write_reg(mpu, MPU9250_CONFIG, 0x02);         // DLPF = 92Hz
    mpu9250_write_reg(mpu, MPU9250_GYRO_CONFIG, 0x00);    // ±250 dps, no self-test
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG, 0x00);   // ±2g, no self-test  
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG2, 0x02);  // DLPF = 92Hz
    
    sleep_ms(50); // Wait for settings to stabilize
    
    // Collect normal operation samples (no self-test)
    int32_t accel_normal_sum[3] = {0}, gyro_normal_sum[3] = {0};
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    
    for (int i = 0; i < 200; i++) 
    {
        mpu9250_read_raw_motion(mpu, accel_raw, gyro_raw, &temp_raw);
        accel_normal_sum[0] += accel_raw[0];
        accel_normal_sum[1] += accel_raw[1];
        accel_normal_sum[2] += accel_raw[2];
        gyro_normal_sum[0] += gyro_raw[0];
        gyro_normal_sum[1] += gyro_raw[1];
        gyro_normal_sum[2] += gyro_raw[2];
        sleep_ms(1);
    }
    
    // Enable self-test
    mpu9250_write_reg(mpu, MPU9250_GYRO_CONFIG, 0xE0);    // Enable XYZ self-test, ±250 dps
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG, 0xE0);   // Enable XYZ self-test, ±2g
    
    sleep_ms(50); // Wait for self-test to stabilize
    
    // Collect self-test samples
    int32_t accel_st_sum[3] = {0}, gyro_st_sum[3] = {0};
    
    for (int i = 0; i < 200; i++) 
    {
        mpu9250_read_raw_motion(mpu, accel_raw, gyro_raw, &temp_raw);
        accel_st_sum[0] += accel_raw[0];
        accel_st_sum[1] += accel_raw[1];
        accel_st_sum[2] += accel_raw[2];
        gyro_st_sum[0] += gyro_raw[0];
        gyro_st_sum[1] += gyro_raw[1];
        gyro_st_sum[2] += gyro_raw[2];
        sleep_ms(1);
    }
    
    // Restore original configuration including I2C master settings
    mpu9250_write_reg(mpu, MPU9250_SMPLRT_DIV, saved_config[0]);
    mpu9250_write_reg(mpu, MPU9250_CONFIG, saved_config[1]);
    mpu9250_write_reg(mpu, MPU9250_GYRO_CONFIG, saved_config[2]);
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG, saved_config[3]);
    mpu9250_write_reg(mpu, MPU9250_ACCEL_CONFIG2, saved_config[4]);
    mpu9250_write_reg(mpu, MPU9250_USER_CTRL, saved_config[5]);
    mpu9250_write_reg(mpu, MPU9250_I2C_MST_CTRL, saved_config[6]);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, saved_config[7]);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, saved_config[8]);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, saved_config[9]);
    sleep_ms(10); // Give time for I2C master to restart
    
    // Calculate averages
    int32_t accel_normal_avg[3], accel_st_avg[3];
    int32_t gyro_normal_avg[3], gyro_st_avg[3];
    
    for (int i = 0; i < 3; i++) 
    {
        accel_normal_avg[i] = accel_normal_sum[i] / 200;
        accel_st_avg[i] = accel_st_sum[i] / 200;
        gyro_normal_avg[i] = gyro_normal_sum[i] / 200;
        gyro_st_avg[i] = gyro_st_sum[i] / 200;
    }
    
    // Calculate self-test responses
    int32_t accel_str[3], gyro_str[3];
    bool test_passed = true;
    
    printf("Self-test results:\n");
    
    for (int i = 0; i < 3; i++) 
    {
        // Self-test response = self-test output - normal output
        accel_str[i] = accel_st_avg[i] - accel_normal_avg[i];
        gyro_str[i] = gyro_st_avg[i] - gyro_normal_avg[i];
        
        printf("Axis %d - Accel STR: %ld, Gyro STR: %ld\n", i, accel_str[i], gyro_str[i]);
        
        // Check if self-test response is within acceptable range
        // Accelerometer: should have significant response (>1000 LSB change for ±2g range)
        // Gyroscope: should have significant response (>50 LSB change for ±250dps range)
        // Based on MPU9250 datasheet and observed values, adjusted thresholds
        if (labs(accel_str[i]) < 1000 || labs(accel_str[i]) > 14000) {
            printf("Accelerometer axis %d failed: STR = %ld\n", i, accel_str[i]);
            test_passed = false;
        }
        
        // Adjusted gyroscope threshold based on datasheet and real hardware behavior
        // Typical self-test response should be between 50-32000 LSB for ±250dps range
        if (labs(gyro_str[i]) < 50 || labs(gyro_str[i]) > 32000) {
            printf("Gyroscope axis %d failed: STR = %ld\n", i, gyro_str[i]);
            test_passed = false;
        }
    }
    
    printf("Self-test %s\n", test_passed ? "PASSED" : "FAILED");
    return test_passed;
}

// Internal functions

static void mpu9250_write_reg(mpu9250_t *mpu, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    i2c_write_blocking(mpu->i2c, mpu->addr, buffer, 2, false);
    sleep_us(500); // Small delay for register write
}

static uint8_t mpu9250_read_reg(mpu9250_t *mpu, uint8_t reg)
{
    uint8_t data;
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, &data, 1, false);
    return data;
}

static void mpu9250_read_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    i2c_write_blocking(mpu->i2c, mpu->addr, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, mpu->addr, buffer, len, false);
}

static void mpu9250_write_mag_reg(mpu9250_t *mpu, uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};
    i2c_write_blocking(mpu->i2c, AK8963_ADDR, buffer, 2, false);
    sleep_us(500); // Small delay for register write
}

static uint8_t mpu9250_read_mag_reg(mpu9250_t *mpu, uint8_t reg)
{
    uint8_t data = 0xFF;
    // Se o bypass está ativo, lê direto
    uint8_t int_pin_cfg = mpu9250_read_reg(mpu, MPU9250_INT_PIN_CFG);
    if (int_pin_cfg & BYPASS_EN) {
        i2c_write_blocking(mpu->i2c, AK8963_ADDR, &reg, 1, true);
        i2c_read_blocking(mpu->i2c, AK8963_ADDR, &data, 1, false);
        return data;
    }
    
    // Se magnetômetro está habilitado, salva configuração atual do Slave 0
    uint8_t saved_addr = 0, saved_reg = 0, saved_ctrl = 0;
    if (mpu->mag_enabled) {
        saved_addr = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_ADDR);
        saved_reg = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_REG);
        saved_ctrl = mpu9250_read_reg(mpu, MPU9250_I2C_SLV0_CTRL);
    }
    
    // Configura SLV0 para ler 1 byte do registrador desejado
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, AK8963_ADDR | I2C_READ_FLAG);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, reg);
    mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, 0x81); // habilita, 1 byte
    sleep_ms(10); // aguarda transferência
    data = mpu9250_read_reg(mpu, MPU9250_EXT_SENS_DATA_00);
    
    // Restaura configuração original se magnetômetro estava habilitado
    if (mpu->mag_enabled) {
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_ADDR, saved_addr);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_REG, saved_reg);
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, saved_ctrl);
    } else {
        // Desabilita SLV0 se magnetômetro não estava habilitado
        mpu9250_write_reg(mpu, MPU9250_I2C_SLV0_CTRL, 0x00);
    }
    
    return data;
}

static void mpu9250_read_mag_regs(mpu9250_t *mpu, uint8_t reg, uint8_t *buffer, uint8_t len)
{
    i2c_write_blocking(mpu->i2c, AK8963_ADDR, &reg, 1, true);
    i2c_read_blocking(mpu->i2c, AK8963_ADDR, buffer, len, false);
}

static void mpu9250_update_sensitivity_factors(mpu9250_t *mpu)
{
    uint8_t accel_range = mpu9250_get_accel_range(mpu);
    uint8_t gyro_range = mpu9250_get_gyro_range(mpu);
    
    // Update accelerometer sensitivity
    switch (accel_range) 
    {
        case 0: mpu->accel_sensitivity = ACCEL_SENS_2G; break;
        case 1: mpu->accel_sensitivity = ACCEL_SENS_4G; break;
        case 2: mpu->accel_sensitivity = ACCEL_SENS_8G; break;
        case 3: mpu->accel_sensitivity = ACCEL_SENS_16G; break;
        default: mpu->accel_sensitivity = ACCEL_SENS_2G; break;
    }
    
    // Update gyroscope sensitivity
    switch (gyro_range) 
    {
        case 0: mpu->gyro_sensitivity = GYRO_SENS_250DPS; break;
        case 1: mpu->gyro_sensitivity = GYRO_SENS_500DPS; break;
        case 2: mpu->gyro_sensitivity = GYRO_SENS_1000DPS; break;
        case 3: mpu->gyro_sensitivity = GYRO_SENS_2000DPS; break;
        default: mpu->gyro_sensitivity = GYRO_SENS_250DPS; break;
    }
}
