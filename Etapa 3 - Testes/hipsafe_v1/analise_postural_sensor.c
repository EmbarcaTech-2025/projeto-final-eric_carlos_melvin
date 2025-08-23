#include "drivers/mpu6050/mpu6050_i2c.h"
#include "Fusion/Fusion.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "analise_postural_sensor.h"

#ifndef PI
#define PI 3.14159265359f
#endif

// Configurações da aplicação
#define SAMPLE_RATE 100.0f
#define SAMPLE_PERIOD (1.0f / SAMPLE_RATE)

// Variáveis globais para armazenar os últimos ângulos lidos
float g_roll = 0.0f, g_pitch = 0.0f, g_yaw = 0.0f;

// Variáveis globais para Fusion
CalibrationData accel_cal = {0};
CalibrationData gyro_cal = {0};
FusionAhrs ahrs;
bool fusion_initialized = false;

// Função para converter dados brutos do MPU6050 para unidades físicas
void convert_mpu6050_data(int16_t raw_accel[3], int16_t raw_gyro[3], FusionVector *accel, FusionVector *gyro) 
{
    // Obter configurações de range
    uint8_t accel_range = mpu6050_get_accel_range();
    uint8_t gyro_range = mpu6050_get_gyro_range();
    
    // Determinar sensibilidade do acelerômetro
    float accel_sensitivity;
    switch (accel_range) 
    {
        case 0: accel_sensitivity = ACCEL_SENS_2G; break;
        case 1: accel_sensitivity = ACCEL_SENS_4G; break;
        case 2: accel_sensitivity = ACCEL_SENS_8G; break;
        case 3: accel_sensitivity = ACCEL_SENS_16G; break;
        default: accel_sensitivity = ACCEL_SENS_2G; break;
    }
    
    // Determinar sensibilidade do giroscópio
    float gyro_sensitivity;
    switch (gyro_range) 
    {
        case 0: gyro_sensitivity = GYRO_SENS_250DPS; break;
        case 1: gyro_sensitivity = GYRO_SENS_500DPS; break;
        case 2: gyro_sensitivity = GYRO_SENS_1000DPS; break;
        case 3: gyro_sensitivity = GYRO_SENS_2000DPS; break;
        default: gyro_sensitivity = GYRO_SENS_250DPS; break;
    }
    
    // Converter para unidades físicas
    accel->axis.x = (float)raw_accel[0] / accel_sensitivity;
    accel->axis.y = (float)raw_accel[1] / accel_sensitivity;
    accel->axis.z = (float)raw_accel[2] / accel_sensitivity;
    
    gyro->axis.x = (float)raw_gyro[0] / gyro_sensitivity;
    gyro->axis.y = (float)raw_gyro[1] / gyro_sensitivity;
    gyro->axis.z = (float)raw_gyro[2] / gyro_sensitivity;
}

// Função para aplicar calibração aos dados dos sensores
FusionVector apply_calibration(FusionVector raw_data, CalibrationData *cal) 
{
    if (!cal->calibrated) 
    {
        return raw_data;
    }
    
    return FusionCalibrationInertial(raw_data, cal->misalignment, 
                                   cal->sensitivity, cal->offset);
}

// Requisita as posições dos sensores e atualiza os ângulos globais
bool requisitaPosicoes() 
{
    // Verificar se o sistema foi inicializado
    if (!fusion_initialized) 
    {
        printf("Erro: Sistema Fusion não inicializado!\n");
        return false;
    }
    
    int16_t raw_accel[3], raw_gyro[3], temp;
    FusionVector accel_raw, gyro_raw;
    
    // Ler dados brutos do MPU6050
    mpu6050_read_raw(raw_accel, raw_gyro, &temp);
    
    // Converter para unidades físicas
    convert_mpu6050_data(raw_accel, raw_gyro, &accel_raw, &gyro_raw);
    
    // Aplicar calibração
    FusionVector accel_calibrated = apply_calibration(accel_raw, &accel_cal);
    FusionVector gyro_calibrated = apply_calibration(gyro_raw, &gyro_cal);
    
    // Atualizar AHRS (sem magnetômetro)
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyro_calibrated, accel_calibrated, SAMPLE_PERIOD);
    
    // Obter orientação em ângulos de Euler
    FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    
    // Atualizar variáveis globais
    g_roll = euler.angle.roll;
    g_pitch = euler.angle.pitch;
    g_yaw = euler.angle.yaw;
    
    // Exibe os resultados (mantido conforme solicitado)
    printf("Roll: %6.2f°, Pitch: %6.2f°, Yaw: %6.2f°\n", g_roll, g_pitch, g_yaw);
    
    return true;
}

// Compara posições atuais com limites de segurança
bool comparaPosicoes() 
{
    // Aqui pode-se adicionar outras regras de comparação se necessário
    // Para este exemplo, sempre retorna true
    return true;
}

// Verifica se a posição é perigosa (roll > 90 graus)
bool posicaoPerigosa() 
{
    return (g_roll > 90.0f);
}