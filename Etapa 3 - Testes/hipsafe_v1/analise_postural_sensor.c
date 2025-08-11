#include "drivers/mpu6050/mpu6050_i2c.h"
#include "drivers/filter/madgwickFilter.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265359f
#endif

// Variáveis globais para armazenar os últimos ângulos lidos
float g_roll = 0.0f, g_pitch = 0.0f, g_yaw = 0.0f;

// Requisita as posições dos sensores e atualiza os ângulos globais
bool requisitaPosicoes() 
{
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    float ax, ay, az, gx, gy, gz;
    
    // Constantes de conversão
    const float accel_scale = 1.0f / ACCEL_SENS_2G;    // Para ±2g
    const float gyro_scale = 1.0f / GYRO_SENS_250DPS;  // Para ±250°/s
    const float deg_to_rad = PI / 180.0f;              // Conversão para rad/s

    // Lê dados brutos do sensor
    mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);
    
    // Converte acelerômetro para g (unidades de gravidade)
    ax = accel_raw[0] * accel_scale;
    ay = accel_raw[1] * accel_scale;
    az = accel_raw[2] * accel_scale;
    
    // Converte giroscópio para rad/s
    gx = gyro_raw[0] * gyro_scale * deg_to_rad;
    gy = gyro_raw[1] * gyro_scale * deg_to_rad;
    gz = gyro_raw[2] * gyro_scale * deg_to_rad;

    // Aplica o filtro de Madgwick
    imu_filter(ax, ay, az, gx, gy, gz);
    
    // Converte quaternion para ângulos de Euler
    eulerAngles(q_est, &g_roll, &g_pitch, &g_yaw);

    // Exibe os resultados
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

// Verifica se a posição é perigosa (pitch > 90 graus)
bool posicaoPerigosa() 
{
    return (g_roll > 90.0f);
}
