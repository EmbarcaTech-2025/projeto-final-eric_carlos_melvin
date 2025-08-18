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

// Variáveis para calibração do giroscópio
static float gyro_offset[3] = {0.0f, 0.0f, 0.0f};
static bool calibrated = false;

// Função para calibrar o giroscópio
void calibrateGyro() 
{
    const int samples = 1000;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    
    for(int i = 0; i < samples; i++) 
    {
        mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);
        sum[0] += gyro_raw[0];
        sum[1] += gyro_raw[1];
        sum[2] += gyro_raw[2];
        
        // Pequeno delay entre leituras (ajuste conforme seu sistema)
        for(volatile int j = 0; j < 10000; j++); // Delay simples
    }
    
    gyro_offset[0] = sum[0] / samples;
    gyro_offset[1] = sum[1] / samples;
    gyro_offset[2] = sum[2] / samples;
    
    calibrated = true;
    printf("Calibração concluída! Offsets: X=%.2f, Y=%.2f, Z=%.2f\n", 
           gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}

// Função para resetar o filtro
void resetFilter() 
{
    extern struct quaternion q_est;
    q_est.q1 = 1.0f;
    q_est.q2 = 0.0f;
    q_est.q3 = 0.0f;
    q_est.q4 = 0.0f;
    calibrated = false; // Força nova calibração
    printf("Filtro resetado!\n");
}

// Detecta se o sensor está parado (baixa atividade do giroscópio)
bool isStationary(float gx, float gy, float gz) {
    const float threshold = 0.02f; // rad/s - ajuste conforme necessário
    float magnitude = sqrtf(gx*gx + gy*gy + gz*gz);
    return (magnitude < threshold);
}

// Função para aplicar filtro passa-baixa simples nos ângulos
void applyLowPassFilter(float* current, float new_value, float alpha) {
    *current = alpha * new_value + (1.0f - alpha) * (*current);
}

// Requisita as posições dos sensores e atualiza os ângulos globais
bool requisitaPosicoes() 
{
    // Calibra o giroscópio na primeira execução
    if (!calibrated) 
    {
        calibrateGyro();
    }
    
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    float ax, ay, az, gx, gy, gz;
    static float filtered_roll = 0.0f, filtered_pitch = 0.0f, filtered_yaw = 0.0f;
    static bool first_run = true;
    
    // Constantes de conversão
    const float accel_scale = 1.0f / ACCEL_SENS_2G;    // Para ±2g
    const float gyro_scale = 1.0f / GYRO_SENS_250DPS;  // Para ±250°/s
    const float deg_to_rad = PI / 180.0f;              // Conversão para rad/s
    const float filter_alpha = 0.8f;                   // Fator do filtro passa-baixa (0.0 a 1.0)

    // Lê dados brutos do sensor
    mpu6050_read_raw(accel_raw, gyro_raw, &temp_raw);
    
    // Converte acelerômetro para g (unidades de gravidade)
    ax = accel_raw[0] * accel_scale;
    ay = accel_raw[1] * accel_scale;
    az = accel_raw[2] * accel_scale;
    
    // Converte giroscópio para rad/s e aplica calibração
    gx = (gyro_raw[0] - gyro_offset[0]) * gyro_scale * deg_to_rad;
    gy = (gyro_raw[1] - gyro_offset[1]) * gyro_scale * deg_to_rad;
    gz = (gyro_raw[2] - gyro_offset[2]) * gyro_scale * deg_to_rad;

    // Verifica se o sensor está parado
    bool is_stationary = isStationary(gx, gy, gz);
    
    // Se estiver parado por muito tempo, aplique correção adicional
    static int stationary_count = 0;
    if (is_stationary) 
    {
        stationary_count++;
        if (stationary_count > 100) { // Após ~1 segundo parado (assumindo 100Hz)
            // Força correção do acelerômetro mais forte quando parado
            // Isso ajuda a corrigir a deriva quando não há movimento
            stationary_count = 100; // Evita overflow
        }
    } else {
        stationary_count = 0;
    }

    // Aplica o filtro de Madgwick
    imu_filter(ax, ay, az, gx, gy, gz);
    
    // Converte quaternion para ângulos de Euler
    float temp_roll, temp_pitch, temp_yaw;
    eulerAngles(q_est, &temp_roll, &temp_pitch, &temp_yaw);

    // Aplica filtro passa-baixa para suavizar os ângulos
    if (first_run) {
        filtered_roll = temp_roll;
        filtered_pitch = temp_pitch;
        filtered_yaw = temp_yaw;
        first_run = false;
    } else {
        applyLowPassFilter(&filtered_roll, temp_roll, filter_alpha);
        applyLowPassFilter(&filtered_pitch, temp_pitch, filter_alpha);
        applyLowPassFilter(&filtered_yaw, temp_yaw, filter_alpha);
    }

    // Atualiza variáveis globais com valores filtrados
    g_roll = filtered_roll;
    g_pitch = filtered_pitch;
    g_yaw = filtered_yaw;

    // Exibe os resultados
    printf("Roll: %6.2f°, Pitch: %6.2f°, Yaw: %6.2f° %s\n", 
           g_roll, g_pitch, g_yaw, is_stationary ? "[PARADO]" : "");
    
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
    return (fabsf(g_roll) > 90.0f);
}