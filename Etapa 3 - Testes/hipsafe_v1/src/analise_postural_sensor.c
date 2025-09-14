#include "drivers/mpu6050/mpu6050_i2c.h"
#include "drivers/filter/madgwickFilter.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265359f
#endif

// Array global para armazenar os últimos ângulos lidos por cada sensor
float g_roll[3] = {0.0f, 0.0f, 0.0f};
float g_pitch[3] = {0.0f, 0.0f, 0.0f};
float g_yaw[3] = {0.0f, 0.0f, 0.0f};

typedef struct {
    float x_coord;
    float y_coord;
    float z_coord;
} offset_data_t;

// Array global para armazenar as orientações estimadas calculadas por cada sensor
struct quaternion q_est[3] = {
    {1.0f, 0.0f, 0.0f, 0.0f}, // Sensor 0
    {1.0f, 0.0f, 0.0f, 0.0f}, // Sensor 1
    {1.0f, 0.0f, 0.0f, 0.0f}  // Sensor 2
};

// Estrutura contendo dados de calibração dos giroscopios.
static offset_data_t gyro_offset[3] = {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f}
};

static bool is_stationary[3] = {false, false, false};
static uint8_t stationary_count[3] = {0, 0, 0};
static bool calibration_flags[3] = {false, false, false};
static bool first_run[3] = {true, true, true}; // Flag para verificar se é a primeira execução

// Função para calibrar o giroscópio
void calibrateGyro(mpu6050_t *mpu) 
{
    const int samples = 1000;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    
    for(int i = 0; i < samples; i++) 
    {
        mpu6050_read_raw(mpu, accel_raw, gyro_raw, &temp_raw);
        sum[0] += gyro_raw[0];
        sum[1] += gyro_raw[1];
        sum[2] += gyro_raw[2];
        
        // Pequeno delay entre leituras
        for(volatile int j = 0; j < 10000; j++); // Delay simples
    }
    
    gyro_offset[mpu->id].x_coord = sum[0] / samples;
    gyro_offset[mpu->id].y_coord = sum[1] / samples;
    gyro_offset[mpu->id].z_coord = sum[2] / samples;
    
    calibration_flags[mpu->id] = true;

    printf("Calibração concluída! Offsets: X=%.2f, Y=%.2f, Z=%.2f\n", 
           gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}

bool isStationary(float gx, float gy, float gz) {
    const float threshold = 0.02f; // rad/s - ajuste conforme necessário
    float magnitude = sqrtf(gx*gx + gy*gy + gz*gz);
    return (magnitude < threshold);
}

// Função para resetar o filtro
void resetFilter(mpu6050_t *mpu) 
{
    q_est[mpu->id].q1 = 1.0f;
    q_est[mpu->id].q2 = 0.0f;
    q_est[mpu->id].q3 = 0.0f;
    q_est[mpu->id].q4 = 0.0f;
    calibration_flags[mpu->id] = false; // Força nova calibração
    printf("Filtro resetado!\n");
}


// Função para aplicar filtro passa-baixa simples nos ângulos
void applyLowPassFilter(float* current, float new_value, float alpha) {
    *current = alpha * new_value + (1.0f - alpha) * (*current);
}

// Requisita as posições dos sensores e atualiza os ângulos globais
bool requisitaPosicoes(mpu6050_t *mpu) 
{
    // printf("MPU ID: %d\n", mpu->id);
    if(!calibration_flags[mpu->id]){
        calibrateGyro(mpu);
    }

    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    static float filtered_roll = 0.0f, filtered_pitch = 0.0f, filtered_yaw = 0.0f;
    float ax, ay, az, gx, gy, gz;
    
    // Constantes de conversão
    const float accel_scale = 1.0f / ACCEL_SENS_2G;    // Para ±2g
    const float gyro_scale = 1.0f / GYRO_SENS_250DPS;  // Para ±250°/s
    const float deg_to_rad = PI / 180.0f;              // Conversão para rad/s
    const float filter_alpha = 0.8f; // Fator de suavização do filtro passa-baixa

    // Lê dados brutos do sensor
    mpu6050_read_raw(mpu, accel_raw, gyro_raw, &temp_raw);
    
    // Converte acelerômetro para g (unidades de gravidade)
    ax = accel_raw[0] * accel_scale;
    ay = accel_raw[1] * accel_scale;
    az = accel_raw[2] * accel_scale;
    
    // Converte giroscópio para rad/s
    gx = (gyro_raw[0] - gyro_offset[mpu->id].x_coord) * gyro_scale * deg_to_rad;
    gy = (gyro_raw[1] - gyro_offset[mpu->id].y_coord) * gyro_scale * deg_to_rad;
    gz = (gyro_raw[2] - gyro_offset[mpu->id].z_coord) * gyro_scale * deg_to_rad;

    is_stationary[mpu->id] = isStationary(gx, gy, gz);

    if (is_stationary[mpu->id]) 
    {
        stationary_count[mpu->id]++;
        if (stationary_count[mpu->id] > 10) { // Após ~1 segundo parado (assumindo 100Hz)
            // Força correção do acelerômetro mais forte quando parado
            // Isso ajuda a corrigir a deriva quando não há movimento
            stationary_count[mpu->id] = 10; // Evita overflow
        }
    } else {
        stationary_count[mpu->id] = 0;
    }

    // Aplica o filtro de Madgwick
    imu_filter(&q_est[mpu->id],ax, ay, az, gx, gy, gz);
    
    

    // Converte quaternion para ângulos de Euler
    float temp_roll, temp_pitch, temp_yaw;
    eulerAngles(q_est[mpu->id], &temp_roll, &temp_pitch, &temp_yaw);
    //eulerAngles(q_est[mpu->id], &g_roll[mpu->id], &g_pitch[mpu->id], &g_yaw[mpu->id]);

    // Aplica filtro passa-baixa para suavizar os ângulos
    if (first_run) {
        filtered_roll = temp_roll;
        filtered_pitch = temp_pitch;
        filtered_yaw = temp_yaw;
        first_run[mpu->id] = false;
    } else {
        applyLowPassFilter(&filtered_roll, temp_roll, filter_alpha);
        applyLowPassFilter(&filtered_pitch, temp_pitch, filter_alpha);
        applyLowPassFilter(&filtered_yaw, temp_yaw, filter_alpha);
    }

    g_roll[mpu->id] = filtered_roll;
    g_pitch[mpu->id] = filtered_pitch;
    g_yaw[mpu->id] = filtered_yaw;
    // Exibe os resultados
    // printf("[Valor em todos os roll] Roll 0: %6.2f°, Roll 1 %6.2f°, Roll 2: %6.2f°\n", g_roll[0], g_roll[1], g_roll[2]);
    // printf("-----------------------------------------------------------------------\n");
    printf("[MPU6050 #%d] Roll: %6.0f°, Pitch: %6.0f°, Yaw: %6.0f°\n", mpu->id, g_roll[mpu->id], g_pitch[mpu->id], g_yaw[mpu->id]);

    return true;
}

// Compara posições atuais com limites de segurança
bool comparaPosicoes() 
{
    // Aqui pode-se adicionar outras regras de comparação se necessário
    // Para este exemplo, sempre retorna true
    
    return false;
}

// Verifica se a posição é perigosa (pitch > 90 graus)
// Este código existe só para fins de teste.
bool posicaoPerigosa() 
{
    return (g_roll[0] > 90.0f);
}
