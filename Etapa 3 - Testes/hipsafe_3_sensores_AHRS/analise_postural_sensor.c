#include "drivers/mpu6050/mpu6050_i2c.h"
#include "drivers/filter/MadgwickAHRS.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef PI
#define PI 3.14159265359f
#endif

// Estrutura para quaternion
typedef struct {
    float w, x, y, z;
} quaternion_t;

// Função para converter quaternion global para ângulos de Euler
void quaternion_to_euler(float q0, float q1, float q2, float q3, float *roll, float *pitch, float *yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / PI;

    // Pitch (y-axis rotation)
    float sinp = 2 * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1)
        *pitch = copysignf(90.0f, sinp); // Use 90 degrees if out of range
    else
        *pitch = asinf(sinp) * 180.0f / PI;

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / PI;
}

// Array global para armazenar os últimos ângulos lidos por cada sensor
float g_roll[3] = {0.0f, 0.0f, 0.0f};
float g_pitch[3] = {0.0f, 0.0f, 0.0f};
float g_yaw[3] = {0.0f, 0.0f, 0.0f};

// Arrays para armazenar quaternions individuais de cada sensor (usando variáveis separadas)
// já que o MadgwickAHRS usa variáveis globais q0, q1, q2, q3
float sensor_q0[3] = {1.0f, 1.0f, 1.0f};
float sensor_q1[3] = {0.0f, 0.0f, 0.0f};
float sensor_q2[3] = {0.0f, 0.0f, 0.0f};
float sensor_q3[3] = {0.0f, 0.0f, 0.0f};

// Requisita as posições dos sensores e atualiza os ângulos globais
bool requisitaPosicoes(mpu6050_t *mpu) 
{
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    float ax, ay, az, gx, gy, gz;
    
    // Constantes de conversão
    const float accel_scale = 1.0f / ACCEL_SENS_2G;    // Para ±2g
    const float gyro_scale = 1.0f / GYRO_SENS_250DPS;  // Para ±250°/s
    const float deg_to_rad = PI / 180.0f;              // Conversão para rad/s

    // Lê dados brutos do sensor
    mpu6050_read_raw(mpu, accel_raw, gyro_raw, &temp_raw);
    
    // Converte acelerômetro para g (unidades de gravidade)
    ax = accel_raw[0] * accel_scale;
    ay = accel_raw[1] * accel_scale;
    az = accel_raw[2] * accel_scale;
    
    // Converte giroscópio para rad/s
    gx = gyro_raw[0] * gyro_scale * deg_to_rad;
    gy = gyro_raw[1] * gyro_scale * deg_to_rad;
    gz = gyro_raw[2] * gyro_scale * deg_to_rad;

    // Salva valores atuais dos quaternions globais
    float temp_q0 = q0, temp_q1 = q1, temp_q2 = q2, temp_q3 = q3;
    
    // Restaura quaternions específicos do sensor
    q0 = sensor_q0[mpu->id];
    q1 = sensor_q1[mpu->id];
    q2 = sensor_q2[mpu->id];
    q3 = sensor_q3[mpu->id];

    // Aplica o filtro de Madgwick (usando apenas IMU, sem magnetômetro)
    MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    
    // Salva quaternions atualizados do sensor
    sensor_q0[mpu->id] = q0;
    sensor_q1[mpu->id] = q1;
    sensor_q2[mpu->id] = q2;
    sensor_q3[mpu->id] = q3;
    
    // Restaura quaternions globais (se necessário para outros sensores)
    q0 = temp_q0; q1 = temp_q1; q2 = temp_q2; q3 = temp_q3;
    
    // Converte quaternion para ângulos de Euler
    quaternion_to_euler(sensor_q0[mpu->id], sensor_q1[mpu->id], sensor_q2[mpu->id], sensor_q3[mpu->id], 
                       &g_roll[mpu->id], &g_pitch[mpu->id], &g_yaw[mpu->id]);

    // Exibe os resultados em formato de tabela lado a lado para sensores 0 e 1
    /*
    static int print_header = 1;
    if (mpu->id == 1) {
        if (print_header) {
            printf("\n+---------------------+---------------------+\n");
            printf("|   Sensor 0         |   Sensor 1         |\n");
            printf("+--------+--------+--------+--------+--------+--------+\n");
            printf("| Roll   | Pitch  | Yaw    | Roll   | Pitch  | Yaw    |\n");
            printf("+--------+--------+--------+--------+--------+--------+\n");
            print_header = 0;
        }
        printf("| %6.2f | %6.2f | %6.2f | %6.2f | %6.2f | %6.2f |\n",
            g_roll[0], g_pitch[0], g_yaw[0],
            g_roll[1], g_pitch[1], g_yaw[1]);
    }
    */

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
// Este código existe só para fins de teste.
bool posicaoPerigosa() 
{
    return (g_roll[0] > 90.0f);
}
