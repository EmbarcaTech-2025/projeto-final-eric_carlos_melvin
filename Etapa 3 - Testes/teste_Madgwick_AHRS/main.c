#include <stdio.h>
#include "pico/stdlib.h"
#include "mpu6050_i2c.h"
#include "MadgwickAHRS.h"
#include <math.h>
#include "pico/multicore.h"

float roll = 0.0, pitch=0.0, yaw=0.0; // Variáveis globais para armazenar os ângulos de Euler
struct repeating_timer timer;
struct repeating_timer timer2;
volatile bool sample_mpu6050 = false; // Flag para indicar quando amostrar o MPU
volatile bool print_mpu6050 = false;  // Flag para indicar quando imprimir os dados do MPU

bool repeating_timer_callback(struct repeating_timer *t) {
    sample_mpu6050 = true;
    return true; // Return true to continue the timer, false to stop
}

bool repeating_timer_callback2(struct repeating_timer *t){
    print_mpu6050 = true;
    return true;
}

void core1_entry() 
{
    int32_t delay_us_print = 10000;
    if (!add_repeating_timer_us(delay_us_print, repeating_timer_callback2, NULL, &timer2)) {
        printf("Failed to add repeating timer\n");
        return 1;
    }
    
    // Esta função pode ser usada para tarefas adicionais no segundo núcleo, se necessário
    while (1) 
    {
        // Placeholder - nenhuma tarefa definida
        if(print_mpu6050 == true){
            printf("Roll: %6.2f°, Pitch: %6.2f°, Yaw: %6.2f°\n", roll, pitch, yaw);
            print_mpu6050 = false;
        }
    }
}

int main() 
{
    stdio_init_all();           // Inicializa USB serial
    mpu6050_setup_i2c();       // Configura barramento I2C
    mpu6050_reset();           // Reinicia o sensor
    
    // Configure o sensor para ±2g (mais sensível) e ±250°/s
    mpu6050_set_accel_range(0); // 0 = ±2g
    mpu6050_set_gyro_range(0);  // 0 = ±250°/s
    
    int16_t accel_raw[3], gyro_raw[3], temp_raw;
    float ax, ay, az, gx, gy, gz;

    int32_t delay_us = 1000; // 1000us = 1ms → 1000Hz sampling rate

    if (!add_repeating_timer_us(delay_us, repeating_timer_callback, NULL, &timer)) {
        printf("Failed to add repeating timer\n");
        return 1;
    }
    
    // Constantes de conversão
    const float accel_scale = 1.0f / ACCEL_SENS_2G;    // Para ±2g
    const float gyro_scale = 1.0f / GYRO_SENS_250DPS;  // Para ±250°/s
    const float deg_to_rad = 3.14159265358979f / 180.0f; // Conversão para rad/s
    const float rad_to_deg = 180.0f / 3.14159265358979f; // Conversão para graus
    
    printf("Sistema iniciado - Filtro de Madgwick com MPU6050\n");
    printf("Aguarde alguns segundos para estabilização...\n\n");

    multicore_launch_core1(core1_entry); // Inicia o segundo núcleo para exibição dos dados

    // Loop principal com frequência de ~100Hz (10ms)
    while (1) 
    {
        if(sample_mpu6050 == true){
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
            
            // Aplica o filtro de Madgwick (versão IMU sem magnetômetro)
            MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
            
            // Calcula ângulos de Euler a partir dos quaternions globais q0, q1, q2, q3
            yaw = (atan2f(2.0f * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3))*rad_to_deg;
            pitch = (-asinf(2.0f * (q1*q3 - q0*q2)))*rad_to_deg;
            roll = (atan2f(2.0f * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3))*rad_to_deg;

            sample_mpu6050 = false;
        }
    }
}