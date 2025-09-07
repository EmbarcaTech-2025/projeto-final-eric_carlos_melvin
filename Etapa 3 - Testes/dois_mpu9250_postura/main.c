#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "imu_driver.h"
#include "mpu9250_i2c.h"

 // Estrutura para o primeiro MPU9250
// ESTE SENSOR REPRESENTA O TRONCO
mpu9250_t mpu_0 = {
    .i2c = i2c1,           // Use I2C1
    .sda_gpio = 2,         // SDA on GPIO 2
    .scl_gpio = 3,         // SCL on GPIO 3
    .addr = MPU9250_ADDR_0, // Default I2C address (0x68)
    .id = 1,               // Device ID
    .mag_enabled = false   // Will be set during initialization
};

 // Estrutura para o segundo MPU9250
// ESTE SENSOR REPRESENTA A COXA/PERNA
mpu9250_t mpu_1 = {
    .i2c = i2c1,           // Use I2C1
    .sda_gpio = 2,         // SDA on GPIO 2
    .scl_gpio = 3,         // SCL on GPIO 3
    .addr = MPU9250_ADDR_1, // Default I2C address (0x68)
    .id = 1,               // Device ID
    .mag_enabled = false   // Will be set during initialization
};

//Configuração padrão para o MPU9250
mpu9250_config_t config = {
    .accel_range = MPU9250_ACCEL_RANGE_2G,      // ±2g
    .gyro_range = MPU9250_GYRO_RANGE_250DPS,    // ±250°/s
    .dlpf_filter = MPU9250_DLPF_41HZ,           // 41Hz low-pass filter
    .sample_rate_divider = 9,                    // 100Hz sample rate (1000/(1+9))
    .enable_magnetometer = true                  // Enable magnetometer
};

int main() 
{
    // Inicializa comunicação serial
    stdio_init_all();
    
    // Aguarda tempo maior para estabilizar a comunicação serial USB CDC
    sleep_ms(2000); // Aumentado de 1000 para 2000ms
    
    // Verifica se o sistema foi reiniciado pelo watchdog
    if (watchdog_caused_reboot()) {
        printf("\n\n=== SISTEMA REINICIADO PELO WATCHDOG ===\n");
        printf("Aguardando estabilização da comunicação USB...\n");
        sleep_ms(2000); // Tempo adicional após reset por watchdog
    }

    printf("\n\n");
    printf("========================================\n");
    printf("=== Sistema AHRS com 2 MPU9250 ===\n");
    printf("========================================\n");
    printf("Inicializando sistema...\n");
    
    // Inicializa os drivers do IMU
    imu_init(&mpu_0, &config);
    imu_init(&mpu_1, &config);

    // Inicia a aquisição e processamento dos dados para ambos os sensores
    imu_start_dual_sensors(&mpu_0, &mpu_1);

    //imu_print_hip_angles();

    // Nunca deveria chegar aqui
    return 0;
}