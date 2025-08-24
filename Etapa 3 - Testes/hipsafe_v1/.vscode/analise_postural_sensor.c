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

// Variáveis globais para armazenar os últimos ângulos lidos de cada sensor
float g_roll[NUM_SENSORS] = {0.0f};
float g_pitch[NUM_SENSORS] = {0.0f}; 
float g_yaw[NUM_SENSORS] = {0.0f};

// Variáveis globais para Fusion (um AHRS para cada sensor)
CalibrationData accel_cal[NUM_SENSORS] = {0};
CalibrationData gyro_cal[NUM_SENSORS] = {0};
FusionAhrs ahrs[NUM_SENSORS];
bool fusion_initialized = false;

// Função para converter dados brutos do MPU6050 para unidades físicas
void convert_mpu6050_data(int16_t raw_accel[3], int16_t raw_gyro[3], FusionVector *accel, FusionVector *gyro, int sensor_id) 
{
    // Obter configurações de range do sensor específico
    uint8_t accel_range = mpu6050_get_accel_range(&mpu_sensors[sensor_id]);
    uint8_t gyro_range = mpu6050_get_gyro_range(&mpu_sensors[sensor_id]);
    
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
    
    return read_all_sensor_positions();
}

// Lê as posições de todos os sensores
bool read_all_sensor_positions(void) 
{
    int16_t accel_data[3][3], gyro_data[3][3], temp_data[3];
    
    // Ler dados de todos os sensores
    if (!mpu6050_read_all_sensors(accel_data, gyro_data, temp_data)) {
        printf("Erro ao ler sensores!\n");
        return false;
    }
    
    // Processar dados de cada sensor
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) 
    {
        FusionVector accel_raw, gyro_raw;
        
        // Converter para unidades físicas
        convert_mpu6050_data(accel_data[sensor], gyro_data[sensor], &accel_raw, &gyro_raw, sensor);
        
        // Aplicar calibração
        FusionVector accel_calibrated = apply_calibration(accel_raw, &accel_cal[sensor]);
        FusionVector gyro_calibrated = apply_calibration(gyro_raw, &gyro_cal[sensor]);
        
        // Atualizar AHRS (sem magnetômetro)
        FusionAhrsUpdateNoMagnetometer(&ahrs[sensor], gyro_calibrated, accel_calibrated, SAMPLE_PERIOD);
        
        // Obter orientação em ângulos de Euler
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs[sensor]));
        
        // Atualizar variáveis globais
        g_roll[sensor] = euler.angle.roll;
        g_pitch[sensor] = euler.angle.pitch;
        g_yaw[sensor] = euler.angle.yaw;
    }
    
    // Exibir resultados em formato de tabela
    printf("+----------------------+----------------------+");
    printf("\n|     Sensor 0        |     Sensor 1        |");
    printf("\n+----------------------+----------------------+");
    printf("\n| Roll : %7.2f°      | Roll : %7.2f°      |", g_roll[0], g_roll[1]);
    printf("\n| Pitch: %7.2f°      | Pitch: %7.2f°      |", g_pitch[0], g_pitch[1]);
    printf("\n| Yaw  : %7.2f°      | Yaw  : %7.2f°      |", g_yaw[0], g_yaw[1]);
    printf("\n+----------------------+----------------------+\n");
    // Para exibir o sensor 2, descomente abaixo:
    // printf("Sensor 2: Roll:%6.2f°, Pitch:%6.2f°, Yaw:%6.2f°\n", g_roll[2], g_pitch[2], g_yaw[2]);
    
    return true;
}

// Compara posições atuais com limites de segurança
bool comparaPosicoes() 
{
    // Aqui pode-se adicionar outras regras de comparação se necessário
    // Para este exemplo, sempre retorna true
    return true;
}

// Verifica se a posição é perigosa (qualquer sensor com pitch > 90 graus)
bool posicaoPerigosa() 
{
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (g_pitch[i] > 90.0f) {
            printf("Sensor %d detectou posição perigosa: Pitch = %.2f°\n", i, g_pitch[i]);
            return true;
        }
    }
    return false;
}

// Calibra todos os sensores
void calibrate_all_sensors(void) 
{
    printf("Iniciando calibração de TODOS os %d sensores...\n", NUM_SENSORS);
    printf("Mantenha TODOS os sensores IMÓVEIS durante a calibração!\n");
    
    // Som de início da calibração (3 beeps curtos e rápidos)
    printf("Emitindo sinal sonoro de início...\n");
    // Note: Esta função precisa ser definida no main
    
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) 
    {
        printf("Calibrando sensor %d...\n", sensor);
        
        FusionVector accel_sum = {0};
        FusionVector gyro_sum = {0};
        
        // Coletar amostras para calibração
        for (int i = 0; i < 1000; i++) // CALIBRATION_SAMPLES = 1000
        {
            int16_t raw_accel[3], raw_gyro[3], temp;
            FusionVector accel, gyro;
            
            mpu6050_read_raw(&mpu_sensors[sensor], raw_accel, raw_gyro, &temp);
            convert_mpu6050_data(raw_accel, raw_gyro, &accel, &gyro, sensor);
            
            // Acumular valores
            accel_sum = FusionVectorAdd(accel_sum, accel);
            gyro_sum = FusionVectorAdd(gyro_sum, gyro);
            
            // Indicador de progresso
            if ((i + 1) % 200 == 0) 
            {
                printf("  Sensor %d: %d/1000 amostras\n", sensor, i + 1);
            }
            
            sleep_ms(10); // 100Hz
        }
        
        // Calcular offsets médios
        float scale = 1.0f / 1000.0f;
        gyro_cal[sensor].offset = FusionVectorMultiplyScalar(gyro_sum, scale);
        
        // Para o acelerômetro, assumir que Z = 1g quando estático
        accel_cal[sensor].offset.axis.x = accel_sum.axis.x / 1000.0f;
        accel_cal[sensor].offset.axis.y = accel_sum.axis.y / 1000.0f;
        accel_cal[sensor].offset.axis.z = (accel_sum.axis.z / 1000.0f) - 1.0f; // Compensar gravidade
        
        // Inicializar sensibilidade como identidade
        accel_cal[sensor].sensitivity = (FusionVector){.axis = {1.0f, 1.0f, 1.0f}};
        gyro_cal[sensor].sensitivity = (FusionVector){.axis = {1.0f, 1.0f, 1.0f}};
        
        // Inicializar matriz de desalinhamento como identidade
        accel_cal[sensor].misalignment = FUSION_IDENTITY_MATRIX;
        gyro_cal[sensor].misalignment = FUSION_IDENTITY_MATRIX;
        
        accel_cal[sensor].calibrated = true;
        gyro_cal[sensor].calibrated = true;
        
        printf("Sensor %d calibrado!\n", sensor);
        printf("  Gyro offset: X=%.3f, Y=%.3f, Z=%.3f deg/s\n", 
               gyro_cal[sensor].offset.axis.x, gyro_cal[sensor].offset.axis.y, gyro_cal[sensor].offset.axis.z);
        printf("  Accel offset: X=%.3f, Y=%.3f, Z=%.3f g\n", 
               accel_cal[sensor].offset.axis.x, accel_cal[sensor].offset.axis.y, accel_cal[sensor].offset.axis.z);
    }
    
    printf("Calibração de todos os sensores concluída!\n");
}

// Inicializa todos os sistemas Fusion AHRS
void initialize_all_fusion_systems(void) 
{
    printf("Configurando sistemas Fusion AHRS para %d sensores...\n", NUM_SENSORS);
    
    for (int i = 0; i < NUM_SENSORS; i++) 
    {
        // Inicializar AHRS
        FusionAhrsInitialise(&ahrs[i]);
        
        // Configurar parâmetros otimizados para aplicação na perna
        FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,       // North-West-Up
            .gain = 0.5f,                           // Ganho moderado para estabilidade
            .gyroscopeRange = 2000.0f,              // Range máximo do giroscópio em deg/s
            .accelerationRejection = 10.0f,         // Rejeição de aceleração (movimentos)
            .magneticRejection = 0.0f,              // Não usado (sem magnetômetro)
            .recoveryTriggerPeriod = (unsigned int)(5 * SAMPLE_RATE) // Período de recuperação (5 segundos)
        };
        
        FusionAhrsSetSettings(&ahrs[i], &settings);
        printf("AHRS %d configurado\n", i);
    }
    
    fusion_initialized = true;
    
    printf("Todos os sistemas AHRS configurados!\n");
    printf("  - Ganho: %.1f\n", 0.5f);
    printf("  - Rejeição de aceleração: %.1f\n", 10.0f);
    printf("  - Taxa de amostragem: %.1f Hz\n", SAMPLE_RATE);
}