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
FusionAhrs ahrs[NUM_SENSORS];
bool fusion_initialized = false;

// Adicione estas variáveis globais para calibração
CalibrationData accel_cal[NUM_SENSORS] = {0};
CalibrationData gyro_cal[NUM_SENSORS] = {0};

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
    
    // Variáveis estáticas para detectar deriva
    static FusionVector last_gyro[NUM_SENSORS] = {FUSION_VECTOR_ZERO};
    static bool first_read = true;
    
    // Processar dados de cada sensor
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) 
    {
        FusionVector accel_raw, gyro_raw;
        
        // Converter para unidades físicas
        convert_mpu6050_data(accel_data[sensor], gyro_data[sensor], &accel_raw, &gyro_raw, sensor);
        
        // Detectar se o sensor está realmente parado (baixa atividade no giroscópio)
        float gyro_magnitude = FusionVectorMagnitude(gyro_raw);
        bool sensor_stationary = (gyro_magnitude < 0.5f); // Limiar baixo para detectar parado
        
        // Se calibrado, aplicar calibração do giroscópio
        FusionVector gyro_calibrated = apply_calibration(gyro_raw, &gyro_cal[sensor]);
        
        // Se o sensor está parado há muito tempo, zerar entrada do giroscópio para evitar deriva
        if (sensor_stationary && gyro_cal[sensor].calibrated) {
            // Aplicar deadband mais agressivo para sensor parado
            if (fabs(gyro_calibrated.axis.x) < 0.1f) gyro_calibrated.axis.x = 0.0f;
            if (fabs(gyro_calibrated.axis.y) < 0.1f) gyro_calibrated.axis.y = 0.0f;
            if (fabs(gyro_calibrated.axis.z) < 0.1f) gyro_calibrated.axis.z = 0.0f;
        }
        
        // Acelerômetro sem calibração para preservar orientação
        FusionVector accel_uncalibrated = accel_raw;
        
        // Atualizar AHRS (sem magnetômetro)
        FusionAhrsUpdateNoMagnetometer(&ahrs[sensor], gyro_calibrated, accel_uncalibrated, SAMPLE_PERIOD);
        
        // Obter orientação em ângulos de Euler
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs[sensor]));
        
        // Atualizar variáveis globais
        g_roll[sensor] = euler.angle.roll;
        g_pitch[sensor] = euler.angle.pitch;
        g_yaw[sensor] = euler.angle.yaw;
        
        // Armazenar última leitura para próxima iteração
        last_gyro[sensor] = gyro_raw;
    }
    
    first_read = false;
    
    // Exibir resultados apenas a cada 50 amostras (reduz spam no terminal)
    static int display_counter = 0;
    if (++display_counter >= 50) {
        // Calcular ângulos relativos apenas entre sensores 0 e 1
        float roll_rel_01 = g_roll[1] - g_roll[0];    // Sensor 1 relativo ao Sensor 0
        float pitch_rel_01 = g_pitch[1] - g_pitch[0];
        float yaw_rel_01 = normalize_angle(g_yaw[1] - g_yaw[0]);
        
        // Calcular ângulo de flexão do quadril (valor absoluto da diferença de pitch)
        float flexao_quadril = fabs(pitch_rel_01);
        
        printf("\n================= ÂNGULOS ABSOLUTOS =================");
        printf("\n+----------------------+----------------------+");
        printf("\n|  Sensor 0 (Tronco)  |  Sensor 1 (Perna)   |");
        printf("\n+----------------------+----------------------+");
        printf("\n| Roll : %7.2f°      | Roll : %7.2f°      |", g_roll[0], g_roll[1]);
        printf("\n| Pitch: %7.2f°      | Pitch: %7.2f°      |", g_pitch[0], g_pitch[1]);
        printf("\n| Yaw  : %7.2f°      | Yaw  : %7.2f°      |", g_yaw[0], g_yaw[1]);
        printf("\n+----------------------+----------------------+");
        
        printf("\n\n================= ANÁLISE POSTURAL ==================");
        printf("\n+----------------------+----------------------+");
        printf("\n|  Flexão de Quadril   |    Status Postural  |");
        printf("\n+----------------------+----------------------+");
        printf("\n|      %7.2f°        |", flexao_quadril);
        
        if (flexao_quadril <= 50.0f) {
            printf("       NORMAL       |");
        } else {
            printf("     ⚠️ CURVADO     |");
        }
        
        printf("\n+----------------------+----------------------+");
        
        printf("\n\n================= ÂNGULOS RELATIVOS =================");
        printf("\n+----------------------+");
        printf("\n|   Perna - Tronco    |");
        printf("\n+----------------------+");
        printf("\n| Roll : %7.2f°      |", roll_rel_01);
        printf("\n| Pitch: %7.2f°      |", pitch_rel_01);
        printf("\n| Yaw  : %7.2f°      |", yaw_rel_01);
        printf("\n+----------------------+\n");
        
        display_counter = 0;
    }
    
    return true;
}

// Compara posições atuais com limites de segurança
bool comparaPosicoes() 
{
    // Aqui pode-se adicionar outras regras de comparação se necessário
    // Para este exemplo, sempre retorna true
    return true;
}

// Verifica se a posição é perigosa (flexão de quadril > 50 graus)
bool posicaoPerigosa() 
{
    // Calcular diferenças relativas entre sensores 0 (quadril/tronco) e 1 (perna)
    float roll_rel, pitch_rel, yaw_rel;
    
    calculate_relative_angles(0, 1, &roll_rel, &pitch_rel, &yaw_rel);
    
    // Calcular o ângulo de flexão do quadril
    // O ângulo de flexão é a diferença de pitch entre tronco e perna
    float flexao_quadril = fabs(pitch_rel);
    
    // Se a flexão for maior que 50 graus, é posição perigosa (pessoa muito curvada)
    if (flexao_quadril > 50.0f) {
        printf("ALERTA: Flexão de quadril perigosa: %.1f° (> 50°) - Pessoa muito curvada!\n", 
               flexao_quadril);
        printf("  Sensor 0 (tronco): Pitch=%.1f°\n", g_pitch[0]);
        printf("  Sensor 1 (perna):  Pitch=%.1f°\n", g_pitch[1]);
        printf("  Diferença:         %.1f°\n", pitch_rel);
        return true;
    }
    
    return false;
}

// Modifique a função apply_calibration para não zerar a orientação inicial
FusionVector apply_calibration(FusionVector raw_data, CalibrationData *cal) {
    if (!cal->calibrated) {
        return raw_data; // Retorna dados brutos se não calibrado
    }
    
    // Aplica apenas correção de offset (bias)
    FusionVector corrected = FusionVectorSubtract(raw_data, cal->offset);
    
    return corrected;
}

// Função auxiliar para normalizar ângulos no range [-180, 180]
float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// Função para calcular ângulos relativos entre dois sensores
void calculate_relative_angles(int sensor_ref, int sensor_target, float *roll_rel, float *pitch_rel, float *yaw_rel) {
    if (sensor_ref >= NUM_SENSORS || sensor_target >= NUM_SENSORS || sensor_ref < 0 || sensor_target < 0) {
        *roll_rel = *pitch_rel = *yaw_rel = 0.0f;
        return;
    }
    
    *roll_rel = g_roll[sensor_target] - g_roll[sensor_ref];
    *pitch_rel = g_pitch[sensor_target] - g_pitch[sensor_ref];
    *yaw_rel = normalize_angle(g_yaw[sensor_target] - g_yaw[sensor_ref]);
}

// Função para obter todos os ângulos relativos de uma vez
void get_all_relative_angles(float relative_angles[NUM_SENSORS-1][NUM_SENSORS-1][3]) {
    // relative_angles[i][j][0] = roll, [1] = pitch, [2] = yaw
    // Sensor (i+1) relativo ao sensor j
    
    for (int ref = 0; ref < NUM_SENSORS-1; ref++) {
        for (int target = ref+1; target < NUM_SENSORS; target++) {
            int idx_ref = ref;
            int idx_target = target - ref - 1;
            
            calculate_relative_angles(ref, target, 
                                    &relative_angles[idx_target][idx_ref][0],
                                    &relative_angles[idx_target][idx_ref][1], 
                                    &relative_angles[idx_target][idx_ref][2]);
        }
    }
}

// Modifique a inicialização dos sistemas Fusion
void initialize_all_fusion_systems(void) 
{
    printf("Configurando sistemas Fusion AHRS para %d sensores...\n", NUM_SENSORS);
    
    for (int i = 0; i < NUM_SENSORS; i++) 
    {
        // Inicializar AHRS
        FusionAhrsInitialise(&ahrs[i]);
        
        // Configurar settings com ganho muito baixo para estabilidade máxima
        FusionAhrsSettings settings = {
            .convention = FusionConventionNwu,  // North-West-Up
            .gain = 0.01f,  // GANHO EXTREMAMENTE BAIXO para evitar deriva
            .gyroscopeRange = 500.0f,  // Conforme configuração do MPU (±500°/s)
            .accelerationRejection = 100.0f,  // Máximo para manter orientação estável
            .magneticRejection = 100.0f,
            .recoveryTriggerPeriod = 20 * SAMPLE_RATE  // 20 segundos
        };
        
        FusionAhrsSetSettings(&ahrs[i], &settings);
        
        printf("Sistema Fusion %d configurado com ganho baixo!\n", i);
    }
    
    fusion_initialized = true;
    printf("Todos os sistemas Fusion inicializados com sucesso!\n");
}

// Nova função para calibração rápida do giroscópio (apenas bias)
void quick_gyroscope_calibration(void) 
{
    printf("Iniciando calibração rápida do giroscópio...\n");
    printf("IMPORTANTE: Mantenha os sensores COMPLETAMENTE PARADOS por 3 segundos!\n");
    
    const int calibration_samples = 300; // 3 segundos a 100Hz
    FusionVector gyro_sum[NUM_SENSORS] = {FUSION_VECTOR_ZERO};
    int valid_samples = 0;
    
    for (int sample = 0; sample < calibration_samples; sample++) 
    {
        int16_t accel_data[3][3], gyro_data[3][3], temp_data[3];
        
        if (mpu6050_read_all_sensors(accel_data, gyro_data, temp_data)) 
        {
            for (int sensor = 0; sensor < NUM_SENSORS; sensor++) 
            {
                FusionVector accel_raw, gyro_raw;
                convert_mpu6050_data(accel_data[sensor], gyro_data[sensor], &accel_raw, &gyro_raw, sensor);
                
                gyro_sum[sensor] = FusionVectorAdd(gyro_sum[sensor], gyro_raw);
            }
            valid_samples++;
        }
        sleep_ms(10);
    }
    
    // Calcular offset do giroscópio (bias de deriva)
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) 
    {
        gyro_cal[sensor].offset = FusionVectorMultiplyScalar(gyro_sum[sensor], 1.0f / valid_samples);
        gyro_cal[sensor].sensitivity = FUSION_VECTOR_ONES;
        gyro_cal[sensor].misalignment = FUSION_IDENTITY_MATRIX;
        gyro_cal[sensor].calibrated = true;
        
        // Não calibramos o acelerômetro para preservar orientação inicial
        accel_cal[sensor].offset = FUSION_VECTOR_ZERO;
        accel_cal[sensor].sensitivity = FUSION_VECTOR_ONES;
        accel_cal[sensor].misalignment = FUSION_IDENTITY_MATRIX;
        accel_cal[sensor].calibrated = false; // Acelerômetro não calibrado
        
        printf("Sensor %d - Gyro bias: [%.4f, %.4f, %.4f] °/s\n",
               sensor,
               gyro_cal[sensor].offset.axis.x, 
               gyro_cal[sensor].offset.axis.y, 
               gyro_cal[sensor].offset.axis.z);
    }
    
    printf("Calibração do giroscópio concluída!\n");
}

// Nova função para inicialização completa com estabilização
void initialize_all_fusion_systems_stable(void) 
{
    // 1. Inicializar sistemas Fusion
    initialize_all_fusion_systems();
    
    // 2. Fazer calibração rápida do giroscópio
    quick_gyroscope_calibration();
    
    // 3. Período de estabilização (permite que o filtro Fusion se estabilize)
    printf("Aguardando estabilização do filtro AHRS (5 segundos)...\n");
    
    for (int i = 0; i < (int)(5.0f * SAMPLE_RATE); i++) {  // 5 segundos
        int16_t accel_data[3][3], gyro_data[3][3], temp_data[3];
        
        if (mpu6050_read_all_sensors(accel_data, gyro_data, temp_data)) {
            for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
                FusionVector accel_raw, gyro_raw;
                convert_mpu6050_data(accel_data[sensor], gyro_data[sensor], &accel_raw, &gyro_raw, sensor);
                
                // Aplicar calibração do giroscópio
                FusionVector gyro_calibrated = apply_calibration(gyro_raw, &gyro_cal[sensor]);
                
                // Atualiza AHRS para estabilização
                FusionAhrsUpdateNoMagnetometer(&ahrs[sensor], gyro_calibrated, accel_raw, SAMPLE_PERIOD);
            }
        }
        sleep_ms(10);
    }
    
    printf("Sistemas estabilizados e prontos para uso!\n");
}