#include "imu_driver.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "mpu9250_i2c.h"
#include "MadgwickAHRS.h" // Para o uso das funções e estruturas do Madgwick
#include "calibrate.h" // Garante que o header correto está incluso
#include <math.h>

#include "algoritmo_postura.h"
#include "pico/multicore.h"


// Valores de calibração obtidos da rotina de calibração
static const calibration_t sensor_calibration = {
    .mag_offset = {.x = 46.259476, .y = 15.621095, .z = -9.516212},
    .mag_scale = {.x = 0.514466, .y = 1.523512, .z = 2.500883},
    .accel_offset = {.x = -0.011733, .y = -0.025583, .z = -0.096481},
    .accel_scale_lo = {.x = -1.015413, .y = 1.000058, .z = 0.897678},
    .accel_scale_hi = {.x = 0.992299, .y = -0.998858, .z = -1.113556},
    .gyro_bias_offset = {.x = -1.344947, .y = 1.100829, .z = 1.126400}
};

// Inicialização das estruturas de dados para cada sensor
static sensor_data_t sensor_0 = {0};
static sensor_data_t sensor_1 = {0};
static mpu9250_t *global_mpu0 = NULL;
static mpu9250_t *global_mpu1 = NULL;

// Inicialização dos temporizadores para amostragem periódica e para leitura de dados
static struct repeating_timer timer;
static struct repeating_timer timer2;
static volatile bool sample_mpu9250 = false; // Flag para indicar quando amostrar o MPU
static volatile bool print_mpu9250 = false;  // Flag para indicar quando imprimir os dados do MPU

// * Constantes de conversão
static const float accel_scale = 1.0f / ACCEL_SENS_2G;    // Para ±2g
static const float gyro_scale = 1.0f / GYRO_SENS_250DPS;  // Para ±250°/s
static const float deg_to_rad = 3.14159265358979f / 180.0f; // Conversão para rad/s
static const float RAD_TO_DEG_CONST = 180.0f / 3.14159265358979f; // Conversão para graus

// Função para imprimir ângulos do quadril e identificar posturas perigosas
void imu_print_hip_angles(void)
{
    if (!sensor_0.initialized || !sensor_1.initialized) {
        printf("Sensores não inicializados ainda\n");
        return;
    }

    Quaternion q_tronco = { sensor_0.ahrs.orientation.q0, sensor_0.ahrs.orientation.q1, sensor_0.ahrs.orientation.q2, sensor_0.ahrs.orientation.q3 };
    Quaternion q_coxa   = { sensor_1.ahrs.orientation.q0, sensor_1.ahrs.orientation.q1, sensor_1.ahrs.orientation.q2, sensor_1.ahrs.orientation.q3 };
    Quaternion q_rel = relative_quaternion(q_tronco, q_coxa);

    float flexao, aducao, rotacao;
    bool rotacao_interna_30, flexao_maior_90, cruzamento_pernas;
    hip_angles(q_rel, &flexao, &aducao, &rotacao, &rotacao_interna_30, &flexao_maior_90, &cruzamento_pernas);

    float flexao_deg = flexao * 180.0f / 3.14159265358979f;
    float aducao_deg = aducao * 180.0f / 3.14159265358979f;
    float rotacao_deg = rotacao * 180.0f / 3.14159265358979f;

    printf("Quadril: Flexao = %6.2f° | Adução = %6.2f° | Rotacao = %6.2f°\n", flexao_deg, aducao_deg, rotacao_deg);

    if (rotacao_interna_30)
        printf("[ALERTA] Rotação interna maior que 30 graus!\n");
    if (flexao_maior_90)
        printf("[ALERTA] Flexão de quadril maior que 90 graus!\n");
    if (cruzamento_pernas)
        printf("[ALERTA] Cruzamento entre as pernas (adução excessiva)!\n");
}

// Callback do timer para amostragem
static bool sampling_timer_callback(struct repeating_timer *t) 
{
    sample_mpu9250 = true;
    return true;
}

// Callback do timer para impressão
static bool print_timer_callback(struct repeating_timer *t) 
{
    print_mpu9250 = true;
    return true;
}

// Função do núcleo 1 - responsável pela impressão dos dados
static void core1_entry(void) 
{
    int32_t delay_us_print = PRINT_PERIOD_US; // 10ms (mesma velocidade do código anterior)
    if (!add_repeating_timer_us(delay_us_print, print_timer_callback, NULL, &timer2)) 
    {
        printf("Failed to add repeating timer for printing\n");
        return;
    }
    
    while (1)  // Se teve sucesso, entra no loop infinito de impressão de dados.
    {
        if (print_mpu9250 == true) 
        {
            //printf("MPU0: Roll=%6.2f° Pitch=%6.2f° Yaw=%6.2f° | MPU1: Roll=%6.2f° Pitch=%6.2f° Yaw=%6.2f°\n", 
            //       sensor_0.roll, sensor_0.pitch, sensor_0.yaw,
             //      sensor_1.roll, sensor_1.pitch, sensor_1.yaw);
            
            // Imprime também os ângulos de flexão e rotação do quadril
            imu_print_hip_angles();

            print_mpu9250 = false;
        }
    }
}

// Processa os dados do IMU para um sensor específico
/**
 * @brief Processa os dados do IMU escolhido, aplicando a calibração e o filtro de Madgwick.
 * 
 * @param mpu Estrutura de dados contendo informações do MPU9250
 * @param sensor Estrutura de dados contendo informações relevantes para o uso de um sensor inercial
 */
static void process_imu_data(mpu9250_t *mpu, sensor_data_t *sensor) 
{
    // Inicializa as variáveis temporárias
    mpu9250_raw_data_t raw_data;
    mpu9250_data_t data;

    mpu9250_read_raw(mpu, &raw_data); // Lê dados brutos do sensor (única leitura)
    
    sensor_watchdog_feed(mpu->id, &raw_data); // Alimenta o watchdog com dados brutos para detecção de travamento

    // Converte dados brutos para unidades físicas
    data.accel[0] = (float)raw_data.accel[0] / mpu->accel_sensitivity;
    data.accel[1] = (float)raw_data.accel[1] / mpu->accel_sensitivity;
    data.accel[2] = (float)raw_data.accel[2] / mpu->accel_sensitivity;
    data.gyro[0] = (float)raw_data.gyro[0] / mpu->gyro_sensitivity;
    data.gyro[1] = (float)raw_data.gyro[1] / mpu->gyro_sensitivity;
    data.gyro[2] = (float)raw_data.gyro[2] / mpu->gyro_sensitivity;
    data.mag[0] = (float)raw_data.mag[0] * mpu->mag_asa[0] * MAG_SENS;
    data.mag[1] = (float)raw_data.mag[1] * mpu->mag_asa[1] * MAG_SENS;
    data.mag[2] = (float)raw_data.mag[2] * mpu->mag_asa[2] * MAG_SENS;
    
    // Aplica calibração, chamando a função do arquivo calibrate.c
    apply_calibration(&data, &sensor_calibration);

    // Armazena os dados calibrados em variáveis locais
    float gx = data.gyro[0] * deg_to_rad;
    float gy = data.gyro[1] * deg_to_rad;
    float gz = data.gyro[2] * deg_to_rad;
    float ax = data.accel[0];
    float ay = data.accel[1];
    float az = data.accel[2];
    float mx = data.mag[0];
    float my = data.mag[1];
    float mz = data.mag[2];
    
    // Inicialização inteligente do quaternion na primeira iteração
    if (!sensor->initialized) 
    {
        // Estima orientação inicial baseada no acelerômetro
        float norm = sqrtf(ax*ax + ay*ay + az*az);
        if (norm > 0.1f)  // Verifica se há dados válidos
        { 
            ax /= norm; ay /= norm; az /= norm;
            
            // Calcula quaternion inicial assumindo que o acelerômetro aponta para baixo
            float roll_init = atan2f(ay, az);
            float pitch_init = atan2f(-ax, sqrtf(ay*ay + az*az));
            
            // Converte para quaternion
            float cy = cosf(0 * 0.5f); // yaw = 0
            float sy = sinf(0 * 0.5f);
            float cp = cosf(pitch_init * 0.5f);
            float sp = sinf(pitch_init * 0.5f);
            float cr = cosf(roll_init * 0.5f);
            float sr = sinf(roll_init * 0.5f);

            sensor->ahrs.orientation.q0 = cr * cp * cy + sr * sp * sy;
            sensor->ahrs.orientation.q1 = sr * cp * cy - cr * sp * sy;
            sensor->ahrs.orientation.q2 = cr * sp * cy + sr * cp * sy;
            sensor->ahrs.orientation.q3 = cr * cp * sy - sr * sp * cy;

            sensor->initialized = true;
            printf("Quaternion inicializado para sensor MPU %d\n", mpu->id);
        }
    }

    // Aplica o filtro de Madgwick (versão AHRS com magnetômetro)
    MadgwickAHRSupdate(&sensor->ahrs);

    // Reduz o beta após alguns segundos para maior estabilidade
    sensor->sample_count++; 
    if (sensor->sample_count == 500) // Após 5 segundos (500 amostras a 100Hz)
    { 
        sensor->ahrs.beta = 0.2f; // Reduz para valor mais estável
        printf("Beta reduzido para maior estabilidade - MPU %d\n", mpu->id);
    }

    // Calcula ângulos de Euler a partir dos quaternions do sensor
    sensor->yaw = (atan2f(2.0f * (sensor->ahrs.orientation.q1*sensor->ahrs.orientation.q2 + sensor->ahrs.orientation.q0*sensor->ahrs.orientation.q3), 
                         sensor->ahrs.orientation.q0*sensor->ahrs.orientation.q0 + sensor->ahrs.orientation.q1*sensor->ahrs.orientation.q1 - sensor->ahrs.orientation.q2*sensor->ahrs.orientation.q2 - sensor->ahrs.orientation.q3*sensor->ahrs.orientation.q3)) * RAD_TO_DEG_CONST;
    sensor->pitch = (-asinf(2.0f * (sensor->ahrs.orientation.q1*sensor->ahrs.orientation.q3 - sensor->ahrs.orientation.q0*sensor->ahrs.orientation.q2))) * RAD_TO_DEG_CONST;
    sensor->roll = (atan2f(2.0f * (sensor->ahrs.orientation.q0*sensor->ahrs.orientation.q1 + sensor->ahrs.orientation.q2*sensor->ahrs.orientation.q3), 
                          sensor->ahrs.orientation.q0*sensor->ahrs.orientation.q0 - sensor->ahrs.orientation.q1*sensor->ahrs.orientation.q1 - sensor->ahrs.orientation.q2*sensor->ahrs.orientation.q2 + sensor->ahrs.orientation.q3*sensor->ahrs.orientation.q3)) * RAD_TO_DEG_CONST;
}

// Função para inicializar o IMu

/**
 * @brief Função para inicialização do IMU.
 * 
 * @param mpu Estrutura de dados contendo informações do MPU6050
 * @param config Estrutura de configuração do MPU6050
 */
void imu_init(mpu9250_t *mpu, mpu9250_config_t *config) 
{
    // Inicializa o MPU9250
    printf("Inicializando MPU9250...\n");
    if (!mpu9250_init(mpu, config)) 
    {
        printf("ERROR: Failed to initialize MPU9250!\n");
        while (1) 
        {
            sleep_ms(1000);
        }
    }

    printf("MPU9250 Inicializada com sucesso!\n");
    
    // Testa as conexões com o sensor
    if (mpu9250_test_connection(mpu)) 
    {
        printf("MPU9250 conexão: OK\n");
    } 
    else 
    {
        printf("MPU9250 conexão: FAILED\n");
    }
    
    // Testa a conexão com o magnetômetro (é um sensor terceiro acoplado no módulo)
    if (mpu9250_test_mag_connection(mpu)) 
    {
        printf("Magnetometer conexão: OK\n");
    } 
    else 
    {
        printf("Magnetometer conexão: FAILED\n");
    }
    
    // Faz o auto-teste do MPU9250
    printf("Realizando auto-teste...\n");
    if (mpu9250_self_test(mpu)) 
    {
        printf("Auto-teste: APROVADO\n");
    } 
    else 
    {
        printf("Auto-teste: FALHOU\n");
    }

    // Seleciona a estrutura de dados do sensor correspondente
    sensor_data_t *sensor = (mpu->id == 0) ? &sensor_0 : &sensor_1;

    // Inicializa a estrutura do filtro de Madgwick para o sensor
    MadgwickAHRSinit(&sensor->ahrs, SAMPLE_FREQ_HZ);
    // Inicializa o resto das variáveis do sensor
    sensor->sample_count = 0;
    sensor->initialized = false;

    printf("IMU %d inicializado com sucesso!\n", mpu->id);
}

// Função para iniciar a aquisição de dados com dois sensores
void imu_start_dual_sensors(mpu9250_t *mpu0, mpu9250_t *mpu1) 
{
    printf("Iniciando aquisição de dados para ambos os sensores...\n");
    
    // Salva referências globais dos sensores
    global_mpu0 = mpu0;
    global_mpu1 = mpu1;
    
    int32_t delay_us = SAMPLE_PERIOD_US; // 10000us = 10ms → 100Hz sampling rate
    if (!add_repeating_timer_us(delay_us, sampling_timer_callback, NULL, &timer)) 
    {
        printf("Failed to add repeating timer\n");
        return;
    }

    printf("Sistema iniciado - Filtro de Madgwick com 2 MPU9250\n");
    printf("Aguarde alguns segundos para estabilização...\n\n");

    // Inicia o segundo núcleo para exibição dos dados
    multicore_launch_core1(core1_entry);
    
    // Aguarda estabilização antes de inicializar o watchdog
    printf("Aguardando estabilização dos sensores...\n");
    for (int i = 5; i > 0; i--) {
        printf("Aguardando %d segundos...\n", i);
        sleep_ms(1000);
    }
    
    // Agora inicializa o sistema de watchdog
    printf("Inicializando sistema de watchdog...\n");
    sensor_watchdog_init();
    
    printf("Aguardando mais 3 segundos antes de ativar watchdog...\n");
    for (int i = 3; i > 0; i--) {
        printf("Ativando watchdog em %d segundos...\n", i);
        sleep_ms(1000);
    }
    sensor_watchdog_enable();
    
    // Loop principal com frequência de ~100Hz (10ms)
    while (1) 
    {
        if (sample_mpu9250 == true) 
        {
            // Processa dados de ambos os sensores
            process_imu_data(global_mpu0, &sensor_0);
            process_imu_data(global_mpu1, &sensor_1);
            
            // Atualiza o watchdog para verificar travamentos
            sensor_watchdog_update();
            
            sample_mpu9250 = false;
        }
    }
}

// Função para iniciar a aquisição de dados (compatibilidade)
void imu_start(mpu9250_t *mpu) 
{
    printf("Iniciando aquisição de dados IMU %d...\n", mpu->id);
    
    int32_t delay_us = SAMPLE_PERIOD_US; // 10000us = 10ms → 100Hz sampling rate

    if (!add_repeating_timer_us(delay_us, sampling_timer_callback, NULL, &timer)) 
    {
        printf("Failed to add repeating timer\n");
        return;
    }

    printf("Sistema iniciado - Filtro de Madgwick com MPU9250 %d\n", mpu->id);
    printf("Aguarde alguns segundos para estabilização...\n\n");

    // Inicia o segundo núcleo para exibição dos dados
    multicore_launch_core1(core1_entry);
    
    // Seleciona a estrutura de dados do sensor correspondente
    sensor_data_t *sensor = (mpu->id == 0) ? &sensor_0 : &sensor_1;
    
    // Loop principal com frequência de ~100Hz (10ms)
    while (1) 
    {
        if (sample_mpu9250 == true) 
        {
            process_imu_data(mpu, sensor);
            sample_mpu9250 = false;
        }
    }
}

// Função para parar a aquisição
void imu_stop(void) 
{
    cancel_repeating_timer(&timer);
    cancel_repeating_timer(&timer2);
    printf("Aquisição de dados IMU parada.\n");
}


// Função para imprimir dados uma vez
void imu_print_data(void) 
{
    printf("MPU0: Roll=%6.2f° Pitch=%6.2f° Yaw=%6.2f° | MPU1: Roll=%6.2f° Pitch=%6.2f° Yaw=%6.2f°\n", 
           sensor_0.roll, sensor_0.pitch, sensor_0.yaw,
           sensor_1.roll, sensor_1.pitch, sensor_1.yaw);
}
