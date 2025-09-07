#ifndef MPU9250_I2C_H // proteção de inclusão
#define MPU9250_I2C_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// Constantes de sensibilidade do acelerômetro
#define ACCEL_SENS_2G  16384.0f
#define ACCEL_SENS_4G  8192.0f
#define ACCEL_SENS_8G  4096.0f
#define ACCEL_SENS_16G 2048.0f

// Constantes de sensibilidade do giroscópio (LSB/°/s)
#define GYRO_SENS_250DPS  131.0f
#define GYRO_SENS_500DPS  65.5f
#define GYRO_SENS_1000DPS 32.8f
#define GYRO_SENS_2000DPS 16.4f

// Constante de sensibilidade do magnetômetro (LSB/µT)
#define MAG_SENS 0.15f

// Endereços I2C para MPU9250
#define MPU9250_ADDR_0 0x68 
#define MPU9250_ADDR_1 0x69 

// Endereço I2C do magnetômetro AK8963
#define AK8963_ADDR 0x0C

// Pinos GPIO para interface I2C
#define I2C0_SDA 0 
#define I2C0_SCL 1 
#define I2C1_SDA 2 
#define I2C1_SCL 3 

// Opções de range do acelerômetro
typedef enum {
    MPU9250_ACCEL_RANGE_2G = 0x00,
    MPU9250_ACCEL_RANGE_4G = 0x08,
    MPU9250_ACCEL_RANGE_8G = 0x10,
    MPU9250_ACCEL_RANGE_16G = 0x18
} mpu9250_accel_range_t;

// Opções de range do giroscópio  
typedef enum {
    MPU9250_GYRO_RANGE_250DPS = 0x00,
    MPU9250_GYRO_RANGE_500DPS = 0x08,
    MPU9250_GYRO_RANGE_1000DPS = 0x10,
    MPU9250_GYRO_RANGE_2000DPS = 0x18
} mpu9250_gyro_range_t;

// Opções de Filtro Digital Passa-Baixa (DLPF)
typedef enum {
    MPU9250_DLPF_184HZ = 0x01,
    MPU9250_DLPF_92HZ = 0x02,
    MPU9250_DLPF_41HZ = 0x03,
    MPU9250_DLPF_20HZ = 0x04,
    MPU9250_DLPF_10HZ = 0x05,
    MPU9250_DLPF_5HZ = 0x06
} mpu9250_dlpf_t;

// Modos de operação do magnetômetro
typedef enum {
    AK8963_POWER_DOWN = 0x00,
    AK8963_SINGLE_MEASURE = 0x01,
    AK8963_CONTINUOUS_8HZ = 0x02,
    AK8963_CONTINUOUS_100HZ = 0x06,
    AK8963_SELF_TEST = 0x08,
    AK8963_FUSE_ROM = 0x0F
} ak8963_mode_t;

// Estrutura de configuração do MPU9250
typedef struct{
    i2c_inst_t *i2c;        // Instância I2C
    uint sda_gpio;          // Pino GPIO SDA
    uint scl_gpio;          // Pino GPIO SCL
    uint8_t addr;           // Endereço I2C do MPU9250 (0x68 ou 0x69)
    uint8_t id;             // ID do MPU9250
    
    // Fatores de sensibilidade para conversão
    float accel_sensitivity;
    float gyro_sensitivity;
    
    // Valores de calibração do magnetômetro
    float mag_asa[3];       // Valores de ajuste de sensibilidade
    bool mag_enabled;       // Flag de habilitação do magnetômetro
} mpu9250_t;

// Estrutura de dados brutos dos sensores
typedef struct {
    int16_t accel[3];       // Dados brutos do acelerômetro [x, y, z]
    int16_t gyro[3];        // Dados brutos do giroscópio [x, y, z] 
    int16_t mag[3];         // Dados brutos do magnetômetro [x, y, z]
    int16_t temp;           // Dados brutos da temperatura
} mpu9250_raw_data_t;

// Estrutura de dados processados dos sensores
typedef struct {
    float accel[3];         // Dados do acelerômetro em g [x, y, z]
    float gyro[3];          // Dados do giroscópio em °/s [x, y, z]
    float mag[3];           // Dados do magnetômetro em µT [x, y, z]
    float temp;             // Temperatura em °C
} mpu9250_data_t;

// Estrutura de configuração para ajustes do sensor
typedef struct {
    mpu9250_accel_range_t accel_range;
    mpu9250_gyro_range_t gyro_range;
    mpu9250_dlpf_t dlpf_filter;
    uint8_t sample_rate_divider;    // Taxa de atualização = 1000 / (1 + divisor) Hz
    bool enable_magnetometer;
} mpu9250_config_t;

// Protótipos das funções

/**
 * @brief Inicializa a interface I2C para o MPU9250
 * @param mpu Ponteiro para a estrutura MPU9250
 */
void mpu9250_setup_i2c(mpu9250_t *mpu);

/**
 * @brief Reseta o MPU9250 para o estado padrão
 * @param mpu Ponteiro para a estrutura MPU9250
 */
void mpu9250_reset(mpu9250_t *mpu);

/**
 * @brief Inicializa o MPU9250 com a configuração especificada
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param config Ponteiro para a estrutura de configuração
 * @return true se bem-sucedido, false caso contrário
 */
bool mpu9250_init(mpu9250_t *mpu, mpu9250_config_t *config);

/**
 * @brief Obtém a configuração atual do range do acelerômetro
 * @param mpu Ponteiro para a estrutura MPU9250
 * @return Range atual do acelerômetro (0=±2g, 1=±4g, 2=±8g, 3=±16g)
 */
uint8_t mpu9250_get_accel_range(mpu9250_t *mpu);

/**
 * @brief Define o range do acelerômetro
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param range Range do acelerômetro (0=±2g, 1=±4g, 2=±8g, 3=±16g)
 */
void mpu9250_set_accel_range(mpu9250_t *mpu, mpu9250_accel_range_t range);

/**
 * @brief Obtém a configuração atual do range do giroscópio
 * @param mpu Ponteiro para a estrutura MPU9250
 * @return Range atual do giroscópio (0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s)
 */
uint8_t mpu9250_get_gyro_range(mpu9250_t *mpu);

/**
 * @brief Define o range do giroscópio
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param range Range do giroscópio (0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s)
 */
void mpu9250_set_gyro_range(mpu9250_t *mpu, mpu9250_gyro_range_t range);

/**
 * @brief Define o Filtro Digital Passa-Baixa (DLPF)
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param filter Configuração do DLPF
 */
void mpu9250_set_dlpf(mpu9250_t *mpu, mpu9250_dlpf_t filter);

/**
 * @brief Define o divisor da taxa de amostragem
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param divider Divisor da taxa de amostragem (taxa = 1000/(1+divisor) Hz)
 */
void mpu9250_set_sample_rate(mpu9250_t *mpu, uint8_t divider);

/**
 * @brief Verifica se o MPU9250 está conectado e respondendo
 * @param mpu Ponteiro para a estrutura MPU9250
 * @return true se o dispositivo está respondendo, false caso contrário
 */
bool mpu9250_test_connection(mpu9250_t *mpu);

/**
 * @brief Verifica se o magnetômetro está conectado e respondendo  
 * @param mpu Ponteiro para a estrutura MPU9250
 * @return true se o magnetômetro está respondendo, false caso contrário
 */
bool mpu9250_test_mag_connection(mpu9250_t *mpu);

/**
 * @brief Habilita/desabilita o magnetômetro
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param enable true para habilitar, false para desabilitar
 * @return true se bem-sucedido, false caso contrário
 */
bool mpu9250_enable_magnetometer(mpu9250_t *mpu, bool enable);

/**
 * @brief Lê dados brutos dos sensores do MPU9250
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param data Ponteiro para a estrutura de dados brutos
 */
void mpu9250_read_raw(mpu9250_t *mpu, mpu9250_raw_data_t *data);

/**
 * @brief Lê dados brutos do acelerômetro e giroscópio
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param accel Array para armazenar dados do acelerômetro [x, y, z]
 * @param gyro Array para armazenar dados do giroscópio [x, y, z]
 * @param temp Ponteiro para armazenar dados da temperatura
 */
void mpu9250_read_raw_motion(mpu9250_t *mpu, int16_t accel[3], int16_t gyro[3], int16_t *temp);

/**
 * @brief Lê dados brutos do magnetômetro
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param mag Array para armazenar dados do magnetômetro [x, y, z]
 */
void mpu9250_read_raw_mag(mpu9250_t *mpu, int16_t mag[3]);

/**
 * @brief Lê dados processados dos sensores do MPU9250
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param data Ponteiro para a estrutura de dados processados
 */
void mpu9250_read_data(mpu9250_t *mpu, mpu9250_data_t *data);

/**
 * @brief Lê dados processados do acelerômetro e giroscópio
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param accel Array para armazenar dados do acelerômetro em g [x, y, z]
 * @param gyro Array para armazenar dados do giroscópio em °/s [x, y, z]
 * @param temp Ponteiro para armazenar temperatura em °C
 */
void mpu9250_read_motion(mpu9250_t *mpu, float accel[3], float gyro[3], float *temp);

/**
 * @brief Lê dados processados do magnetômetro
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param mag Array para armazenar dados do magnetômetro em µT [x, y, z]
 */
void mpu9250_read_mag(mpu9250_t *mpu, float mag[3]);

/**
 * @brief Lê dados da temperatura
 * @param mpu Ponteiro para a estrutura MPU9250
 * @return Temperatura em graus Celsius
 */
float mpu9250_read_temperature(mpu9250_t *mpu);

/**
 * @brief Função de debug para verificar status do magnetômetro
 * @param mpu Ponteiro para a estrutura MPU9250
 */
void mpu9250_debug_mag_status(mpu9250_t *mpu);

/**
 * @brief Calibra o giroscópio lendo valores de offset
 * @param mpu Ponteiro para a estrutura MPU9250
 * @param samples Número de amostras para calcular a média (recomendado: 1000)
 * @param gyro_offset Array para armazenar valores de offset do giroscópio [x, y, z]
 */
void mpu9250_calibrate_gyro(mpu9250_t *mpu, uint16_t samples, float gyro_offset[3]);

/**
 * @brief Função de auto-teste para o MPU9250
 * @param mpu Ponteiro para a estrutura MPU9250
 * @return true se o auto-teste passou, false caso contrário
 */
bool mpu9250_self_test(mpu9250_t *mpu);

#endif // MPU9250_I2C_H
