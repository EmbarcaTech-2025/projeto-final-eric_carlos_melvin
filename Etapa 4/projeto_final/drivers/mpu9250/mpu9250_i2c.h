// ======================================================================
//  Arquivo: mpu9250_i2c.h
//  Descrição: Interface do driver para o sensor inercial MPU9250 (I2C)
//  Autor: [Seu Nome]
//  Data: [Data de modificação]
// ======================================================================

#ifndef MPU9250_I2C_H
#define MPU9250_I2C_H

#include "pico/stdlib.h"   // Tipos e funções do Pico SDK
#include "hardware/i2c.h"  // Tipos e funções de I2C

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------------------------
// Constantes de sensibilidade dos sensores
// ----------------------------------------------------------------------
#define ACCEL_SENS_2G   16384.0f ///< Sensibilidade do acelerômetro ±2g
#define ACCEL_SENS_4G   8192.0f  ///< Sensibilidade do acelerômetro ±4g
#define ACCEL_SENS_8G   4096.0f  ///< Sensibilidade do acelerômetro ±8g
#define ACCEL_SENS_16G  2048.0f  ///< Sensibilidade do acelerômetro ±16g

#define GYRO_SENS_250DPS   131.0f  ///< Sensibilidade do giroscópio ±250°/s
#define GYRO_SENS_500DPS   65.5f   ///< Sensibilidade do giroscópio ±500°/s
#define GYRO_SENS_1000DPS  32.8f   ///< Sensibilidade do giroscópio ±1000°/s
#define GYRO_SENS_2000DPS  16.4f   ///< Sensibilidade do giroscópio ±2000°/s

#define MAG_SENS 0.15f ///< Sensibilidade do magnetômetro (LSB/µT)

// ----------------------------------------------------------------------
// Endereços I2C dos sensores
// ----------------------------------------------------------------------
#define MPU9250_ADDR_0 0x68 ///< Endereço padrão do MPU9250 (AD0=0)
#define MPU9250_ADDR_1 0x69 ///< Endereço alternativo do MPU9250 (AD0=1)
#define AK8963_ADDR    0x0C ///< Endereço do magnetômetro AK8963

// ----------------------------------------------------------------------
// Pinos GPIO para interface I2C
// ----------------------------------------------------------------------
#define I2C0_SDA 0 ///< GPIO SDA para I2C0
#define I2C0_SCL 1 ///< GPIO SCL para I2C0
#define I2C1_SDA 2 ///< GPIO SDA para I2C1
#define I2C1_SCL 3 ///< GPIO SCL para I2C1

// ----------------------------------------------------------------------
// Tipos de configuração do acelerômetro, giroscópio e filtros
// ----------------------------------------------------------------------
typedef enum {
    MPU9250_ACCEL_RANGE_2G  = 0x00, ///< ±2g
    MPU9250_ACCEL_RANGE_4G  = 0x08, ///< ±4g
    MPU9250_ACCEL_RANGE_8G  = 0x10, ///< ±8g
    MPU9250_ACCEL_RANGE_16G = 0x18  ///< ±16g
} mpu9250_accel_range_t;

typedef enum {
    MPU9250_GYRO_RANGE_250DPS  = 0x00, ///< ±250°/s
    MPU9250_GYRO_RANGE_500DPS  = 0x08, ///< ±500°/s
    MPU9250_GYRO_RANGE_1000DPS = 0x10, ///< ±1000°/s
    MPU9250_GYRO_RANGE_2000DPS = 0x18  ///< ±2000°/s
} mpu9250_gyro_range_t;

typedef enum {
    MPU9250_DLPF_184HZ = 0x01, ///< Filtro passa-baixa 184Hz
    MPU9250_DLPF_92HZ  = 0x02, ///< Filtro passa-baixa 92Hz
    MPU9250_DLPF_41HZ  = 0x03, ///< Filtro passa-baixa 41Hz
    MPU9250_DLPF_20HZ  = 0x04, ///< Filtro passa-baixa 20Hz
    MPU9250_DLPF_10HZ  = 0x05, ///< Filtro passa-baixa 10Hz
    MPU9250_DLPF_5HZ   = 0x06  ///< Filtro passa-baixa 5Hz
} mpu9250_dlpf_t;

typedef enum {
    AK8963_POWER_DOWN      = 0x00, ///< Modo standby
    AK8963_SINGLE_MEASURE  = 0x01, ///< Medida única
    AK8963_CONTINUOUS_8HZ  = 0x02, ///< Medidas contínuas 8Hz
    AK8963_CONTINUOUS_100HZ= 0x06, ///< Medidas contínuas 100Hz
    AK8963_SELF_TEST       = 0x08, ///< Auto-teste
    AK8963_FUSE_ROM        = 0x0F  ///< Leitura dos fusíveis de calibração
} ak8963_mode_t;

// ----------------------------------------------------------------------
// Estruturas de configuração e dados do sensor
// ----------------------------------------------------------------------
/**
 * @brief Estrutura de configuração e estado do MPU9250.
 */
typedef struct{
    i2c_inst_t *i2c;        ///< Instância I2C utilizada
    uint sda_gpio;          ///< Pino GPIO SDA
    uint scl_gpio;          ///< Pino GPIO SCL
    uint8_t addr;           ///< Endereço I2C do MPU9250 (0x68 ou 0x69)
    uint8_t id;             ///< ID lógico do sensor

    // Fatores de sensibilidade para conversão
    float accel_sensitivity; ///< Sensibilidade do acelerômetro
    float gyro_sensitivity;  ///< Sensibilidade do giroscópio

    // Calibração do magnetômetro
    float mag_asa[3];       ///< Ajuste de sensibilidade do magnetômetro
    bool mag_enabled;       ///< true se magnetômetro habilitado

    // Offsets de calibração (em unidades físicas)
    float accel_offset[3];  ///< Offset do acelerômetro (g)
    float gyro_offset[3];   ///< Offset do giroscópio (°/s)
    float mag_offset[3];    ///< Offset do magnetômetro (µT)
} mpu9250_t;

/**
 * @brief Estrutura para dados brutos lidos dos sensores.
 */
typedef struct {
    int16_t accel[3];   ///< Acelerômetro bruto [x, y, z]
    int16_t gyro[3];    ///< Giroscópio bruto [x, y, z]
    int16_t mag[3];     ///< Magnetômetro bruto [x, y, z]
    int16_t temp;       ///< Temperatura bruta
} mpu9250_raw_data_t;

/**
 * @brief Estrutura para dados processados dos sensores.
 */
typedef struct {
    float accel[3];     ///< Acelerômetro em g [x, y, z]
    float gyro[3];      ///< Giroscópio em °/s [x, y, z]
    float mag[3];       ///< Magnetômetro em µT [x, y, z]
    float temp;         ///< Temperatura em °C
} mpu9250_data_t;

/**
 * @brief Estrutura de configuração para ajustes do sensor.
 */
typedef struct {
    mpu9250_accel_range_t accel_range; ///< Range do acelerômetro
    mpu9250_gyro_range_t gyro_range;   ///< Range do giroscópio
    mpu9250_dlpf_t dlpf_filter;        ///< Filtro digital passa-baixa
    uint8_t sample_rate_divider;       ///< Divisor da taxa de amostragem
    bool enable_magnetometer;          ///< true para habilitar magnetômetro
} mpu9250_config_t;

// ----------------------------------------------------------------------
// Protótipos das funções do driver MPU9250
// ----------------------------------------------------------------------
/** @brief Inicializa a interface I2C para o MPU9250. */
void mpu9250_setup_i2c(mpu9250_t *mpu);

/** @brief Reseta o MPU9250 para o estado padrão. */
void mpu9250_reset(mpu9250_t *mpu);

/**
 * @brief Inicializa o MPU9250 com a configuração especificada.
 * @return true se bem-sucedido, false caso contrário
 */
bool mpu9250_init(mpu9250_t *mpu, mpu9250_config_t *config);

/** @brief Obtém a configuração atual do range do acelerômetro. */
uint8_t mpu9250_get_accel_range(mpu9250_t *mpu);

/** @brief Define o range do acelerômetro. */
void mpu9250_set_accel_range(mpu9250_t *mpu, mpu9250_accel_range_t range);

/** @brief Obtém a configuração atual do range do giroscópio. */
uint8_t mpu9250_get_gyro_range(mpu9250_t *mpu);

/** @brief Define o range do giroscópio. */
void mpu9250_set_gyro_range(mpu9250_t *mpu, mpu9250_gyro_range_t range);

/** @brief Define o Filtro Digital Passa-Baixa (DLPF). */
void mpu9250_set_dlpf(mpu9250_t *mpu, mpu9250_dlpf_t filter);

/** @brief Define o divisor da taxa de amostragem. */
void mpu9250_set_sample_rate(mpu9250_t *mpu, uint8_t divider);

/** @brief Verifica se o MPU9250 está conectado e respondendo. */
bool mpu9250_test_connection(mpu9250_t *mpu);

/** @brief Verifica se o magnetômetro está conectado e respondendo. */
bool mpu9250_test_mag_connection(mpu9250_t *mpu);

/** @brief Habilita/desabilita o magnetômetro. */
bool mpu9250_enable_magnetometer(mpu9250_t *mpu, bool enable);

/** @brief Lê dados brutos dos sensores do MPU9250. */
void mpu9250_read_raw(mpu9250_t *mpu, mpu9250_raw_data_t *data);

/** @brief Lê dados brutos do acelerômetro e giroscópio. */
void mpu9250_read_raw_motion(mpu9250_t *mpu, int16_t accel[3], int16_t gyro[3], int16_t *temp);

/** @brief Lê dados brutos do magnetômetro. */
void mpu9250_read_raw_mag(mpu9250_t *mpu, int16_t mag[3]);

/** @brief Lê dados processados dos sensores do MPU9250. */
void mpu9250_read_data(mpu9250_t *mpu, mpu9250_data_t *data);

/** @brief Lê dados processados do acelerômetro e giroscópio. */
void mpu9250_read_motion(mpu9250_t *mpu, float accel[3], float gyro[3], float *temp);

/** @brief Lê dados processados do magnetômetro. */
void mpu9250_read_mag(mpu9250_t *mpu, float mag[3]);

/** @brief Lê dados da temperatura. */
float mpu9250_read_temperature(mpu9250_t *mpu);

/** @brief Função de debug para verificar status do magnetômetro. */
void mpu9250_debug_mag_status(mpu9250_t *mpu);

/**
 * @brief Calibra o giroscópio lendo valores de offset.
 * @param samples Número de amostras para calcular a média (recomendado: 1000)
 * @param gyro_offset Array para armazenar valores de offset do giroscópio [x, y, z]
 */
void mpu9250_calibrate_gyro(mpu9250_t *mpu, uint16_t samples, float gyro_offset[3]);

/** @brief Função de auto-teste para o MPU9250. */
bool mpu9250_self_test(mpu9250_t *mpu);

#ifdef __cplusplus
}
#endif

#endif // MPU9250_I2C_H
