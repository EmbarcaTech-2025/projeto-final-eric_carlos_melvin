// ======================================================================
//  Arquivo: estruturas_de_dados.hpp
//  Descrição: Definição das principais estruturas e enums para sensores,
//             alarmes e monitoramento postural.
// ======================================================================

#ifndef ESTRUTURA_DADOS_HPP_
#define ESTRUTURA_DADOS_HPP_

#include <cstdint>           // Tipos inteiros padrão (ex: uint8_t)
#include "hardware/i2c.h"    // Definição de i2c_inst_t para comunicação I2C

// ----------------------------------------------------------------------
// Estrutura: Orientacao
// ----------------------------------------------------------------------
/**
 * @brief Representa os ângulos articulares principais de uma junta monitorada.
 *        Utilizada para armazenar os valores de flexão, rotação e abdução (em graus).
 */
typedef struct {
    float flexao;   ///< Ângulo de flexão (graus)
    float rotacao;  ///< Ângulo de rotação (graus)
    float abducao;  ///< Ângulo de abdução (graus)
} Orientacao;

// ----------------------------------------------------------------------
// Estrutura: Alarme
// ----------------------------------------------------------------------
/**
 * @brief Representa o estado global do alarme sonoro do sistema.
 *
 * - ligado: indica se o alarme está ativo (situação de risco)
 * - silenciado: indica se o alarme foi silenciado manualmente pelo usuário
 */
typedef struct {
    bool ligado = false;      ///< true se o alarme está ativo
    bool silenciado = false;  ///< true se o alarme está silenciado manualmente
} Alarme;

// ----------------------------------------------------------------------
// Estrutura: mpu6050_t
// ----------------------------------------------------------------------
/**
 * @brief Configuração e identificação de um sensor MPU6050 conectado via I2C.
 *
 * - i2c: ponteiro para a instância I2C utilizada
 * - sda_gpio/scl_gpio: pinos GPIO utilizados para SDA/SCL
 * - addr: endereço I2C do sensor (0x68 ou 0x69)
 * - id: identificador do sensor
 */
typedef struct {
    i2c_inst_t* i2c;          ///< Instância I2C utilizada
    unsigned int sda_gpio;    ///< Pino GPIO para SDA
    unsigned int scl_gpio;    ///< Pino GPIO para SCL
    uint8_t addr;             ///< Endereço I2C do sensor
    uint8_t id;               ///< Identificador lógico do sensor
} mpu6050_t;

// ----------------------------------------------------------------------
// Estrutura: sensor_data
// ----------------------------------------------------------------------
/**
 * @brief Armazena leituras brutas dos sensores inerciais (aceleração, giroscópio, magnetômetro).
 *        Cada vetor contém três componentes (X, Y, Z).
 */
typedef struct {
    float acelerometro[3] = {0.0, 0.0, 0.0};   ///< Aceleração (m/s²) nos eixos X, Y, Z
    float giroscopio[3]   = {0.0, 0.0, 0.0};   ///< Velocidade angular (°/s) nos eixos X, Y, Z
    float magnetometro[3] = {0.0, 0.0, 0.0};   ///< Campo magnético (uT) nos eixos X, Y, Z
} sensor_data;

// ----------------------------------------------------------------------
// Estrutura: Quartenion
// ----------------------------------------------------------------------
/**
 * @brief Representa um quaternion para orientação espacial (formato W, X, Y, Z).
 *        Utilizado para cálculos de orientação 3D a partir de sensores inerciais.
 */
typedef struct {
    float Q0 = 0.0;   ///< Componente W
    float Q1 = 0.0;   ///< Componente X
    float Q2 = 0.0;   ///< Componente Y
    float Q3 = 0.0;   ///< Componente Z
} Quartenion;

// ----------------------------------------------------------------------
// Enum: TipoMovimento
// ----------------------------------------------------------------------
/**
 * @brief Tipos de movimento monitorados para análise postural.
 *        Utilizado para classificar eventos e limites de risco.
 */
enum class TipoMovimento {
    FLEXAO,    ///< Flexão da articulação
    ABDUCAO,   ///< Abdução da articulação
    ROTACAO,   ///< Rotação da articulação
    NORMAL     ///< Situação normal (sem risco)
};

// ----------------------------------------------------------------------
// Enum: LadoCorpo
// ----------------------------------------------------------------------
/**
 * @brief Identifica o lado do corpo monitorado.
 *        Utilizado para rastrear eventos e sensores por perna.
 */
enum class LadoCorpo {
    DIREITO,   ///< Perna direita
    ESQUERDO   ///< Perna esquerda
};

#endif // ESTRUTURA_DADOS_HPP_