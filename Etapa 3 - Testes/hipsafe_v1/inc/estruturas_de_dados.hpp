#ifndef Estrutura_Dados_HPP_
#define Estrutura_Dados_HPP_

#include <cstdint>

// Substituto MOCK para i2c_inst_t do Pico SDK
typedef struct{
    int mock_id;  // apenas um ID fictício
} i2c_inst_t;

// STRUCTS:

typedef struct{
    float flexao;
    float rotacao;
    float abducao;
} Orientacao;

typedef struct {
    bool ligado = false;
    bool silenciado = false;
} Alarme;

typedef struct{
    i2c_inst_t i2c; // I2C instance
    unsigned int sda_gpio;   // SDA GPIO pin
    unsigned int scl_gpio;   // SCL GPIO pin
    uint8_t addr;   // MPU6050 I2C address (0x68 or 0x69)
    uint8_t id;     // MPU6050 Number ID
} mpu6050_t;

typedef struct {
	float acelerometro[3] = {0.0, 0.0, 0.0};
	float giroscopio[3] = {0.0, 0.0, 0.0};
	float magnetometro[3] = {0.0, 0.0, 0.0};
} sensor_data;

typedef struct {
	float Q0 = 0.0;
	float Q1 = 0.0;
	float Q2 = 0.0;
	float Q3 =  0.0;
} Quartenion;

// ENUNS: 
// Definições de tipo de movimento para monitoramento de postura
enum class TipoMovimento
{
    FLEXAO,
    ABDUCAO,
    ROTACAO,    
    NORMAL    
};

// Definições de lado do corpo para rastreamento de localização
enum class LadoCorpo
{
    DIREITO,  // Perna direita
    ESQUERDO  // Perna esquerda
};

#endif