#ifndef ANALISE_POSTURAL_SENSOR_H
#define ANALISE_POSTURAL_SENSOR_H

#include <stdbool.h>
#include "drivers/mpu6050/mpu6050_i2c.h"

// Variáveis globais para armazenar os últimos ângulos lidos de cada sensor
extern float g_roll[3];
extern float g_pitch[3];
extern float g_yaw[3];

/**
 * Requisita as posições dos sensores e atualiza os ângulos globais.
 * 
 * @return true se conseguiu ler os sensores com sucesso, false caso contrário
 */
bool requisitaPosicoes(mpu6050_t *mpu);

/**
 * Compara posições atuais com limites de segurança.
 * 
 * @return true se as posições estão dentro dos limites seguros
 */
bool comparaPosicoes(void);

/**
 * Verifica se a posição atual é perigosa.
 * 
 * @return true se a posição é perigosa (pitch > 90 graus)
 */
bool posicaoPerigosa(void);

#endif
