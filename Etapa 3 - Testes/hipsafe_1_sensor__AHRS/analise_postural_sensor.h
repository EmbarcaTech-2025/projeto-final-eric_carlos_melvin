#ifndef ANALISE_POSTURAL_SENSOR_H
#define ANALISE_POSTURAL_SENSOR_H

#include <stdbool.h>

// Variáveis globais para armazenar os últimos ângulos lidos
extern float g_roll;
extern float g_pitch;
extern float g_yaw;

/**
 * Requisita as posições dos sensores e atualiza os ângulos globais.
 * 
 * @return true se conseguiu ler os sensores com sucesso, false caso contrário
 */
bool requisitaPosicoes(void);

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
