// ======================================================================
//  Arquivo: algoritmo_postura.h
//  Descrição: Algoritmos matemáticos para análise postural com quaternions
// ======================================================================

#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <math.h>      // Funções matemáticas padrão
#include <stdbool.h>   // Tipo booleano padrão

// ----------------------------------------------------------------------
// Estrutura: Quaternion
// ----------------------------------------------------------------------
/**
 * @brief Representa um quaternion para orientação espacial 3D.
 *
 * w: componente escalar
 * x, y, z: componentes vetoriais
 */
typedef struct {
    float w; ///< Componente escalar
    float x; ///< Componente X
    float y; ///< Componente Y
    float z; ///< Componente Z
} Quaternion;

// ----------------------------------------------------------------------
// Conversão de ângulos de Euler para quaternion
// ----------------------------------------------------------------------
/**
 * @brief Converte ângulos de Euler (roll, pitch, yaw) para quaternion.
 * @param roll Ângulo de rotação em torno do eixo X (rad)
 * @param pitch Ângulo de rotação em torno do eixo Y (rad)
 * @param yaw Ângulo de rotação em torno do eixo Z (rad)
 * @return Quaternion correspondente à orientação
 */
Quaternion euler_to_quaternion(float roll, float pitch, float yaw);

// ----------------------------------------------------------------------
// Operações com quaternions
// ----------------------------------------------------------------------
/**
 * @brief Multiplica dois quaternions (q1 ⊗ q2).
 * @param q1 Primeiro quaternion
 * @param q2 Segundo quaternion
 * @return Resultado da multiplicação
 */
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);

/**
 * @brief Calcula o conjugado de um quaternion (q⁻¹ se unitário).
 * @param q Quaternion de entrada
 * @return Conjugado do quaternion
 */
Quaternion quaternion_conjugate(Quaternion q);

/**
 * @brief Calcula o quaternion relativo entre dois segmentos (ex: tronco -> coxa).
 * @param q_tronco Quaternion do tronco
 * @param q_coxa Quaternion da coxa
 * @return Quaternion relativo (orientação da coxa em relação ao tronco)
 */
Quaternion relative_quaternion(Quaternion q_tronco, Quaternion q_coxa);

// ----------------------------------------------------------------------
// Conversão de quaternion para ângulos articulares do quadril
// ----------------------------------------------------------------------
/**
 * @brief Converte um quaternion para ângulos articulares do quadril (flexão, adução, rotação).
 *
 * Utiliza sequência XYZ (anatômica corrigida):
 *  - X: flexão para frente(+)/extensão para trás(-)
 *  - Y: rotação interna/externa
 *  - Z: adução/abdução
 *
 * @param q Quaternion relativo
 * @param[out] flexao Ângulo de flexão (rad)
 * @param[out] aducao Ângulo de adução (rad)
 * @param[out] rotacao Ângulo de rotação (rad)
 */
void quaternion_to_hip_angles(Quaternion q, float *flexao, float *aducao, float *rotacao);

// ----------------------------------------------------------------------
// Funções auxiliares para debug
// ----------------------------------------------------------------------
/**
 * @brief Converte valor em radianos para graus.
 * @param rad Valor em radianos
 * @return Valor em graus
 */
float rad_to_deg(float rad);

/**
 * @brief Imprime no console os ângulos articulares do quadril extraídos de um quaternion relativo.
 * @param q_rel Quaternion relativo (tronco -> coxa)
 */
void print_hip_angles(Quaternion q_rel);

#endif // IMU_MATH_H
