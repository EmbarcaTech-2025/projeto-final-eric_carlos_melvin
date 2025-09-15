#ifndef IMU_MATH_H
#define IMU_MATH_H

#include <math.h>
#include <stdbool.h>

// Estrutura de quaternion
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

// Converte Euler (roll, pitch, yaw) para quaternion
Quaternion euler_to_quaternion(float roll, float pitch, float yaw);

// Multiplicação de quaternions (q1 ⊗ q2)
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2);

// Conjugado do quaternion (q⁻¹ se for unitário)
Quaternion quaternion_conjugate(Quaternion q);

// Calcula o quaternion relativo (tronco -> coxa)
Quaternion relative_quaternion(Quaternion q_tronco, Quaternion q_coxa);

// Converte quaternion para ângulos de Euler usando sequência XYZ (anatômica corrigida)
// X: flexão para frente(+)/extensão para trás(-), Y: rotação interna/externa, Z: adução/abdução
void quaternion_to_hip_angles(Quaternion q, float *flexao, float *aducao, float *rotacao);

// Extrai ângulos articulares do quadril e detecta posturas perigosas
// flexao: flexão para frente(+)/extensão para trás(-) X-axis (em radianos)
// aducao: adução/abdução Z-axis (em radianos)
// rotacao: rotação interna/externa Y-axis (em radianos)
// rotacao_interna_30: true se rotação interna > 30°
// flexao_maior_90: true se flexão para frente > 90°
// cruzamento_pernas: true se adução excessiva (cruzamento)
void hip_angles(Quaternion q_rel, float *flexao, float *aducao, float *rotacao, bool *rotacao_interna_30, bool *flexao_maior_90, bool *cruzamento_pernas);

// Função auxiliar para debug - converte radianos para graus
float rad_to_deg(float rad);

// Função auxiliar para debug - imprime ângulos do quadril
void print_hip_angles(Quaternion q_rel);

#endif
