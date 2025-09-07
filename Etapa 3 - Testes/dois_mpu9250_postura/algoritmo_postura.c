#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include "algoritmo_postura.h"
#include <stdio.h>

// Converte roll, pitch, yaw (em rad) para quaternion
Quaternion euler_to_quaternion(float roll, float pitch, float yaw)
{
    Quaternion q;
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// Multiplica dois quaternions
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2)
{
    Quaternion q;
    q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return q;
}

// Conjugado (inverso para quaternions unitários)
Quaternion quaternion_conjugate(Quaternion q)
{
    Quaternion qc;
    qc.w = q.w;
    qc.x = -q.x;
    qc.y = -q.y;
    qc.z = -q.z;
    return qc;
}

// Calcula quaternion relativo (tronco -> coxa)
Quaternion relative_quaternion(Quaternion q_tronco, Quaternion q_coxa)
{
    Quaternion q_inv = quaternion_conjugate(q_tronco);
    return quaternion_multiply(q_inv, q_coxa);
}

// Converte quaternion para ângulos de Euler usando sequência XYZ (anatômica corrigida)
// X: flexão/extensão, Y: rotação interna/externa, Z: adução/abdução
void quaternion_to_hip_angles(Quaternion q, float *flexao, float *aducao, float *rotacao)
{
    // Sequência XYZ para anatomia do quadril corrigida
    // X (sagital): flexão para frente(+)/extensão para trás(-)
    // Y (transversal): rotação interna(+)/rotação externa(-)
    // Z (frontal): adução(+)/abdução(-)
    
    // Flexão/Extensão (rotação em X) - CORRIGIDO para flexão frente ser positiva
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    *flexao = -atan2f(sinr_cosp, cosr_cosp); // Negativo para inverter: frente = +, trás = -
    
    // Rotação Interna/Externa (rotação em Y)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f)
        *rotacao = copysignf(M_PI / 2.0f, sinp);
    else
        *rotacao = asinf(sinp);
    
    // Adução/Abdução (rotação em Z)
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    *aducao = atan2f(siny_cosp, cosy_cosp);
}

// Extrai ângulos do quadril a partir de q_rel e detecta padrões clínicos
void hip_angles(Quaternion q_rel, float *flexao, float *aducao, float *rotacao,
                bool *rotacao_interna_30, bool *flexao_maior_90, bool *cruzamento_pernas)
{
    // Converte quaternion relativo para ângulos anatômicos
    quaternion_to_hip_angles(q_rel, flexao, aducao, rotacao);
    
    // Converte para graus para facilitar comparações
    float flexao_graus = (*flexao) * 180.0f / M_PI;
    float aducao_graus = (*aducao) * 180.0f / M_PI;
    float rotacao_graus = (*rotacao) * 180.0f / M_PI;
    
    // DETECÇÃO DE PADRÕES CLÍNICOS:
    
    // 1. Rotação interna maior que 30 graus (eixo Y)
    // Considera rotação interna como valores positivos
    *rotacao_interna_30 = (rotacao_graus > 30.0f);
    
    // 2. Flexão para frente maior que 90 graus (eixo X)
    // Agora flexão para frente é positiva, então verificamos se > 90°
    *flexao_maior_90 = (flexao_graus > 90.0f);
    
    // 3. Cruzamento de pernas - critério baseado em adução (eixo Z)
    // Considera combinação de adução excessiva E possível rotação interna
    // Critério: adução > 35° OU (adução > 25° E rotação interna > 15°)
    bool aducao_excessiva = (aducao_graus > 35.0f);
    bool aducao_moderada_com_rotacao = (aducao_graus > 20.0f && rotacao_graus > 15.0f);
    
    *cruzamento_pernas = aducao_excessiva || aducao_moderada_com_rotacao;
}
