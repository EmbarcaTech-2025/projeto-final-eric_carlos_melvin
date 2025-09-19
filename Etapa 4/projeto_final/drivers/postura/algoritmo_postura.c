// ======================================================================
//  Arquivo: algoritmo_postura.c
//  Descrição: Implementação dos algoritmos matemáticos para análise postural com quaternions
//  Autor: [Seu Nome]
//  Data: [Data de modificação]
// ======================================================================

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#include "algoritmo_postura.h"
#include <stdio.h>
#include <math.h>   // Funções matemáticas: cosf, sinf, atan2f, asinf, fabsf, fmaxf, fminf

// ----------------------------------------------------------------------
// Função interna: Normaliza um quaternion (garante unidade numérica)
// ----------------------------------------------------------------------
static Quaternion quaternion_normalize(Quaternion q)
{
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm > 0.0f) {
        float inv = 1.0f / norm;
        q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
    } else {
        // Caso degenerado, retorna identidade
        q.w = 1.0f; q.x = q.y = q.z = 0.0f;
    }
    return q;
}

// ----------------------------------------------------------------------
// Função interna: Normalização com precisão dupla
// ----------------------------------------------------------------------
static Quaternion quaternion_normalize_double(Quaternion q)
{
    double norm = sqrt((double)q.w*q.w + (double)q.x*q.x + (double)q.y*q.y + (double)q.z*q.z);
    if (norm > 0.0) {
        double inv = 1.0 / norm;
        q.w *= (float)inv;
        q.x *= (float)inv;
        q.y *= (float)inv;
        q.z *= (float)inv;
    } else {
        q.w = 1.0f; q.x = q.y = q.z = 0.0f;
    }
    return q;
}

// ----------------------------------------------------------------------
// Conversão de ângulos de Euler (roll, pitch, yaw) para quaternion
// ----------------------------------------------------------------------
Quaternion euler_to_quaternion(float roll, float pitch, float yaw)
{
    // roll = rotação em torno de X, pitch em torno de Y, yaw em torno de Z
    // Ordem de composição: ZYX (yaw * pitch * roll)
    Quaternion q;
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    // Fórmula padrão (ZYX composition)
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    // Garante unidade numérica
    return quaternion_normalize(q);
}

// ----------------------------------------------------------------------
// Multiplicação de dois quaternions (q1 * q2)
// ----------------------------------------------------------------------
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2)
{
    Quaternion q;
    q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return quaternion_normalize(q); // Normaliza para manter unidade
}

// ----------------------------------------------------------------------
// Conjugado de um quaternion (inverso se unitário)
// ----------------------------------------------------------------------
Quaternion quaternion_conjugate(Quaternion q)
{
    Quaternion qc;
    qc.w = q.w;
    qc.x = -q.x;
    qc.y = -q.y;
    qc.z = -q.z;
    return qc;
}

// ----------------------------------------------------------------------
// Calcula o quaternion relativo (tronco -> coxa)
// q_rel = q_tronco^{-1} * q_coxa
// ----------------------------------------------------------------------
Quaternion relative_quaternion(Quaternion q_tronco, Quaternion q_coxa)
{
    // Garante unitários para que conjugado == inverso
    q_tronco = quaternion_normalize(q_tronco);
    q_coxa   = quaternion_normalize(q_coxa);

    Quaternion q_inv = quaternion_conjugate(q_tronco); // Inverso (unitário)
    Quaternion q_rel = quaternion_multiply(q_inv, q_coxa);
    // Normaliza resultado para estabilidade numérica
    return quaternion_normalize(q_rel);
}

// ----------------------------------------------------------------------
// Converte quaternion para ângulos articulares do quadril (flexão, adução, rotação)
// ----------------------------------------------------------------------
void quaternion_to_hip_angles(Quaternion q, float *flexao, float *aducao, float *rotacao)
{
    // Garante unidade
    q = quaternion_normalize(q);

    // roll (X) - flexão/extensão
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (Y) - rotação interna/externa (clamp [-1,1] para evitar NaN)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    sinp = fmaxf(-1.0f, fminf(1.0f, sinp));
    float pitch = asinf(sinp);

    // yaw (Z) - adução/abdução
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    // Mapear para saídas anatômicas:
    // Flexão para frente é POSITIVA (roll positivo => flexão positiva)
    *flexao = -roll;
    *rotacao = pitch;
    *aducao = yaw;
}
