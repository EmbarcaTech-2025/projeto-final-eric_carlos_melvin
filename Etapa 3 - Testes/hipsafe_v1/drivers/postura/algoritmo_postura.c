#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#include "algoritmo_postura.h"
#include <stdio.h>
#include <math.h>   // for cosf, sinf, atan2f, asinf, fabsf, fmaxf, fminf, copysignf

// --- Helpers para quaternions ---
// Normaliza um quaternion (importante para garantir inverso simples = conjugado)
Quaternion quaternion_normalize(Quaternion q)
{
    float norm = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (norm > 0.0f) {
        float inv = 1.0f / norm;
        q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
    } else {
        // caso degenerado, retorna identidade
        q.w = 1.0f; q.x = q.y = q.z = 0.0f;
    }
    return q;
}

// Versão com precisão dupla para normalização crítica (opcional)
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

// Converte roll (X), pitch (Y), yaw (Z) (em rad) para quaternion.
// Convenção usada: roll = rotação em torno de X, pitch em torno de Y, yaw em torno de Z.
// A composição é (yaw * pitch * roll) — ordem Tait-Bryan Z-Y-X, que é
// compatível com as fórmulas inversas usadas abaixo.
Quaternion euler_to_quaternion(float roll, float pitch, float yaw)
{
    Quaternion q;
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    // fórmula padrão (ZYX composition)
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    // garante unidade numérica
    return quaternion_normalize(q);
}

// Multiplica dois quaternions (q1 * q2) com normalização para evitar drift numérico
Quaternion quaternion_multiply(Quaternion q1, Quaternion q2)
{
    Quaternion q;
    q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return quaternion_normalize(q); // normaliza para manter unidade
}

// Conjugado (inverso se o quaternion for unitário)
Quaternion quaternion_conjugate(Quaternion q)
{
    Quaternion qc;
    qc.w = q.w;
    qc.x = -q.x;
    qc.y = -q.y;
    qc.z = -q.z;
    return qc;
}

// Calcula quaternion relativo (tronco -> coxa) com normalização robusta:
// q_rel = q_tronco^{-1} * q_coxa
Quaternion relative_quaternion(Quaternion q_tronco, Quaternion q_coxa)
{
    // garante unitários para que conjugado == inverso
    q_tronco = quaternion_normalize(q_tronco);
    q_coxa   = quaternion_normalize(q_coxa);

    Quaternion q_inv = quaternion_conjugate(q_tronco); // inverso (unitário)
    Quaternion q_rel = quaternion_multiply(q_inv, q_coxa);
    // normaliza resultado para estabilidade numérica
    return quaternion_normalize(q_rel);
}

// Converte quaternion para ângulos de Euler usando a mesma convenção (ZYX) que
// euler_to_quaternion usa. As fórmulas abaixo extraem rotações em X (roll),
// Y (pitch) e Z (yaw). Nomes adaptados para anatomia do quadril:
// X: flexão(+ frente) / extensão(- trás)
// Y: rotação interna(+)/externa(-)
// Z: adução(+)/abdução(-)
//
// Observação de sinal: se a sua definição anatômica exigir inversão de
// algum eixo, faça a inversão explicitamente ao final (por exemplo *flexao = -roll).
void quaternion_to_hip_angles(Quaternion q, float *flexao, float *aducao, float *rotacao)
{
    // assegura unidade
    q = quaternion_normalize(q);

    // roll (X) - flexão/extensão
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    float roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (Y) - rotação interna/externa (note clamp por segurança numérica)
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    // clamp para [-1,1] para evitar NaN por erro numérico
    sinp = fmaxf(-1.0f, fminf(1.0f, sinp));
    float pitch = asinf(sinp);

    // yaw (Z) - adução/abdução
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    float yaw = atan2f(siny_cosp, cosy_cosp);

    // Mapear para saídas anatômicas:
    // - aqui decidimos que flexão para frente é POSITIVA (roll positivo => flexão positiva).
    //   Caso a sua montagem física gere o sinal oposto, inverta aqui.
    *flexao = -roll;
    *rotacao = pitch;
    *aducao = yaw;
}

// Extrai ângulos do quadril a partir de q_rel e detecta padrões clínicos
void hip_angles(Quaternion q_rel, float *flexao, float *aducao, float *rotacao,
                bool *rotacao_interna_30, bool *flexao_maior_90, bool *cruzamento_pernas)
{
    // Converte quaternion relativo para ângulos anatômicos (em rad)
    quaternion_to_hip_angles(q_rel, flexao, aducao, rotacao);

    // Converte para graus para facilitar comparações
    const float RAD2DEG = 180.0f / M_PI_F;
    float flexao_graus = (*flexao) * RAD2DEG;
    float aducao_graus = (*aducao) * RAD2DEG;
    float rotacao_graus = (*rotacao) * RAD2DEG;

    // DETECÇÃO DE PADRÕES CLÍNICOS (valores escolhidos e documentados):
    // 1) Rotação interna maior que 30° (assumimos rotação interna = valores positivos em rotacao_graus)
    *rotacao_interna_30 = (rotacao_graus > 30.0f);

    // 2) Flexão para frente maior que 90°
    *flexao_maior_90 = (flexao_graus > 90.0f);

    // 3) Cruzamento de pernas - critério baseado em adução (Z)
    // Critério usado:
    //    - adução excessiva: adução > 35°
    //    - ou adução > 25° combinado com rotação interna > 15°
    bool aducao_excessiva = (aducao_graus > 35.0f);
    bool aducao_moderada_com_rotacao = (aducao_graus > 25.0f && rotacao_graus > 15.0f);

    *cruzamento_pernas = aducao_excessiva || aducao_moderada_com_rotacao;
}
