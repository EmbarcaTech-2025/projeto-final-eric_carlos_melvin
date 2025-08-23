#ifndef ANALISE_POSTURAL_SENSOR_H
#define ANALISE_POSTURAL_SENSOR_H

#include "Fusion/Fusion.h"
#include <stdbool.h>

// Estruturas para calibração
typedef struct {
    FusionVector offset;
    FusionVector sensitivity;
    FusionMatrix misalignment;
    bool calibrated;
} CalibrationData;

// Variáveis globais externas
extern float g_roll, g_pitch, g_yaw;
extern CalibrationData accel_cal, gyro_cal;
extern FusionAhrs ahrs;
extern bool fusion_initialized;

// Funções principais
bool requisitaPosicoes(void);
bool comparaPosicoes(void);
bool posicaoPerigosa(void);

// Funções auxiliares
void convert_mpu6050_data(int16_t raw_accel[3], int16_t raw_gyro[3], FusionVector *accel, FusionVector *gyro);
FusionVector apply_calibration(FusionVector raw_data, CalibrationData *cal);

#endif // ANALISE_POSTURAL_SENSOR_H