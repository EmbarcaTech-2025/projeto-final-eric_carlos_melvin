#ifndef ANALISE_POSTURAL_SENSOR_H
#define ANALISE_POSTURAL_SENSOR_H

#include "Fusion/Fusion.h"
#include <stdbool.h>

#define NUM_SENSORS 3

// Estruturas para calibração
typedef struct {
    FusionVector offset;
    FusionVector sensitivity;
    FusionMatrix misalignment;
    bool calibrated;
} CalibrationData;

// Variáveis globais externas para múltiplos sensores
extern float g_roll[NUM_SENSORS], g_pitch[NUM_SENSORS], g_yaw[NUM_SENSORS];
extern CalibrationData accel_cal[NUM_SENSORS], gyro_cal[NUM_SENSORS];
extern FusionAhrs ahrs[NUM_SENSORS];
extern bool fusion_initialized;

// Funções principais
bool requisitaPosicoes(void);
bool comparaPosicoes(void);
bool posicaoPerigosa(void);

// Funções auxiliares
void convert_mpu6050_data(int16_t raw_accel[3], int16_t raw_gyro[3], FusionVector *accel, FusionVector *gyro, int sensor_id);
FusionVector apply_calibration(FusionVector raw_data, CalibrationData *cal);

// Novas funções para múltiplos sensores
void calibrate_all_sensors(void);
void initialize_all_fusion_systems(void);
bool read_all_sensor_positions(void);

#endif // ANALISE_POSTURAL_SENSOR_H