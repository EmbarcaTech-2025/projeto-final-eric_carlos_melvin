/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *  Adapted for Raspberry Pi Pico W by EmbarcaTech Team                      *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#ifndef __CALIBRATE_H
#define __CALIBRATE_H

#include "mpu9250_i2c.h"

// Estrutura para armazenar dados de calibração
typedef struct {
    struct {
        float x, y, z;
    } mag_offset;
    struct {
        float x, y, z;
    } mag_scale;
    struct {
        float x, y, z;
    } accel_offset;
    struct {
        float x, y, z;
    } accel_scale_lo;
    struct {
        float x, y, z;
    } accel_scale_hi;
    struct {
        float x, y, z;
    } gyro_bias_offset;
} calibration_t;

// Funções de calibração
void calibrate_gyro(mpu9250_t *mpu, calibration_t *cal);
void calibrate_accel(mpu9250_t *mpu, calibration_t *cal);
void calibrate_mag(mpu9250_t *mpu, calibration_t *cal);

// Função para aplicar calibração aos dados lidos
void apply_calibration(mpu9250_data_t *data, const calibration_t *cal);

// Função para imprimir valores de calibração
void print_calibration(const calibration_t *cal);

#endif