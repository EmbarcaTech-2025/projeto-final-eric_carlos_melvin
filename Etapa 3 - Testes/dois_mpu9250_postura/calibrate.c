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

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "calibrate.h"
#include "mpu9250_i2c.h"

static const char *TAG = "calibrate";

// Função auxiliar para aguardar início da calibração
static void wait_for_start(void)
{
    for (int i = 10; i >= 0; i -= 1)
    {
        printf("Iniciando em %d segundos     \r", i);
        fflush(stdout);
        sleep_ms(1000);
    }
    printf("\n");
}

/**
 * CALIBRAÇÃO DO GIROSCÓPIO
 * 
 * Calibra o giroscópio calculando o bias quando o dispositivo está parado.
 * O dispositivo DEVE permanecer imóvel durante esta calibração.
 */

const int NUM_GYRO_READS = 5000;

void calibrate_gyro(mpu9250_t *mpu, calibration_t *cal)
{
    printf("--- CALIBRAÇÃO DO GIROSCÓPIO ---\n");
    printf("Mantenha o MPU9250 completamente parado. Calculando bias do giroscópio...\n");
    wait_for_start();

    float gyro_sum[3] = {0.0, 0.0, 0.0};
    
    for (int i = 0; i < NUM_GYRO_READS; i++)
    {
        float gyro[3], accel[3], temp;
        mpu9250_read_motion(mpu, accel, gyro, &temp);

        gyro_sum[0] += gyro[0];
        gyro_sum[1] += gyro[1];
        gyro_sum[2] += gyro[2];

        // Pequeno delay para não sobrecarregar o barramento I2C
        if (i % 100 == 0) {
            printf("Progresso: %d/%d\r", i, NUM_GYRO_READS);
            fflush(stdout);
        }
        sleep_us(100);
    }

    // Calcula a média e inverte o sinal para obter o offset de correção
    cal->gyro_bias_offset.x = -(gyro_sum[0] / NUM_GYRO_READS);
    cal->gyro_bias_offset.y = -(gyro_sum[1] / NUM_GYRO_READS);
    cal->gyro_bias_offset.z = -(gyro_sum[2] / NUM_GYRO_READS);

    printf("\nCalibração do giroscópio concluída!\n");
    printf("Offsets: X=%.6f, Y=%.6f, Z=%.6f °/s\n\n", 
           cal->gyro_bias_offset.x, cal->gyro_bias_offset.y, cal->gyro_bias_offset.z);
}

/**
 * CALIBRAÇÃO DO ACELERÔMETRO
 * 
 * Calibra o acelerômetro rotacionando o dispositivo com cada eixo apontando para cima e para baixo.
 * O eixo apontado para cima/baixo será calibrado contra a gravidade (deve estar vertical).
 * Os outros dois eixos estarão perpendiculares à gravidade e lerão próximo de zero.
 */

#define NUM_ACCEL_READS (1000.0)

#define X_AXIS (0)
#define Y_AXIS (1)
#define Z_AXIS (2)
static const char *axes[] = {"X", "Y", "Z"};

#define DIR_UP (0)
#define DIR_DOWN (1)
static const char *directions[] = {"para CIMA", "para BAIXO"};

static struct {
    float x, y, z;
} offset = {0, 0, 0};

static struct {
    float x, y, z;
} scale_lo = {0, 0, 0};

static struct {
    float x, y, z;
} scale_hi = {0, 0, 0};

/**
 * Lê dados do acelerômetro de forma síncrona e coleta valores de offset e escala.
 */
static void calibrate_accel_axis(mpu9250_t *mpu, int axis, int dir)
{
    printf("Lendo valores - mantenha imóvel...\n");
    
    for (int i = 0; i < NUM_ACCEL_READS; i++)
    {
        float accel[3], gyro[3], temp;
        mpu9250_read_motion(mpu, accel, gyro, &temp);

        if (axis == X_AXIS)
        {
            if (dir == DIR_UP)
            {
                scale_lo.x += accel[0];
            }
            else
            {
                scale_hi.x += accel[0];
            }
            // Os outros eixos são usados para offset
            offset.y += accel[1];
            offset.z += accel[2];
        }
        else if (axis == Y_AXIS)
        {
            if (dir == DIR_UP)
            {
                scale_lo.y += accel[1];
            }
            else
            {
                scale_hi.y += accel[1];
            }
            // Os outros eixos são usados para offset
            offset.x += accel[0];
            offset.z += accel[2];
        }
        else if (axis == Z_AXIS)
        {
            if (dir == DIR_UP)
            {
                scale_lo.z += accel[2];
            }
            else
            {
                scale_hi.z += accel[2];
            }
            // Os outros eixos são usados para offset
            offset.x += accel[0];
            offset.y += accel[1];
        }

        if (i % 100 == 0) {
            printf("Progresso: %d/%d\r", i, (int)NUM_ACCEL_READS);
            fflush(stdout);
        }
        sleep_ms(5);
    }
    printf("\n");
}

/**
 * Configura a próxima captura para um eixo e direção (cima/baixo).
 */
static void run_next_capture(mpu9250_t *mpu, int axis, int dir)
{
    printf("Posicione o eixo %s apontando %s.\n", axes[axis], directions[dir]);
    wait_for_start();
    calibrate_accel_axis(mpu, axis, dir);
}

void calibrate_accel(mpu9250_t *mpu, calibration_t *cal)
{
    printf("--- CALIBRAÇÃO DO ACELERÔMETRO ---\n");
    printf("Você precisará orientar o dispositivo em 6 posições diferentes.\n");
    printf("Cada eixo deve apontar para cima e para baixo.\n\n");

    // Zera as variáveis de acumulação
    offset.x = offset.y = offset.z = 0;
    scale_lo.x = scale_lo.y = scale_lo.z = 0;
    scale_hi.x = scale_hi.y = scale_hi.z = 0;

    run_next_capture(mpu, X_AXIS, DIR_UP);
    run_next_capture(mpu, X_AXIS, DIR_DOWN);
    run_next_capture(mpu, Y_AXIS, DIR_UP);
    run_next_capture(mpu, Y_AXIS, DIR_DOWN);
    run_next_capture(mpu, Z_AXIS, DIR_UP);
    run_next_capture(mpu, Z_AXIS, DIR_DOWN);

    // Calcula os valores finais de calibração
    cal->accel_offset.x = offset.x / (NUM_ACCEL_READS * 4.0);
    cal->accel_offset.y = offset.y / (NUM_ACCEL_READS * 4.0);
    cal->accel_offset.z = offset.z / (NUM_ACCEL_READS * 4.0);
    
    cal->accel_scale_lo.x = scale_lo.x / NUM_ACCEL_READS;
    cal->accel_scale_lo.y = scale_lo.y / NUM_ACCEL_READS;
    cal->accel_scale_lo.z = scale_lo.z / NUM_ACCEL_READS;
    
    cal->accel_scale_hi.x = scale_hi.x / NUM_ACCEL_READS;
    cal->accel_scale_hi.y = scale_hi.y / NUM_ACCEL_READS;
    cal->accel_scale_hi.z = scale_hi.z / NUM_ACCEL_READS;

    printf("Calibração do acelerômetro concluída!\n");
    printf("Offset: X=%.6f, Y=%.6f, Z=%.6f g\n",
           cal->accel_offset.x, cal->accel_offset.y, cal->accel_offset.z);
    printf("Scale Lo: X=%.6f, Y=%.6f, Z=%.6f g\n",
           cal->accel_scale_lo.x, cal->accel_scale_lo.y, cal->accel_scale_lo.z);
    printf("Scale Hi: X=%.6f, Y=%.6f, Z=%.6f g\n\n",
           cal->accel_scale_hi.x, cal->accel_scale_hi.y, cal->accel_scale_hi.z);
}

/**
 * CALIBRAÇÃO DO MAGNETÔMETRO
 * 
 * Após iniciar a calibração, você deve mover o sensor em torno de todos os eixos.
 * O objetivo é encontrar os extremos (min/max) dos valores x, y, z para calcular
 * os valores de offset e escala.
 *
 * Cálculos baseados em:
 * http://www.camelsoftware.com/2016/03/13/imu-maths-calculate-orientation-pt3/
 */

void calibrate_mag(mpu9250_t *mpu, calibration_t *cal)
{
    if (!mpu->mag_enabled) {
        printf("ERRO: Magnetômetro não está habilitado!\n");
        return;
    }

    struct {
        float x, y, z;
    } v_min = {9.9e99, 9.9e99, 9.9e99};

    struct {
        float x, y, z;
    } v_max = {-9.9e99, -9.9e99, -9.9e99};

    const int NUM_MAG_READS = 2000;

    printf("--- CALIBRAÇÃO DO MAGNETÔMETRO ---\n");
    printf("Rotacione o magnetômetro em torno de todos os 3 eixos,\n");
    printf("até que os valores min e max não mudem mais.\n\n");

    printf("    x        y        z      min x     min y     min z     max x     max y     max z\n");
    printf("====================================================================================\n");
    
    for (int i = 0; i < NUM_MAG_READS; i++)
    {
        float mag[3];
        mpu9250_read_mag(mpu, mag);
        
        v_min.x = MIN(v_min.x, mag[0]);
        v_min.y = MIN(v_min.y, mag[1]);
        v_min.z = MIN(v_min.z, mag[2]);
        v_max.x = MAX(v_max.x, mag[0]);
        v_max.y = MAX(v_max.y, mag[1]);
        v_max.z = MAX(v_max.z, mag[2]);

        printf(" %7.2f  %7.2f  %7.2f  %7.2f  %7.2f  %7.2f  %7.2f  %7.2f  %7.2f  (%d/%d)\r", 
               mag[0], mag[1], mag[2], 
               v_min.x, v_min.y, v_min.z, 
               v_max.x, v_max.y, v_max.z,
               i + 1, NUM_MAG_READS);
        fflush(stdout);

        sleep_ms(10);
    }

    struct {
        float x, y, z;
    } v_avg = {
        (v_max.x - v_min.x) / 2.0,
        (v_max.y - v_min.y) / 2.0,
        (v_max.z - v_min.z) / 2.0
    };

    float avg_radius = (v_avg.x + v_avg.y + v_avg.z) / 3.0;
    
    cal->mag_offset.x = (v_min.x + v_max.x) / 2.0;
    cal->mag_offset.y = (v_min.y + v_max.y) / 2.0;
    cal->mag_offset.z = (v_min.z + v_max.z) / 2.0;
    
    cal->mag_scale.x = avg_radius / v_avg.x;
    cal->mag_scale.y = avg_radius / v_avg.y;
    cal->mag_scale.z = avg_radius / v_avg.z;

    printf("\n\nCalibração do magnetômetro concluída!\n");
    printf("Offset: X=%.6f, Y=%.6f, Z=%.6f µT\n",
           cal->mag_offset.x, cal->mag_offset.y, cal->mag_offset.z);
    printf("Scale: X=%.6f, Y=%.6f, Z=%.6f\n\n",
           cal->mag_scale.x, cal->mag_scale.y, cal->mag_scale.z);
}

/**
 * Aplica a calibração aos dados lidos dos sensores
 */
void apply_calibration(mpu9250_data_t *data, const calibration_t *cal)
{
    // Aplica calibração do giroscópio (offset)
    data->gyro[0] += cal->gyro_bias_offset.x;
    data->gyro[1] += cal->gyro_bias_offset.y;
    data->gyro[2] += cal->gyro_bias_offset.z;

    // Aplica calibração do acelerômetro (offset e escala)
    // Fórmula: y = m * x + c
    for (int i = 0; i < 3; i++) {
        float *accel_val = &data->accel[i];
        float offset, scale_lo, scale_hi;
        
        if (i == 0) {
            offset = cal->accel_offset.x;
            scale_lo = cal->accel_scale_lo.x;
            scale_hi = cal->accel_scale_hi.x;
        } else if (i == 1) {
            offset = cal->accel_offset.y;
            scale_lo = cal->accel_scale_lo.y;
            scale_hi = cal->accel_scale_hi.y;
        } else {
            offset = cal->accel_offset.z;
            scale_lo = cal->accel_scale_lo.z;
            scale_hi = cal->accel_scale_hi.z;
        }
        
        // Remove offset
        *accel_val -= offset;
        
        // Aplica escala baseada no sinal
        if (*accel_val >= 0) {
            *accel_val = *accel_val / scale_hi;
        } else {
            *accel_val = *accel_val / scale_lo;
        }
    }

    // Aplica calibração do magnetômetro (offset e escala)
    data->mag[0] = (data->mag[0] - cal->mag_offset.x) * cal->mag_scale.x;
    data->mag[1] = (data->mag[1] - cal->mag_offset.y) * cal->mag_scale.y;
    data->mag[2] = (data->mag[2] - cal->mag_offset.z) * cal->mag_scale.z;
}

/**
 * Imprime os valores de calibração em formato C
 */
void print_calibration(const calibration_t *cal)
{
    printf("\n=== VALORES DE CALIBRAÇÃO ===\n");
    printf("// Copie estes valores para seu código:\n\n");
    printf("calibration_t cal = {\n");
    printf("    .mag_offset = {.x = %.6f, .y = %.6f, .z = %.6f},\n",
           cal->mag_offset.x, cal->mag_offset.y, cal->mag_offset.z);
    printf("    .mag_scale = {.x = %.6f, .y = %.6f, .z = %.6f},\n",
           cal->mag_scale.x, cal->mag_scale.y, cal->mag_scale.z);
    printf("    .accel_offset = {.x = %.6f, .y = %.6f, .z = %.6f},\n",
           cal->accel_offset.x, cal->accel_offset.y, cal->accel_offset.z);
    printf("    .accel_scale_lo = {.x = %.6f, .y = %.6f, .z = %.6f},\n",
           cal->accel_scale_lo.x, cal->accel_scale_lo.y, cal->accel_scale_lo.z);
    printf("    .accel_scale_hi = {.x = %.6f, .y = %.6f, .z = %.6f},\n",
           cal->accel_scale_hi.x, cal->accel_scale_hi.y, cal->accel_scale_hi.z);
    printf("    .gyro_bias_offset = {.x = %.6f, .y = %.6f, .z = %.6f}\n",
           cal->gyro_bias_offset.x, cal->gyro_bias_offset.y, cal->gyro_bias_offset.z);
    printf("};\n\n");
}
