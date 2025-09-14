#include "bogus.h"
#include "analise_postural.h"

// RELATIVO AOS PERIFÉRICOS EM GERAL
void setup_buttons() {printf("Button bogus inicializado\n");};
void sd_card_init() {printf("SD_card Inicializado\n");};
void stdio_init_all(){printf("stdio bogus inicialisado\n");}; // Inicializa UART/USB para debug

// RELATIVO AO DRIVER MPU6050
void mpu6050_setup_i2c(mpu6050_t *mpu){};
void mpu6050_reset(mpu6050_t *mpu){};
uint8_t mpu6050_get_accel_range(mpu6050_t *mpu){return 0;}; // Returns 0=±2g, 1=±4g, 2=±8g, 3=±16g
void mpu6050_set_accel_range(mpu6050_t *mpu, uint8_t range){}; // 0=±2g, 1=±4g, 2=±8g, 3=±16g
uint8_t mpu6050_get_gyro_range(mpu6050_t *mpu){return 0;}; // Returns 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
void mpu6050_set_gyro_range(mpu6050_t *mpu, uint8_t range){}; // 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
void mpu6050_read_raw(mpu6050_t *mpu, int16_t accel[3], int16_t gyro[3], int16_t *temp){};

// RELATIVO AO DRIVER DE BUZZER
void buzzer_init(void) {printf("Buzzer buogus iniciado\n");};


