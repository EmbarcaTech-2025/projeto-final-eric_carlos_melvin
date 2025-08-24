#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "pico/types.h"

#define BUZZER_A 21                       // Pino do primeiro buzzer
#define BUZZER_B 10                       // Pino do segundo buzzer

#define BUZZER_FREQ 8000 // Hz
#define BUZZER_LEVEL_OFF 0                // Nível PWM desligado
#define BUZZER_LEVEL_MED 128              // Nível PWM médio
#define BUZZER_LEVEL_HIGH 255             // Nível PWM alto

/**
 * Inicializa o PWM para um buzzer em um pino específico.
 * 
 * @param pin      Pino GPIO conectado ao buzzer
 * @param freq_hz  Frequência do PWM em Hz (deve ser igual à taxa de amostragem do áudio)
 * 
 * Configura o pino para função PWM, define a frequência desejada e inicializa o PWM desligado.
 */
void pwm_init_buzzer(uint pin, uint32_t freq_hz);

/**
 * Inicializa o sistema de buzzer com configurações padrão.
 * Configura o BUZZER_A com a frequência padrão.
 */
void buzzer_init(void);

/**
 * Liga o buzzer com nível especificado.
 * 
 * @param pin   Pino GPIO do buzzer
 * @param level Nível PWM (0-255)
 */
void buzzer_on(uint pin, uint16_t level);

/**
 * Desliga o buzzer.
 * 
 * @param pin Pino GPIO do buzzer
 */
void buzzer_off(uint pin);

/**
 * Liga o alarme principal (BUZZER_A) com nível médio.
 */
void buzzer_alarm_on(void);

/**
 * Desliga o alarme principal (BUZZER_A).
 */
void buzzer_alarm_off(void);

/**
 * Verifica se o buzzer está ligado.
 * 
 * @param pin Pino GPIO do buzzer
 * @return true se o buzzer estiver ligado, false caso contrário
 */
bool buzzer_is_on(uint pin);

/**
 * Verifica se o alarme principal está ligado.
 * 
 * @return true se o alarme estiver ligado, false caso contrário
 */
bool buzzer_alarm_is_on(void);

/**
 * Define o nível do buzzer sem ligá-lo ou desligá-lo.
 * 
 * @param pin   Pino GPIO do buzzer
 * @param level Nível PWM (0-255)
 */
void buzzer_set_level(uint pin, uint16_t level);

/**
 * Obtém o nível atual do buzzer.
 * 
 * @param pin Pino GPIO do buzzer
 * @return Nível PWM atual
 */
uint16_t buzzer_get_level(uint pin);

#endif