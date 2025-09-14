#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BUZZER_PIN 21      // Pino do buzzer
#define BUZZER_FREQ 4000   // Frequência do PWM em Hz

/**
 * Inicializa o buzzer no pino especificado.
 */
void buzzer_init(void);

/**
 * Liga o alarme no buzzer.
 */
void buzzer_alarm_on(void);

/**
 * Silencia o alarme no buzzer.
 */
void buzzer_alarm_off(void);

/**
 * Emite um beep curto para indicar início ou fim de calibração.
 */
void buzzer_beep(void);

#ifdef __cplusplus
}
#endif

#endif