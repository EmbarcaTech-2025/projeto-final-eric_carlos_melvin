
#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file buzzer.h
 * @brief Interface de controle do buzzer via PWM para Raspberry Pi Pico.
 *
 * Este arquivo define as macros e protótipos das funções para inicialização,
 * controle de alarme e emissão de beep em um buzzer conectado ao microcontrolador.
 */

// ==================== Definições de Hardware ====================

/**
 * @def BUZZER_PIN
 * @brief Pino GPIO utilizado para o buzzer.
 *
 * Pino de GPIO do Buzzer da BitDogLab.
 */
#define BUZZER_PIN 21

/**
 * @def BUZZER_FREQ
 * @brief Frequência do sinal PWM em Hz para o buzzer.
 *
 * Recomenda-se valores entre 2kHz e 5kHz para buzzers piezoelétricos comuns.
 */
#define BUZZER_FREQ 4000

// ==================== Protótipos das Funções ====================

/**
 * @brief Inicializa o buzzer configurando o pino e o PWM.
 *
 * Deve ser chamada uma vez na inicialização do sistema para preparar o buzzer.
 */
void buzzer_init(void);

/**
 * @brief Ativa o alarme sonoro no buzzer (som contínuo).
 *
 * Liga o buzzer em duty cycle de 50%, produzindo um som audível.
 */
void buzzer_alarm_on(void);

/**
 * @brief Desativa o alarme sonoro do buzzer.
 *
 * Silencia o buzzer desligando o PWM.
 */
void buzzer_alarm_off(void);

/**
 * @brief Emite um beep curto no buzzer.
 *
 * Útil para indicar eventos como início ou fim de calibração.
 */
void buzzer_beep(void);

#ifdef __cplusplus
}
#endif

#endif // BUZZER_H