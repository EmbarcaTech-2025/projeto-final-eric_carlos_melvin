
#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file button.h
 * @brief Interface de controle dos botões A e B para Raspberry Pi Pico.
 *
 * Este arquivo define as macros, variáveis globais e protótipos das funções
 * para inicialização e detecção de eventos dos botões físicos.
 */

// ==================== Definições de Hardware ====================

/**
 * @def BUTTON_A
 * @brief Pino GPIO utilizado para o botão A.
 *
 */
#define BUTTON_A 5

/**
 * @def BUTTON_B
 * @brief Pino GPIO utilizado para o botão B.
 *
 */
#define BUTTON_B 6

// ==================== Variáveis Globais ====================

/**
 * @brief Indica se o botão A foi pressionado.
 *
 * Deve ser lida e resetada pela aplicação principal após o tratamento do evento.
 */
extern volatile bool button_a_pressed;

/**
 * @brief Indica se o botão B foi pressionado.
 *
 * Deve ser lida e resetada pela aplicação principal após o tratamento do evento.
 */
extern volatile bool button_b_pressed;

// ==================== Protótipos das Funções ====================

/**
 * @brief Inicializa os botões A e B para uso com interrupção e pull-up.
 *
 * Configura os pinos como entrada com resistor de pull-up interno e ativa
 * a interrupção na borda de descida, associando a função de callback para
 * tratamento dos eventos de pressionamento.
 */
void setup_buttons(void);

#ifdef __cplusplus
}
#endif

#endif // BUTTONS_H