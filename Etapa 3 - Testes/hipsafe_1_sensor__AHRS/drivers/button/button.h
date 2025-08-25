#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include "pico/stdlib.h"

// Definições dos pinos GPIO conectados aos botões A e B
#define BUTTON_A 5   // Pino do botão A
#define BUTTON_B 6   // Pino do botão B

// Variáveis globais para indicar se os botões foram pressionados
extern volatile bool button_a_pressed; // true se o botão A foi pressionado
extern volatile bool button_b_pressed; // true se o botão B foi pressionado

/**
 * Inicializa os botões A e B para uso com interrupção.
 * 
 * Configura os pinos como entrada com pull-up e ativa interrupção na borda de descida,
 * associando a função de callback para tratamento do evento.
 */
void setup_buttons(void);

#endif