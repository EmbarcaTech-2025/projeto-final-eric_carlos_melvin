
/************************************************************
 * Módulo de Controle dos Botões A e B                      *
 *                                                          *
 * Este arquivo implementa funções para inicialização,       *
 * detecção e tratamento de eventos dos botões A e B         *
 * utilizando interrupções e debounce por software.          *
 ************************************************************/

#include "button.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/types.h"

// ==================== Variáveis Globais ====================

/**
 * @brief Indica se o botão A foi pressionado.
 *
 * Deve ser lida e resetada pela aplicação principal após o tratamento do evento.
 */
volatile bool button_a_pressed = false;

/**
 * @brief Indica se o botão B foi pressionado.
 *
 * Deve ser lida e resetada pela aplicação principal após o tratamento do evento.
 */
volatile bool button_b_pressed = false;

// ==================== Parâmetros de Debounce ====================

/**
 * @brief Tempo mínimo (em ms) entre dois acionamentos válidos do mesmo botão.
 *
 * Evita múltiplos disparos devido a ruído mecânico (bounce).
 */
#define DEBOUNCE_MS 200

// ==================== Variáveis de Controle Interno ====================

/**
 * @brief Armazena o timestamp do último acionamento válido do botão A.
 */
static uint32_t last_a_time = 0;

/**
 * @brief Armazena o timestamp do último acionamento válido do botão B.
 */
static uint32_t last_b_time = 0;

// ==================== Funções de Callback e Inicialização ====================

/**
 * @brief Callback de interrupção para os botões A e B.
 *
 * Esta função é chamada automaticamente quando ocorre uma borda de descida
 * (pressionamento) em qualquer um dos botões. Ela verifica qual botão gerou
 * a interrupção, aplica o debounce por software e sinaliza o evento para a aplicação.
 *
 * @param gpio   Número do pino GPIO que gerou a interrupção
 * @param events Tipo de evento ocorrido (espera-se GPIO_IRQ_EDGE_FALL)
 */
void button_callback(uint gpio, uint32_t events)
{
    // Obtém o tempo atual em milissegundos desde o boot
    uint32_t now = to_ms_since_boot(get_absolute_time());

    // Tratamento do botão A
    if (gpio == BUTTON_A)
    {
        // Verifica se passou o tempo de debounce desde o último acionamento
        if (now - last_a_time > DEBOUNCE_MS)
        {
            button_a_pressed = true;   // Sinaliza evento para a aplicação
            last_a_time = now;         // Atualiza timestamp do último acionamento
        }
    }

    // Tratamento do botão B
    if (gpio == BUTTON_B)
    {
        if (now - last_b_time > DEBOUNCE_MS)
        {
            button_b_pressed = true;
            last_b_time = now;
        }
    }
}

/**
 * @brief Inicializa os botões A e B para uso com interrupção e pull-up.
 *
 * Configura os pinos dos botões como entrada com resistor de pull-up interno
 * e ativa a interrupção na borda de descida, associando a função de callback
 * para tratamento dos eventos de pressionamento.
 */
void setup_buttons(void)
{
    // ==================== Configuração do Botão A ====================
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);         // Define como entrada
    gpio_pull_up(BUTTON_A);                  // Ativa pull-up interno
    gpio_set_irq_enabled_with_callback(
        BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &button_callback); // Interrupção borda de descida

    // ==================== Configuração do Botão B ====================
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_set_irq_enabled_with_callback(
        BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &button_callback);
}