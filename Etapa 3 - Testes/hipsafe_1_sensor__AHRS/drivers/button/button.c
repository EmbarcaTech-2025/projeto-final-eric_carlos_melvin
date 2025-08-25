// Funções de controle dos botões A e B da BitDogLab
#include "button.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pico/types.h"

// Variáveis globais voláteis para indicar se os botões foram pressionados
volatile bool button_a_pressed = false;
volatile bool button_b_pressed = false;

#define DEBOUNCE_MS 200 // Tempo de debounce em milissegundos

// Variáveis estáticas para armazenar o último tempo de acionamento de cada botão
static uint32_t last_a_time = 0;
static uint32_t last_b_time = 0;

/**
 * Callback de interrupção dos botões.
 * 
 * @param gpio   Número do pino GPIO que gerou a interrupção
 * @param events Tipo de evento ocorrido (borda de descida)
 * 
 * Verifica qual botão foi pressionado, aplica debounce e sinaliza a ação.
 */
void button_callback(uint gpio, uint32_t events) 
{
    uint32_t now = to_ms_since_boot(get_absolute_time()); // Obtém tempo atual em ms desde o boot
    if (gpio == BUTTON_A) 
    {
        if (now - last_a_time > DEBOUNCE_MS)             // Verifica debounce do botão A
        {                                                
            button_a_pressed = true;                     // Sinaliza que o botão A foi pressionado
            last_a_time = now;                           // Atualiza o tempo do último acionamento
        }
    }
    if (gpio == BUTTON_B)                                
    {
        if (now - last_b_time > DEBOUNCE_MS)             // Verifica debounce do botão B 
        {           
            button_b_pressed = true;                     // Sinaliza que o botão B foi pressionado
            last_b_time = now;                           // Atualiza o tempo do último acionamento
        }
    }
}

/**
 * Inicializa os botões A e B para uso com interrupção.
 * 
 * Configura os pinos como entrada com pull-up e ativa interrupção na borda de descida,
 * associando a função de callback para tratamento do evento.
 */
void setup_buttons() 
{
    gpio_init(BUTTON_A);                                         // Inicializa pino do botão A
    gpio_set_dir(BUTTON_A, GPIO_IN);                             // Define como entrada
    gpio_pull_up(BUTTON_A);                                      // Ativa pull-up interno
    gpio_set_irq_enabled_with_callback(BUTTON_A, GPIO_IRQ_EDGE_FALL, true, &button_callback); // Interrupção borda de descida

    gpio_init(BUTTON_B);                                         // Inicializa pino do botão B
    gpio_set_dir(BUTTON_B, GPIO_IN);                             // Define como entrada
    gpio_pull_up(BUTTON_B);                                      // Ativa pull-up interno
    gpio_set_irq_enabled_with_callback(BUTTON_B, GPIO_IRQ_EDGE_FALL, true, &button_callback); // Interrupção borda de descida
}