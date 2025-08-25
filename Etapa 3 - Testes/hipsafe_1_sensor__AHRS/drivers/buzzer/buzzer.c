// Funções de controle e reprodução de áudio nos buzzers via PWM
#include <stdio.h>
#include "buzzer.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"
#include "pico/types.h"

// Variáveis estáticas para controle interno
static bool buzzer_a_initialized = false;
static bool buzzer_b_initialized = false;
static bool alarm_is_on = false; // Variável para rastrear estado do alarme

/**
 * Inicializa o PWM para um buzzer em um pino específico.
 * 
 * @param pin      Pino GPIO conectado ao buzzer
 * @param freq_hz  Frequência do PWM em Hz (deve ser igual à taxa de amostragem do áudio)
 * 
 * Configura o pino para função PWM, define a frequência desejada e inicializa o PWM desligado.
 */
void pwm_init_buzzer(uint pin, uint32_t freq_hz) 
{
    gpio_set_function(pin, GPIO_FUNC_PWM);                  // Configura o pino como saída PWM
    uint slice_num = pwm_gpio_to_slice_num(pin);            // Obtém o slice PWM correspondente ao pino

    uint32_t clock = clock_get_hz(clk_sys);                 // Obtém a frequência do clock do sistema
    uint32_t top = clock / freq_hz - 1;                     // Calcula o valor de "top (wrap)" para a frequência desejada
    pwm_config config = pwm_get_default_config();           // Obtém a configuração padrão do PWM
    pwm_config_set_wrap(&config, top);                      // Define o valor de "top" (resolução do PWM)
    pwm_init(slice_num, &config, true);                     // Inicializa o PWM com a configuração definida
    pwm_set_gpio_level(pin, BUZZER_LEVEL_OFF);              // Garante que o PWM inicia desligado
    
    // Marca como inicializado
    if (pin == BUZZER_A) {
        buzzer_a_initialized = true;
    } else if (pin == BUZZER_B) {
        buzzer_b_initialized = true;
    }
}

/**
 * Inicializa o sistema de buzzer com configurações padrão.
 * Configura o BUZZER_A com a frequência padrão.
 */
void buzzer_init(void)
{
    pwm_init_buzzer(BUZZER_A, BUZZER_FREQ);
}

/**
 * Liga o buzzer com nível especificado.
 * 
 * @param pin   Pino GPIO do buzzer
 * @param level Nível PWM (0-255)
 */
void buzzer_on(uint pin, uint16_t level)
{
    // Verifica se o buzzer foi inicializado
    if ((pin == BUZZER_A && !buzzer_a_initialized) || 
        (pin == BUZZER_B && !buzzer_b_initialized)) {
        pwm_init_buzzer(pin, BUZZER_FREQ);
    }
    
    // Limita o nível ao máximo permitido
    if (level > BUZZER_LEVEL_HIGH) {
        level = BUZZER_LEVEL_HIGH;
    }
    
    pwm_set_gpio_level(pin, level);
}

/**
 * Desliga o buzzer.
 * 
 * @param pin Pino GPIO do buzzer
 */
void buzzer_off(uint pin)
{
    pwm_set_gpio_level(pin, BUZZER_LEVEL_OFF);
}

/**
 * Liga o alarme principal (BUZZER_A) com nível médio.
 */
void buzzer_alarm_on(void)
{
    buzzer_on(BUZZER_A, BUZZER_LEVEL_MED);
    alarm_is_on = true; // Marca que o alarme está ligado
}

/**
 * Desliga o alarme principal (BUZZER_A).
 */
void buzzer_alarm_off(void)
{
    buzzer_off(BUZZER_A);
    alarm_is_on = false; // Marca que o alarme está desligado
}

/**
 * Verifica se o buzzer está ligado.
 * 
 * @param pin Pino GPIO do buzzer
 * @return true se o buzzer estiver ligado, false caso contrário
 */
bool buzzer_is_on(uint pin)
{
    // Usa gpio_get_out_level para verificar se o GPIO está em nível alto
    return gpio_get_out_level(pin);
}

/**
 * Verifica se o alarme principal está ligado.
 * 
 * @return true se o alarme estiver ligado, false caso contrário
 */
bool buzzer_alarm_is_on(void)
{
    return alarm_is_on; // Retorna o estado rastreado da variável
}

/**
 * Define o nível do buzzer sem ligá-lo ou desligá-lo.
 * 
 * @param pin   Pino GPIO do buzzer
 * @param level Nível PWM (0-255)
 */
void buzzer_set_level(uint pin, uint16_t level)
{
    // Verifica se o buzzer foi inicializado
    if ((pin == BUZZER_A && !buzzer_a_initialized) || 
        (pin == BUZZER_B && !buzzer_b_initialized)) {
        pwm_init_buzzer(pin, BUZZER_FREQ);
    }
    
    // Limita o nível ao máximo permitido
    if (level > BUZZER_LEVEL_HIGH) {
        level = BUZZER_LEVEL_HIGH;
    }
    
    pwm_set_gpio_level(pin, level);
}

/**
 * Obtém o nível atual do buzzer.
 * 
 * @param pin Pino GPIO do buzzer
 * @return Nível PWM atual (0 para desligado, >0 para ligado)
 */
uint16_t buzzer_get_level(uint pin)
{
    // Como não há função para ler o nível PWM atual, 
    // retorna um valor baseado no estado do GPIO
    return gpio_get_out_level(pin) ? BUZZER_LEVEL_MED : BUZZER_LEVEL_OFF;
}