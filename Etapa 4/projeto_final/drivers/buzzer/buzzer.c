// Funções de controle e reprodução de áudio nos buzzers via PWM
#include <stdio.h>
#include "buzzer.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

// Variável para rastrear o estado do alarme
static bool alarm_is_on = false;

/**
 * Inicializa o buzzer no pino especificado.
 */
void buzzer_init(void)
{
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);           // Configura o pino como saída PWM
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);     // Obtém o slice PWM correspondente ao pino

    uint32_t clock = clock_get_hz(clk_sys);                 // Obtém a frequência do clock do sistema
    uint32_t top = clock / BUZZER_FREQ - 1;                 // Calcula o valor de "top (wrap)" para a frequência desejada
    pwm_config config = pwm_get_default_config();           // Obtém a configuração padrão do PWM
    pwm_config_set_clkdiv(&config, 4.0f);                   // Ajusta divisor de clock para volume mais alto
    pwm_config_set_wrap(&config, top);                      // Define o valor de "top" (resolução do PWM)
    pwm_init(slice_num, &config, true);                     // Inicializa o PWM com a configuração definida
    pwm_set_gpio_level(BUZZER_PIN, 0);                      // Garante que o PWM inicia desligado
}

/**
 * Liga o alarme no buzzer.
 */
void buzzer_alarm_on(void)
{
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t top = clock / BUZZER_FREQ - 1;
    pwm_set_wrap(slice_num, top);
    pwm_set_gpio_level(BUZZER_PIN, top / 2); // 50% duty cycle para volume alto
    alarm_is_on = true;
}

/**
 * Silencia o alarme no buzzer.
 */
void buzzer_alarm_off(void)
{
    pwm_set_gpio_level(BUZZER_PIN, 0); // Desliga o PWM
    alarm_is_on = false;
}

/**
 * Emite um beep curto para indicar início ou fim de calibração.
 */
void buzzer_beep(void)
{
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    uint32_t clock = clock_get_hz(clk_sys);
    uint32_t top = clock / BUZZER_FREQ - 1;
    pwm_set_wrap(slice_num, top);
    pwm_set_gpio_level(BUZZER_PIN, top / 2); // 50% duty cycle para volume alto
    sleep_ms(200);                           // Mantém o beep por 200ms
    pwm_set_gpio_level(BUZZER_PIN, 0);       // Desliga o PWM
}