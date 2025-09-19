
/************************************************************
 * Driver de Controle do Buzzer via PWM                     *
 *                                                          *
 * Este arquivo implementa funções para inicialização,       *
 * controle de alarme e emissão de beep em um buzzer        *
 * utilizando o periférico PWM do Raspberry Pi Pico.         *
 ************************************************************/

#include <stdio.h>
#include "buzzer.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

// Variável estática para indicar se o alarme está ativado
static bool alarm_is_on = false;

/**
 * @brief Inicializa o buzzer configurando o pino e o PWM.
 *
 * Esta função deve ser chamada uma vez durante a inicialização do sistema.
 * Ela configura o pino do buzzer para funcionar como saída PWM, define a frequência
 * desejada e garante que o buzzer inicie desligado.
 */
void buzzer_init(void)
{
    // Configura o pino do buzzer para função PWM
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);

    // Obtém o número do slice PWM associado ao pino do buzzer
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Obtém a frequência do clock do sistema (clk_sys)
    uint32_t clock_freq = clock_get_hz(clk_sys);

    // Calcula o valor de "top" (wrap) para obter a frequência desejada no PWM
    uint32_t top = clock_freq / BUZZER_FREQ - 1;

    // Obtém a configuração padrão do PWM
    pwm_config config = pwm_get_default_config();

    // Ajusta o divisor de clock para controlar o volume (quanto menor, maior o volume)
    pwm_config_set_clkdiv(&config, 4.0f);

    // Define o valor de "top" (resolução do PWM)
    pwm_config_set_wrap(&config, top);

    // Inicializa o PWM com a configuração definida e já habilita o PWM
    pwm_init(slice_num, &config, true);

    // Garante que o buzzer inicie desligado (nível PWM = 0)
    pwm_set_gpio_level(BUZZER_PIN, 0);
}

/**
 * @brief Ativa o alarme sonoro no buzzer.
 *
 * Esta função liga o buzzer em um duty cycle de 50%, produzindo um som contínuo.
 * O valor de "top" é recalculado para garantir a frequência correta.
 * A variável alarm_is_on é atualizada para refletir o estado do alarme.
 */
void buzzer_alarm_on(void)
{
    // Obtém o número do slice PWM do pino do buzzer
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Obtém a frequência do clock do sistema
    uint32_t clock_freq = clock_get_hz(clk_sys);

    // Calcula o valor de "top" para a frequência desejada
    uint32_t top = clock_freq / BUZZER_FREQ - 1;

    // Atualiza o valor de "top" do PWM
    pwm_set_wrap(slice_num, top);

    // Define o duty cycle em 50% para volume máximo
    pwm_set_gpio_level(BUZZER_PIN, top / 2);

    // Atualiza o estado do alarme
    alarm_is_on = true;
}

/**
 * @brief Desativa o alarme sonoro do buzzer.
 *
 * Esta função silencia o buzzer, desligando o PWM e atualizando o estado do alarme.
 */
void buzzer_alarm_off(void)
{
    // Desliga o PWM (nível 0)
    pwm_set_gpio_level(BUZZER_PIN, 0);

    // Atualiza o estado do alarme
    alarm_is_on = false;
}

/**
 * @brief Emite um beep curto no buzzer.
 *
 * Esta função é útil para indicar eventos como início ou fim de calibração.
 * O beep tem duração de 200 ms e duty cycle de 50%.
 */
void buzzer_beep(void)
{
    // Obtém o número do slice PWM do pino do buzzer
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Obtém a frequência do clock do sistema
    uint32_t clock_freq = clock_get_hz(clk_sys);

    // Calcula o valor de "top" para a frequência desejada
    uint32_t top = clock_freq / BUZZER_FREQ - 1;

    // Atualiza o valor de "top" do PWM
    pwm_set_wrap(slice_num, top);

    // Liga o buzzer em 50% de duty cycle
    pwm_set_gpio_level(BUZZER_PIN, top / 2);

    // Mantém o beep por 200 ms
    sleep_ms(200);

    // Desliga o buzzer
    pwm_set_gpio_level(BUZZER_PIN, 0);
}