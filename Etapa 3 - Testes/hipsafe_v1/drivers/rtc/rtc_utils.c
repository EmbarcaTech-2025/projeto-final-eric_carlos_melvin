
/**
 * @file rtc_utils.c
 * @brief Implementação das funções utilitárias para o RTC DS3231.
 */

#include "rtc_utils.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT    i2c0
#define I2C_SDA     0
#define I2C_SCL     1
#define I2C_BAUDRATE 100000

static ds3231_t rtc;


/**
 * @brief Verifica se o RTC DS3231 está conectado ao barramento I2C.
 * @return true se o dispositivo respondeu, false caso contrário.
 */
static bool rtc_is_connected(void) 
{
    uint8_t dummy = 0;
    int ret = i2c_read_blocking(I2C_PORT, DS3231_DEVICE_ADRESS, &dummy, 1, false);
    return (ret >= 0);
}


/**
 * @brief Inicializa o RTC DS3231 e ajusta automaticamente para a data/hora do sistema de compilação.
 *
 * Configura o barramento I2C, inicializa o driver do DS3231 e ajusta o relógio
 * com a data e hora do momento da compilação do firmware (macros __DATE__ e __TIME__).
 * Deve ser chamada uma única vez no início do programa.
 */
void rtc_ds3231_init(void) 
{
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ds3231_init(&rtc, I2C_PORT, DS3231_DEVICE_ADRESS, AT24C32_EEPROM_ADRESS_0);

    // Só ajusta o RTC se ele estiver "zerado" (ano < 2020)
    ds3231_data_t dt_now;
    if (ds3231_read_current_time(&rtc, &dt_now) == 0) {
        int year_full = (dt_now.century ? 2000 : 1900) + dt_now.year;
        if (year_full < 2020) {
            int ano, mes, dia, hora, min, seg;
            char mes_str[4];
            sscanf(__DATE__, "%3s %d %d", mes_str, &dia, &ano);
            sscanf(__TIME__, "%d:%d:%d", &hora, &min, &seg);
            if      (!strcmp(mes_str, "Jan")) mes = 1;
            else if (!strcmp(mes_str, "Feb")) mes = 2;
            else if (!strcmp(mes_str, "Mar")) mes = 3;
            else if (!strcmp(mes_str, "Apr")) mes = 4;
            else if (!strcmp(mes_str, "May")) mes = 5;
            else if (!strcmp(mes_str, "Jun")) mes = 6;
            else if (!strcmp(mes_str, "Jul")) mes = 7;
            else if (!strcmp(mes_str, "Aug")) mes = 8;
            else if (!strcmp(mes_str, "Sep")) mes = 9;
            else if (!strcmp(mes_str, "Oct")) mes = 10;
            else if (!strcmp(mes_str, "Nov")) mes = 11;
            else if (!strcmp(mes_str, "Dec")) mes = 12;
            else mes = 1;
            ds3231_data_t dt_set = {
                .seconds = seg,
                .minutes = min,
                .hours   = hora,
                .day     = 1,
                .date    = dia,
                .month   = mes,
                .year    = (ano >= 2000) ? (ano - 2000) : (ano - 1900),
                .century = (ano >= 2000) ? 1 : 0
            };
            ds3231_configure_time(&rtc, &dt_set);
        }
    }
}


/**
 * @brief Atualiza a estrutura ds3231_data_t com a data/hora atual do RTC.
 *
 * @param[out] dt Ponteiro para a estrutura que receberá os dados lidos do RTC.
 * @return true se a leitura foi bem-sucedida, false em caso de erro de comunicação.
 */
bool rtc_update_datetime(ds3231_data_t *dt) 
{
    return ds3231_read_current_time(&rtc, dt) == 0;
}


/**
 * @brief Formata a data/hora em string reutilizável no padrão "YYYY-MM-DD HH:MM:SS".
 *
 * @param[in] dt Ponteiro para a estrutura ds3231_data_t preenchida.
 * @param[out] out Buffer de destino para a string formatada.
 * @param[in] out_size Tamanho do buffer de destino.
 * @return true se a string foi gerada com sucesso, false se ponteiro inválido.
 *
 * @note Útil para montagem de JSONs ou logs de eventos.
 */
bool rtc_get_datetime_string(const ds3231_data_t *dt, char *out, size_t out_size) 
{
    if (!dt) return false;
    int year = (dt->century ? 2000 : 1900) + dt->year;
    unsigned month = (dt->month > 0 && dt->month <= 12) ? dt->month : 1;
    snprintf(out, out_size, "%04d-%02u-%02u %02u:%02u:%02u", year, month, dt->date, dt->hours, dt->minutes, dt->seconds);
    return true;
}
