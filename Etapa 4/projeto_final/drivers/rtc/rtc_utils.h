// ======================================================================
//  Arquivo: rtc_utils.h
//  Descrição: Interface utilitária para manipulação do RTC DS3231
// ======================================================================

#ifndef RTC_UTILS_H
#define RTC_UTILS_H

#include <stdbool.h>    // Tipos booleanos padrão
#include <stddef.h>     // Definição de size_t
#include "ds3231.h"    // Tipos e funções do driver DS3231

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file rtc_utils.h
 * @brief Biblioteca utilitária para manipulação do RTC DS3231 em projetos embarcados.
 *
 * Fornece funções para inicialização, leitura e formatação de data/hora do RTC de forma simples e reutilizável.
 */

// ----------------------------------------------------------------------
// Inicialização do RTC
// ----------------------------------------------------------------------
/**
 * @brief Inicializa o RTC DS3231 e ajusta automaticamente para a data/hora do sistema de compilação.
 *
 * Esta função configura o barramento I2C, inicializa o driver do DS3231 e ajusta o relógio
 * com a data e hora do momento da compilação do firmware (macros __DATE__ e __TIME__).
 * Deve ser chamada uma única vez no início do programa.
 */
void rtc_ds3231_init(void);

// ----------------------------------------------------------------------
// Leitura da data/hora atual do RTC
// ----------------------------------------------------------------------
/**
 * @brief Atualiza a estrutura ds3231_data_t com a data/hora atual do RTC.
 *
 * @param[out] dt Ponteiro para a estrutura que receberá os dados lidos do RTC.
 * @return true se a leitura foi bem-sucedida, false em caso de erro de comunicação.
 */
bool rtc_update_datetime(ds3231_data_t *dt);

// ----------------------------------------------------------------------
// Formatação de data/hora em string padrão
// ----------------------------------------------------------------------
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
bool rtc_get_datetime_string(const ds3231_data_t *dt, char *out, size_t out_size);

#ifdef __cplusplus
}
#endif

#endif // RTC_UTILS_H
