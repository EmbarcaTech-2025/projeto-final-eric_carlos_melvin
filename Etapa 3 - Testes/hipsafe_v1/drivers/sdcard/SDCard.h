#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "../rtc/rtc_utils.h"  // Para usar as funções do RTC DS3231

#ifdef __cplusplus
extern "C" {
#endif

// Estrutura para armazenar um registro completo de movimento
typedef struct {
    char inicio[25];        // Início no formato ISO 8601 (ex: "2025-09-07T13:45:30Z")
    char fim[25];           // Fim no formato ISO 8601 (ex: "2025-09-07T13:45:33Z")
    char perna[16];         // "direita" ou "esquerda"
    char movimento[20];     // Tipo de movimento (ex: "Flexão")
    float angulo_maximo;    // Ângulo máximo (ex: 92.5)
} movimento_data_t;

// ============================================================================
// DEFINIÇÕES DE HARDWARE - Configuração dos pinos SPI para o cartão SD
// ============================================================================

#define SPI_PORT spi0       // Usa o SPI0 do Pico (existem spi0 e spi1)
#define PIN_MISO 16         // Pino GPIO16 - Master In Slave Out (dados do SD para Pico)
#define PIN_CS   17         // Pino GPIO17 - Chip Select (seleciona o SD para comunicação)
#define PIN_SCK  18         // Pino GPIO18 - Serial Clock (clock da comunicação SPI)
#define PIN_MOSI 19         // Pino GPIO19 - Master Out Slave In (dados do Pico para SD)

// ============================================================================
// PROTÓTIPOS DAS FUNÇÕES PÚBLICAS
// ============================================================================

/**
 * Inicializa o cartão SD e prepara o sistema de arquivos.
 * Inicializa primeiro o RTC DS3231, depois configura o SPI, monta o sistema de arquivos FAT 
 * e cria o arquivo CSV com cabeçalho se necessário.
 * 
 * @return true se a inicialização foi bem-sucedida, false caso contrário
 */
bool sd_card_init(void);

/**
 * Adiciona uma nova linha de dados no arquivo CSV.
 * Os dados são gravados no formato: Inicio,Fim,Perna,Movimento,AnguloMaximo
 *
 * @param inicio        Início no formato ISO 8601 (ex: "2025-09-07T13:45:30Z")
 * @param fim           Fim no formato ISO 8601 (ex: "2025-09-07T13:45:33Z")
 * @param perna         "direita" ou "esquerda"
 * @param movimento     Tipo de movimento (ex: "Flexão")
 * @param angulo_maximo Ângulo máximo (ex: 92.5)
 * @return true se o registro foi adicionado com sucesso, false caso contrário
 */
bool add_csv_record(const char* inicio, const char* fim, const char* perna, const char* movimento, float angulo_maximo);

/**
 * Lê todo o arquivo CSV e exibe o conteúdo no console.
 * Útil para visualizar todos os dados armazenados no cartão SD.
 */
void view_csv_data(void);

/**
 * Obtém a data/hora atual do RTC DS3231 e formata como string ISO 8601.
 * O formato de saída é "YYYY-MM-DDTHH:MM:SSZ" (ex: "2025-09-07T13:45:30Z")
 * Se não conseguir ler do RTC, usa uma data padrão como fallback.
 *
 * @param buffer      Buffer onde armazenar o resultado formatado
 * @param buffer_size Tamanho máximo do buffer
 */
void get_current_datetime_iso(char* buffer, size_t buffer_size);

/**
 * Função utilitária para registrar um movimento completo automaticamente.
 * Captura o timestamp atual como início, executa a função de movimento fornecida,
 * captura o timestamp final e registra tudo no CSV.
 *
 * @param perna         "direita" ou "esquerda"
 * @param movimento     Tipo de movimento (ex: "Flexão")
 * @param angulo_maximo Ângulo máximo atingido
 * @return true se o registro foi adicionado com sucesso, false caso contrário
 */
bool register_movement_with_timestamps(const char* perna, const char* movimento, float angulo_maximo);
#ifdef __cplusplus
}
#endif

#endif
