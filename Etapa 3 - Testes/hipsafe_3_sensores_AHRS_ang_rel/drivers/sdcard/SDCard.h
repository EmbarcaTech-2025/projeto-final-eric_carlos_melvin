#ifndef SDCARD_H
#define SDCARD_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

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
 * Configura o SPI, monta o sistema de arquivos FAT e cria o arquivo CSV com cabeçalho se necessário.
 * 
 * @return true se a inicialização foi bem-sucedida, false caso contrário
 */
bool init_sd_card(void);

/**
 * Adiciona uma nova linha de dados no arquivo CSV.
 * Os dados são gravados no formato: ID,DateTime,Permanencia,Alerta,Valor,Tipo
 * 
 * @param permanencia Duração no formato "H:MM:SS" (ex: "0:01:35")
 * @param alerta      Local/tipo do alerta (ex: "Perna Dir", "Perna Esq")
 * @param valor       Valor numérico do sensor (0-100, por exemplo)
 * @param tipo        Tipo de movimento (ex: "Abdução", "Rotação", "Flexão")
 * @return true se o registro foi adicionado com sucesso, false caso contrário
 */
bool add_csv_record(const char* permanencia, const char* alerta, int valor, const char* tipo);

/**
 * Lê todo o arquivo CSV e exibe o conteúdo no console.
 * Útil para visualizar todos os dados armazenados no cartão SD.
 */
void view_csv_data(void);

/**
 * Obtém a data/hora atual do RTC e formata como string.
 * O formato de saída é "M/D/YY HH:MM" (ex: "5/23/25 10:00")
 * 
 * @param buffer      Buffer onde armazenar o resultado formatado
 * @param buffer_size Tamanho máximo do buffer
 */
void get_current_datetime(char* buffer, size_t buffer_size);

/**
 * Inicializa o RTC com uma data/hora de demonstração.
 * Define uma data/hora específica para testes (23/05/2025 10:00:00).
 */
void init_rtc_demo(void);

/**
 * Função de conveniência que combina init_rtc_demo() e init_sd_card().
 * Inicializa o RTC e o sistema de SD Card em uma única chamada.
 * 
 * @return true se ambas as inicializações foram bem-sucedidas, false caso contrário
 */
static inline bool sd_card_init(void) {
    init_rtc_demo();
    return init_sd_card();
}

#endif
