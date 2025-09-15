// ============================================================================
// INCLUDES - Bibliotecas necessárias para o funcionamento do sistema
// ============================================================================

#include <stdio.h>          // Funções de entrada/saída (printf, scanf, etc.)
#include <string.h>         // Funções para manipulação de strings (strlen, strcpy, etc.)
#include <time.h>           // Funções relacionadas a tempo e data
#include "pico/stdlib.h"    // Biblioteca padrão do Raspberry Pi Pico
#include "hardware/spi.h"   // Biblioteca para comunicação SPI

// Bibliotecas para o sistema de arquivos FAT no cartão SD
#include "ff.h"             // FatFS - sistema de arquivos FAT
#include "diskio.h"         // Interface de disco para FatFS
#include "f_util.h"         // Utilitários para FatFS
#include "hw_config.h"      // Configuração do hardware
#include "sd_card.h"        // Driver do cartão SD

// Biblioteca para RTC DS3231 
#include "rtc_utils.h"      // Utilitários para RTC DS3231 (ajustado para drivers/rtc)

// ============================================================================
// DEFINIÇÕES DE HARDWARE - Configuração dos pinos SPI para o cartão SD
// ============================================================================

#define SPI_PORT spi0       // Usa o SPI0 do Pico (existem spi0 e spi1)
#define PIN_MISO 16         // Pino GPIO16 - Master In Slave Out (dados do SD para Pico)
#define PIN_CS   17         // Pino GPIO17 - Chip Select (seleciona o SD para comunicação)
#define PIN_SCK  18         // Pino GPIO18 - Serial Clock (clock da comunicação SPI)
#define PIN_MOSI 19         // Pino GPIO19 - Master Out Slave In (dados do Pico para SD)

// ============================================================================
// ESTRUTURAS DE DADOS - Define como os dados serão organizados
// ============================================================================

// Estrutura para armazenar um registro completo de movimento
typedef struct {
    char inicio[25];        // Início no formato ISO 8601 (ex: "2025-09-07T13:45:30Z")
    char fim[25];           // Fim no formato ISO 8601 (ex: "2025-09-07T13:45:33Z")
    char perna[16];         // "direita" ou "esquerda"
    char movimento[20];     // Tipo de movimento (ex: "Flexão")
    float angulo_maximo;    // Ângulo máximo (ex: 92.5)
} movimento_data_t;

// ============================================================================
// VARIÁVEIS GLOBAIS - Dados compartilhados por todo o programa
// ============================================================================

static FATFS fs;               // Sistema de arquivos do cartão SD
static bool sd_mounted = false; // Flag indicando se o SD está montado e pronto

// ============================================================================
// PROTÓTIPOS DAS FUNÇÕES - Declaração das funções antes de serem implementadas
// ============================================================================

bool init_sd_card(void);    // Inicializa o cartão SD e prepara o sistema de arquivos
bool add_csv_record(const char* inicio, const char* fim, const char* perna, const char* movimento, float angulo_maximo);  // Adiciona registro no CSV
void view_csv_data(void);   // Lê e exibe todos os dados do arquivo CSV
void get_current_datetime_iso(char* buffer, size_t buffer_size);  // Obtém data/hora atual formatada ISO 8601

// ============================================================================
// FUNÇÃO: get_current_datetime_iso()
// PROPÓSITO: Obtém a data/hora atual do RTC DS3231 e formata como string ISO 8601
// PARÂMETROS: buffer - onde armazenar o resultado, buffer_size - tamanho máximo
// ============================================================================
void get_current_datetime_iso(char* buffer, size_t buffer_size)
{
    ds3231_data_t dt;

    // Lê a data/hora atual do RTC DS3231
    if (rtc_update_datetime(&dt))
    {
        // Calcula o ano completo (considerando century bit)
        int year_full = (dt.century ? 2000 : 1900) + dt.year;

        // Formato: "YYYY-MM-DDTHH:MM:SSZ"
        snprintf(buffer, buffer_size, "%04d-%02d-%02dT%02d:%02d:%02dZ",
            year_full, dt.month, dt.date, dt.hours, dt.minutes, dt.seconds);
    }
    else
    {
        // Fallback caso não consiga ler do RTC
        snprintf(buffer, buffer_size, "2025-01-01T00:00:00Z");
        printf("Aviso: Não foi possível ler do RTC, usando data padrão\n");
    }
}

// ============================================================================
// FUNÇÃO: init_sd_card()
// PROPÓSITO: Inicializa o cartão SD e prepara o sistema de arquivos
// RETORNO: true se sucesso, false se erro
// ============================================================================

bool init_sd_card(void)
{
    printf("Inicializando SD card...\n");

    // PASSO 1: Inicializar o RTC DS3231
    printf("Inicializando RTC DS3231...\n");
    rtc_ds3231_init();  // Usa a função de rtc_utils.c que configura automaticamente
    printf("RTC DS3231 inicializado\n");

    // PASSO 2: Inicializar o driver do cartão SD
    if (!sd_init_driver())
    {
        printf("Erro: Falha ao inicializar driver SD\n");
        return false;               // Retorna erro se driver falhou
    }
    
    // PASSO 3: Obter referência do cartão SD configurado (primeiro cartão = índice 0)
    sd_card_t* sd_card = sd_get_by_num(0);
    if (!sd_card)
    {
        printf("Erro: Não foi possível obter referência do cartão SD\n");
        return false;               // Retorna erro se não encontrou o cartão
    }

    // PASSO 4: Montar o sistema de arquivos FAT do cartão SD
    // "0:" = nome do drive, 1 = montar imediatamente
    FRESULT fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK)
    {
        printf("Erro ao montar SD card: %s (%d)\n", FRESULT_str(fr), fr);
        return false;               // Retorna erro se montagem falhou
    }

    sd_mounted = true;              // Marca que SD está pronto para uso
    printf("SD card montado com sucesso!\n");

    // PASSO 5: Verificar se arquivo CSV já existe, se não, criar com cabeçalho
    FIL file;                       // Estrutura para manipular arquivo
    fr = f_open(&file, "dados.csv", FA_READ);  // Tenta abrir arquivo para leitura

    if (fr != FR_OK) 
    {
        // Arquivo não existe, vamos criar um novo com cabeçalho
        fr = f_open(&file, "dados.csv", FA_CREATE_NEW | FA_WRITE);
        if (fr == FR_OK)
        {
            // Escreve o cabeçalho do CSV (nomes das colunas)
            f_printf(&file, "Inicio,Fim,Perna,Movimento,AnguloMaximo\n");
            f_close(&file);         // Fecha o arquivo
            printf("Arquivo CSV criado com cabeçalho\n");
        }
        else
        {
            printf("Erro ao criar arquivo CSV: %s (%d)\n", FRESULT_str(fr), fr);
            return false;           // Retorna erro se não conseguiu criar arquivo
        }
    }
    else
    {
        // Arquivo já existe, apenas fecha
        f_close(&file);
        printf("Arquivo CSV já existe\n");
    }

    return true;                    // Sucesso! SD card está pronto
}

// ============================================================================
// FUNÇÃO: add_csv_record()
// PROPÓSITO: Adiciona uma nova linha de dados no arquivo CSV
// PARÂMETROS: inicio, fim, perna, movimento, angulo_maximo
// RETORNO: true se sucesso, false se erro
// ============================================================================

bool add_csv_record(const char* inicio, const char* fim, const char* perna, const char* movimento, float angulo_maximo)
{
    // VERIFICAÇÃO: SD card deve estar montado e pronto
    if (!sd_mounted)
    {
        printf("Erro: SD card não está montado\n");
        return false;
    }

    FIL file;
    FRESULT fr;

    // PASSO 1: Abrir arquivo CSV para adicionar dados no final (append)
    fr = f_open(&file, "dados.csv", FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        printf("Erro ao abrir arquivo CSV: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }

    // PASSO 2: Escrever nova linha no formato CSV
    // Formato: Inicio,Fim,Perna,Movimento,AnguloMaximo
    // Exemplo: 2025-09-07T13:45:30Z,2025-09-07T13:45:33Z,direita,Flexão,92.5
    f_printf(&file, "%s,%s,%s,%s,%.2f\n",
        inicio,
        fim,
        perna,
        movimento,
        angulo_maximo);

    // PASSO 3: Fechar arquivo
    f_close(&file);

    // PASSO 4: Mostrar confirmação no console
    printf("Registro adicionado: %s, %s, %s, %s, %.2f\n",
        inicio, fim, perna, movimento, angulo_maximo);

    return true;
}

// ============================================================================
// FUNÇÃO: register_movement_with_timestamps()
// PROPÓSITO: Registra um movimento completo com timestamps automáticos
// PARÂMETROS: perna - "direita" ou "esquerda"
//            movimento - tipo de movimento
//            angulo_maximo - ângulo máximo atingido
// RETORNO: true se sucesso, false se erro
// ============================================================================

bool register_movement_with_timestamps(const char* perna, const char* movimento, float angulo_maximo)
{
    char inicio[25], fim[25];

    // Captura timestamp de início
    get_current_datetime_iso(inicio, sizeof(inicio));

    // Simula um pequeno delay para o movimento (em uma aplicação real,
    // este seria o tempo real de execução do movimento)
    sleep_ms(100);  // 100ms de delay como exemplo

    // Captura timestamp de fim
    get_current_datetime_iso(fim, sizeof(fim));

    // Registra no CSV
    return add_csv_record(inicio, fim, perna, movimento, angulo_maximo);
}

// ============================================================================
// FUNÇÃO: view_csv_data()
// PROPÓSITO: Lê todo o arquivo CSV e exibe no console
// ============================================================================

void view_csv_data(void)
{
    // VERIFICAÇÃO: SD card deve estar montado
    if (!sd_mounted)
    {
        printf("Erro: SD card não está montado\n");
        return;
    }

    FIL file;                       // Estrutura para manipular arquivo
    FRESULT fr;                     // Resultado das operações
    char line[256];                 // Buffer para armazenar cada linha lida (máx 256 chars)

    // PASSO 1: Abrir arquivo CSV para leitura
    fr = f_open(&file, "dados.csv", FA_READ);
    if (fr != FR_OK) 
    {
        printf("Erro ao abrir arquivo CSV: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }

    printf("\n=== DADOS ARMAZENADOS NO CSV ===\n");

    // PASSO 2: Ler arquivo linha por linha e exibir
    while (f_gets(line, sizeof(line), &file)) // Lê uma linha por vez
    {
        // Remover caractere de quebra de linha (\n) do final da string
        size_t len = strlen(line);                  // Calcula tamanho da string
        if (len > 0 && line[len-1] == '\n')         // Se último char é \n
        {
            line[len-1] = '\0';                     // Substitui por terminador \0
        }
        printf("%s\n", line);                       // Exibe a linha no console
    }

    printf("=== FIM DOS DADOS ===\n\n");

    // PASSO 3: Fechar arquivo
    f_close(&file);
}