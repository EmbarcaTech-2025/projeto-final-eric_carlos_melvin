// ============================================================================
// INCLUDES - Bibliotecas necessárias para o funcionamento do sistema
// ============================================================================

#include <stdio.h>          // Funções de entrada/saída (printf, scanf, etc.)
#include <string.h>         // Funções para manipulação de strings (strlen, strcpy, etc.)
#include <time.h>           // Funções relacionadas a tempo e data
#include "pico/stdlib.h"    // Biblioteca padrão do Raspberry Pi Pico
#include "hardware/spi.h"   // Biblioteca para comunicação SPI
#include "hardware/rtc.h"   // Biblioteca para o relógio em tempo real RTC

// Bibliotecas para o sistema de arquivos FAT no cartão SD
#include "ff.h"             // FatFS - sistema de arquivos FAT
#include "diskio.h"         // Interface de disco para FatFS
#include "f_util.h"         // Utilitários para FatFS
#include "hw_config.h"      // Configuração do hardware
#include "sd_card.h"        // Driver do cartão SD

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

// Estrutura para armazenar um registro completo de sensor
typedef struct {
    int id;                 // Identificador único do registro (1, 2, 3...)
    char datetime[20];      // Data e hora no formato "5/23/25 10:00"
    char permanencia[10];   // Duração no formato "0:00:35"
    char alerta[20];        // Local/tipo do alerta ("Perna Dir", "Perna Esq")
    int valor;              // Valor numérico do sensor (0-100, por exemplo)
    char tipo[20];          // Tipo de movimento ("Abdução", "Rotação")
} sensor_data_t;

// ============================================================================
// VARIÁVEIS GLOBAIS - Dados compartilhados por todo o programa
// ============================================================================

static int next_id = 1;        // Próximo ID a ser usado (auto-incremento)
static FATFS fs;               // Sistema de arquivos do cartão SD
static bool sd_mounted = false; // Flag indicando se o SD está montado e pronto

// ============================================================================
// PROTÓTIPOS DAS FUNÇÕES - Declaração das funções antes de serem implementadas
// ============================================================================

bool init_sd_card(void);    // Inicializa o cartão SD e prepara o sistema de arquivos
bool add_csv_record(const char* permanencia, const char* alerta, int valor, const char* tipo);  // Adiciona registro no CSV
void view_csv_data(void);   // Lê e exibe todos os dados do arquivo CSV
void get_current_datetime(char* buffer, size_t buffer_size);  // Obtém data/hora atual formatada
void init_rtc_demo(void);   // Inicializa o RTC com data/hora de demonstração

// ============================================================================
// FUNÇÃO: init_rtc_demo()
// PROPÓSITO: Configura o relógio interno com uma data/hora fictícia para demo
// ============================================================================

void init_rtc_demo(void) 
{
    // Cria uma estrutura com data/hora específica para demonstração
    datetime_t t = {
        .year  = 2025,      // Ano: 2025
        .month = 5,         // Mês: Maio (5)
        .day   = 23,        // Dia: 23
        .dotw  = 5,         // Dia da semana: Friday (0=Domingo, 1=Segunda...)
        .hour  = 10,        // Hora: 10h
        .min   = 0,         // Minutos: 00min
        .sec   = 0          // Segundos: 00seg
    };
    
    rtc_init();                 // Inicializa o hardware do RTC
    rtc_set_datetime(&t);       // Define a data/hora configurada acima
    sleep_us(64);               // Aguarda 64 microssegundos para estabilizar
}

// ============================================================================
// FUNÇÃO: get_current_datetime()
// PROPÓSITO: Obtém a data/hora atual do RTC e formata como string
// PARÂMETROS: buffer - onde armazenar o resultado, buffer_size - tamanho máximo
// ============================================================================

void get_current_datetime(char* buffer, size_t buffer_size) 
{
    datetime_t t;                       // Estrutura para receber data/hora atual
    rtc_get_datetime(&t);               // Lê data/hora atual do RTC
    
    // Formata a data/hora como "5/23/25 10:00" e armazena no buffer
    snprintf(buffer, buffer_size, "%d/%d/%02d %02d:%02d", 
             t.month,           // Mês (1-12)
             t.day,             // Dia (1-31)
             t.year % 100,      // Ano com 2 dígitos (25 para 2025)
             t.hour,            // Hora (0-23)
             t.min);            // Minutos (0-59)
}

// ============================================================================
// FUNÇÃO: init_sd_card()
// PROPÓSITO: Inicializa o cartão SD e prepara o sistema de arquivos
// RETORNO: true se sucesso, false se erro
// ============================================================================

bool init_sd_card(void) 
{
    printf("Inicializando SD card...\n");
    
    // PASSO 1: Inicializar o driver do cartão SD
    if (!sd_init_driver()) 
    {
        printf("Erro: Falha ao inicializar driver SD\n");
        return false;               // Retorna erro se driver falhou
    }
    
    // PASSO 2: Obter referência do cartão SD configurado (primeiro cartão = índice 0)
    sd_card_t* sd_card = sd_get_by_num(0);
    if (!sd_card) 
    {
        printf("Erro: Não foi possível obter referência do cartão SD\n");
        return false;               // Retorna erro se não encontrou o cartão
    }
    
    // PASSO 3: Montar o sistema de arquivos FAT do cartão SD
    // "0:" = nome do drive, 1 = montar imediatamente
    FRESULT fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) 
    {
        printf("Erro ao montar SD card: %s (%d)\n", FRESULT_str(fr), fr);
        return false;               // Retorna erro se montagem falhou
    }
    
    sd_mounted = true;              // Marca que SD está pronto para uso
    printf("SD card montado com sucesso!\n");
    
    // PASSO 4: Verificar se arquivo CSV já existe, se não, criar com cabeçalho
    FIL file;                       // Estrutura para manipular arquivo
    fr = f_open(&file, "dados.csv", FA_READ);  // Tenta abrir arquivo para leitura
    
    if (fr != FR_OK) 
    {
        // Arquivo não existe, vamos criar um novo com cabeçalho
        fr = f_open(&file, "dados.csv", FA_CREATE_NEW | FA_WRITE);
        if (fr == FR_OK) 
        {
            // Escreve o cabeçalho do CSV (nomes das colunas)
            f_printf(&file, "ID,DateTime,Permanencia,Alerta,Valor,Tipo\n");
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
// PARÂMETROS: permanencia - duração ("0:00:35")
//            alerta - local/tipo ("Perna Dir")  
//            valor - valor numérico do sensor
//            tipo - tipo de movimento ("Abdução")
// RETORNO: true se sucesso, false se erro
// ============================================================================

bool add_csv_record(const char* permanencia, const char* alerta, int valor, const char* tipo) 
{
    // VERIFICAÇÃO: SD card deve estar montado e pronto
    if (!sd_mounted) 
    {
        printf("Erro: SD card não está montado\n");
        return false;
    }
    
    FIL file;                       // Estrutura para manipular arquivo
    FRESULT fr;                     // Resultado das operações de arquivo
    char datetime_str[20];          // Buffer para armazenar data/hora formatada
    
    // PASSO 1: Obter data/hora atual formatada
    get_current_datetime(datetime_str, sizeof(datetime_str));
    
    // PASSO 2: Abrir arquivo CSV para adicionar dados no final (append)
    // FA_OPEN_APPEND = abrir para adicionar no final
    // FA_WRITE = permissão de escrita
    fr = f_open(&file, "dados.csv", FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) 
    {
        printf("Erro ao abrir arquivo CSV: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    }
    
    // PASSO 3: Escrever nova linha no formato CSV
    // Formato: ID,DateTime,Permanencia,Alerta,Valor,Tipo
    // Exemplo: 1,5/23/25 10:00,0:00:35,Perna Dir,95,Abdução
    f_printf(&file, "%d,%s,%s,%s,%d,%s\n", 
             next_id,               // ID auto-incrementado
             datetime_str,          // Data/hora atual
             permanencia,           // Duração passada como parâmetro
             alerta,                // Local/tipo passado como parâmetro
             valor,                 // Valor numérico passado como parâmetro
             tipo);                 // Tipo de movimento passado como parâmetro
    
    // PASSO 4: Fechar arquivo (importante para salvar os dados!)
    f_close(&file);
    
    // PASSO 5: Mostrar confirmação no console e incrementar ID para próximo registro
    printf("Registro %d adicionado: %s, %s, %s, %d, %s\n", 
           next_id, datetime_str, permanencia, alerta, valor, tipo);
    
    next_id++;                      // Incrementa ID para próximo registro
    return true;                    // Sucesso!
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
