#include <iostream>
#include <vector>
#include "pico/stdlib.h"

// Headers C
extern "C" {
    #include "drivers/button/button.h"
    #include "drivers/buzzer/buzzer.h"
    #include "drivers/sdcard/SDCard.h"
    #include "drivers/mpu6050/mpu6050_i2c.h"
    #include "analise_postural_sensor.h"
}

// Headers C++
#include "evento/evento.h"

// Configurações da aplicação
#define SAMPLE_RATE 100.0f
#define CALIBRATION_SAMPLES 1000

// Declarações de funções
bool eventoAberto();
void ligaAlarme();
void desligaAlarme();
void silenciarAlarme();
void armazenarNoSDCard(const Evento& evento);
void calibrate_sensors();
void initialize_fusion_system();
void buzzer_beep(uint32_t duration_ms);

// Variáveis globais
bool alarme_silenciado = false;
std::vector<Evento*> eventosAbertos;
static int contador_prints = 0; // Contador para reduzir frequência dos prints (100Hz seria muito output)

// Função auxiliar para criar beeps temporários
void buzzer_beep(uint32_t duration_ms) 
{
    buzzer_on(BUZZER_A, BUZZER_LEVEL_MED);
    sleep_ms(duration_ms);
    buzzer_off(BUZZER_A);
}

// Função para calibração de offset (bias) dos sensores
void calibrate_sensors() 
{
    printf("Iniciando calibração dos sensores...\n");
    printf("Mantenha os sensores IMÓVEIS durante a calibração!\n");
    
    // Som de início da calibração (3 beeps curtos e rápidos)
    printf("Emitindo sinal sonoro de início...\n");
    for (int i = 0; i < 3; i++) {
        buzzer_beep(100); // Beep curto de 100ms
        sleep_ms(150);    // Pausa de 150ms entre beeps
    }
    sleep_ms(500); // Pausa antes de iniciar calibração
    
    // Usar a nova função para calibrar todos os sensores
    calibrate_all_sensors();
    
    // Som de fim da calibração (1 beep longo e grave)
    printf("Calibração concluída!\n");
    printf("Emitindo sinal sonoro de conclusão...\n");
    buzzer_beep(800); // Beep longo de 800ms para indicar sucesso
}

// Função para configurar o algoritmo AHRS
void initialize_fusion_system() 
{
    printf("Configurando sistema Fusion AHRS...\n");
    
    // Usar a nova função para inicializar todos os sistemas Fusion
    initialize_all_fusion_systems();
    
    printf("AHRS configurado para todos os sensores!\n");
}

int main() 
{
    // Inicialização do sistema
    stdio_init_all(); // Inicializa UART/USB para debug
    
    printf("=== HIPSAFE v1 - Sistema de Monitoramento Postural ===\n");
    printf("Iniciando sistema...\n");
    
    // Inicializar buzzer PRIMEIRO para usar nos sinais de calibração
    printf("Inicializando buzzer...\n");
    buzzer_init();
    
    // Inicializar I2C para os MPU6050
    printf("Configurando sensores MPU6050...\n");
    
    // Inicializar estruturas dos sensores
    mpu6050_init_all_sensors();
    
    // Configurar I2C para todos os sensores
    mpu6050_setup_all_sensors();
    
    // Reiniciar todos os sensores
    mpu6050_reset_all_sensors();
    
    // Configurar todos os sensores para ±4g e ±500°/s (otimizado para aplicação na perna)
    mpu6050_configure_all_sensors(1, 1); // 1 = ±4g, 1 = ±500°/s
    printf("Todos os MPU6050 configurados: ±4g, ±500°/s\n");
    
    // Aguardar estabilização do sensor
    printf("Aguardando estabilização do sensor...\n");
    sleep_ms(100);
    
    // Calibrar sensores
    calibrate_sensors();
    
    // Configurar sistema Fusion AHRS
    initialize_fusion_system();
    
    // Inicializar botões
    printf("Inicializando botões...\n");
    setup_buttons();
    
    // Inicializar SD Card
    printf("Inicializando SD Card...\n");
    sd_card_init();
    
    printf("Sistema inicializado com sucesso!\n");
    printf("Configuração: Taxa de amostragem 100Hz (período = 10ms)\n");
    printf("Iniciando monitoramento postural...\n\n");

    while (true) 
    {
        // Interrupção externa: Botão A pressionado
        if (button_a_pressed) 
        {
            printf("Botão A pressionado!\n");
            if (buzzer_alarm_is_on()) 
            {
                printf("Silenciando alarme...\n");
                silenciarAlarme();
                alarme_silenciado = true;
            }
            button_a_pressed = false; // Reset flag
        }

        // Fluxo principal
        if (requisitaPosicoes()) 
        {    
            if (comparaPosicoes()) 
            {
                if (posicaoPerigosa()) 
                {
                    // Encontrar qual sensor detectou a posição perigosa e usar seu pitch
                    float max_pitch = 0.0f;
                    int sensor_perigo = -1;
                    for (int i = 0; i < NUM_SENSORS; i++) {
                        if (g_pitch[i] > 90.0f && g_pitch[i] > max_pitch) {
                            max_pitch = g_pitch[i];
                            sensor_perigo = i;
                        }
                    }
                    
                    printf("POSIÇÃO PERIGOSA DETECTADA! Sensor %d - Pitch: %.2f°\n", sensor_perigo, max_pitch);
                    if (!eventoAberto()) 
                    {
                        printf("Criando novo evento de risco...\n");
                        // Cria evento simples para flexão perigosa da perna direita
                        std::string duracao = "0:01:20";
                        Evento* novoEvento = new Evento(TipoMovimento::FLEXAO, LadoCorpo::PERNA_DIR, max_pitch, duracao);
                        eventosAbertos.push_back(novoEvento);
                        printf("Evento criado! Total de eventos abertos: %zu\n", eventosAbertos.size());
                        ligaAlarme();
                    }
                } 
                else 
                {
                    if (eventoAberto()) 
                    {
                        printf("Posição segura retomada. Fechando evento...\n");
                        // Destroi objeto da classe Evento
                        Evento* evento = eventosAbertos.back();
                        armazenarNoSDCard(*evento);
                        printf("Evento salvo no SD Card\n");
                        delete evento;
                        eventosAbertos.pop_back();
                        printf("Evento removido. Eventos restantes: %zu\n", eventosAbertos.size());
                        if (!eventoAberto()) 
                        {
                            printf("Todos os eventos fechados. Desligando alarme...\n");
                            desligaAlarme();
                            alarme_silenciado = false;
                        }
                    }
                }
            }
        }
        else 
        {
            printf("Erro ao ler sensores!\n");
        }
        
        sleep_ms(10); // Taxa de amostragem 100Hz (período = 10ms, DELTA_T = 0.01s)
    }
    return 0;
}

bool eventoAberto() 
{
    return !eventosAbertos.empty();
}

void ligaAlarme() 
{
    if (!alarme_silenciado) {
        printf("LIGANDO ALARME!\n");
        buzzer_alarm_on();
    } else {
        printf("Alarme silenciado - não ativando buzzer\n");
    }
}

void desligaAlarme() 
{
    printf("Desligando alarme\n");
    buzzer_alarm_off();
}

void silenciarAlarme() 
{
    printf("Silenciando alarme\n");
    buzzer_alarm_off();
}

void armazenarNoSDCard(const Evento& evento) 
{
    // Salva dados simples do evento no cartão SD
    const char* duracao = evento.getDuracao().c_str();
    const char* descricao = evento.getDescricao().c_str();
    int angulo = (int)evento.getAngulo();
    const char* tipo = "Postura";

    printf("Salvando evento no SD Card:\n");
    printf("   Duração: %s\n", duracao);
    printf("   Descrição: %s\n", descricao);
    printf("   Ângulo: %d°\n", angulo);
    printf("   Tipo: %s\n", tipo);

    add_csv_record(duracao, descricao, angulo, tipo);
    printf("Evento salvo com sucesso!\n");
}