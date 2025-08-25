#include <iostream>
#include <vector>
#include "pico/stdlib.h"

// Headers C
extern "C" {
    #include "drivers/button/button.h"
    #include "drivers/buzzer/buzzer.h"
    #include "drivers/sdcard/SDCard.h"
    #include "drivers/mpu6050/mpu6050_i2c.h"
    #include "drivers/filter/MadgwickAHRS.h"
    #include "analise_postural_sensor.h"
}

// Headers C++
#include "evento/evento.h"

// Declarações de funções
bool eventoAberto();
void ligaAlarme();
void desligaAlarme();
void silenciarAlarme();
void armazenarNoSDCard(const Evento& evento);

// Variáveis globais
bool alarme_silenciado = false;
std::vector<Evento*> eventosAbertos;
static int contador_prints = 0; // Contador para reduzir frequência dos prints (100Hz seria muito output)

int main() 
{
    // Inicialização do sistema
    stdio_init_all(); // Inicializa UART/USB para debug
    
    printf("=== HIPSAFE v1 - Sistema de Monitoramento Postural ===\n");
    printf("Iniciando sistema...\n");
    
    // Inicializar I2C para o MPU6050
    printf("Configurando sensor MPU6050...\n");
    mpu6050_setup_i2c();
    mpu6050_reset(); // Reinicia o sensor
    
    // Configura o sensor para ±2g (mais sensível) e ±250°/s
    mpu6050_set_accel_range(0); // 0 = ±2g
    mpu6050_set_gyro_range(0);  // 0 = ±250°/s
    printf("MPU6050 configurado: ±2g, ±250°/s\n");
    
    // Inicializar botões
    printf("Inicializando botões...\n");
    setup_buttons();
    
    // Inicializar buzzer
    printf("Inicializando buzzer...\n");
    buzzer_init();
    
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
            // Mostra dados apenas a cada 50 iterações (~2Hz) para não sobrecarregar o terminal
            contador_prints++;
            if (contador_prints >= 50) {
                printf("Posições lidas - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f° [100Hz]\n", g_roll, g_pitch, g_yaw);
                contador_prints = 0;
            }
            
            if (comparaPosicoes()) 
            {
                if (posicaoPerigosa()) 
                {
                    printf("POSIÇÃO PERIGOSA DETECTADA! Roll: %.2f°\n", g_pitch);
                    if (!eventoAberto()) 
                    {
                        printf("Criando novo evento de risco...\n");
                        // Cria evento simples para flexão perigosa da perna direita
                        std::string duracao = "0:01:20";
                        Evento* novoEvento = new Evento(TipoMovimento::FLEXAO, LadoCorpo::PERNA_DIR, g_roll, duracao);
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
            else 
            {
                if (contador_prints == 0) { // Mostra apenas quando outros dados são exibidos
                    printf("Posições fora dos limites seguros\n");
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