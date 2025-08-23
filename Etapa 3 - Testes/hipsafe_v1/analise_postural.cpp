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
    printf("Mantenha o sensor IMÓVEL durante a calibração!\n");
    
    // Som de início da calibração (3 beeps curtos e rápidos)
    printf("Emitindo sinal sonoro de início...\n");
    for (int i = 0; i < 3; i++) {
        buzzer_beep(100); // Beep curto de 100ms
        sleep_ms(150);    // Pausa de 150ms entre beeps
    }
    sleep_ms(500); // Pausa antes de iniciar calibração
    
    FusionVector accel_sum = {0};
    FusionVector gyro_sum = {0};
    
    // Coletar amostras para calibração
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) 
    {
        int16_t raw_accel[3], raw_gyro[3], temp;
        FusionVector accel, gyro;
        
        mpu6050_read_raw(raw_accel, raw_gyro, &temp);
        convert_mpu6050_data(raw_accel, raw_gyro, &accel, &gyro);
        
        // Acumular valores
        accel_sum = FusionVectorAdd(accel_sum, accel);
        gyro_sum = FusionVectorAdd(gyro_sum, gyro);
        
        // Indicador de progresso
        if ((i + 1) % 100 == 0) 
        {
            printf("Calibração: %d/%d amostras\n", i + 1, CALIBRATION_SAMPLES);
        }
        
        sleep_ms(10); // 100Hz
    }
    
    // Calcular offsets médios
    float scale = 1.0f / (float)CALIBRATION_SAMPLES;
    gyro_cal.offset = FusionVectorMultiplyScalar(gyro_sum, scale);
    
    // Para o acelerômetro, assumir que Z = 1g quando estático
    accel_cal.offset.axis.x = accel_sum.axis.x / CALIBRATION_SAMPLES;
    accel_cal.offset.axis.y = accel_sum.axis.y / CALIBRATION_SAMPLES;
    accel_cal.offset.axis.z = (accel_sum.axis.z / CALIBRATION_SAMPLES) - 1.0f; // Compensar gravidade
    
    // Inicializar sensibilidade como identidade
    accel_cal.sensitivity = (FusionVector){.axis = {1.0f, 1.0f, 1.0f}};
    gyro_cal.sensitivity = (FusionVector){.axis = {1.0f, 1.0f, 1.0f}};
    
    // Inicializar matriz de desalinhamento como identidade
    accel_cal.misalignment = FUSION_IDENTITY_MATRIX;
    gyro_cal.misalignment = FUSION_IDENTITY_MATRIX;
    
    accel_cal.calibrated = true;
    gyro_cal.calibrated = true;
    
    // Som de fim da calibração (1 beep longo e grave)
    printf("Calibração concluída!\n");
    printf("Emitindo sinal sonoro de conclusão...\n");
    buzzer_beep(800); // Beep longo de 800ms para indicar sucesso
    
    printf("Gyro offset: X=%.3f, Y=%.3f, Z=%.3f deg/s\n", 
           gyro_cal.offset.axis.x, gyro_cal.offset.axis.y, gyro_cal.offset.axis.z);
    printf("Accel offset: X=%.3f, Y=%.3f, Z=%.3f g\n", 
           accel_cal.offset.axis.x, accel_cal.offset.axis.y, accel_cal.offset.axis.z);
}

// Função para configurar o algoritmo AHRS
void initialize_fusion_system() 
{
    printf("Configurando sistema Fusion AHRS...\n");
    
    // Inicializar AHRS
    FusionAhrsInitialise(&ahrs);
    
    // Configurar parâmetros otimizados para aplicação na perna
    FusionAhrsSettings settings = {
        .convention = FusionConventionNwu,       // North-West-Up
        .gain = 0.5f,                           // Ganho moderado para estabilidade
        .gyroscopeRange = 2000.0f,              // Range máximo do giroscópio em deg/s
        .accelerationRejection = 10.0f,         // Rejeição de aceleração (movimentos)
        .magneticRejection = 0.0f,              // Não usado (sem magnetômetro)
        .recoveryTriggerPeriod = (unsigned int)(5 * SAMPLE_RATE) // Período de recuperação (5 segundos)
    };
    
    FusionAhrsSetSettings(&ahrs, &settings);
    
    fusion_initialized = true;
    
    printf("AHRS configurado com:\n");
    printf("  - Ganho: %.1f\n", settings.gain);
    printf("  - Rejeição de aceleração: %.1f\n", settings.accelerationRejection);
    printf("  - Taxa de amostragem: %.1f Hz\n", SAMPLE_RATE);
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
    
    // Inicializar I2C para o MPU6050
    printf("Configurando sensor MPU6050...\n");
    mpu6050_setup_i2c();
    mpu6050_reset(); // Reinicia o sensor
    
    // Configura o sensor para ±4g e ±500°/s (otimizado para aplicação na perna)
    mpu6050_set_accel_range(1); // 1 = ±4g (movimentos moderados)
    mpu6050_set_gyro_range(1);  // 1 = ±500°/s (rotações moderadas)
    printf("MPU6050 configurado: ±4g, ±500°/s\n");
    
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
                    printf("POSIÇÃO PERIGOSA DETECTADA! Roll: %.2f°\n", g_roll);
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