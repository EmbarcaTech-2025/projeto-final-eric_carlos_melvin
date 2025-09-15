#include "analise_postural.h"
#include "evento.h"
#include <vector>
#include <memory>
#include <ctime>
#include <cmath>

// Definições de constantes matemáticas
#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// Função auxiliar para conversão de graus para radianos
static float deg_to_rad(float degrees) {
    return degrees * M_PI_F / 180.0f;
}

// Includes para integração com outros módulos
extern "C" {
    // #include "SDCard.h"  // Comentado - não usando SDCard por enquanto
    #include "buzzer.h"
    #include "algoritmo_postura.h"
    #include "MadgwickAHRS.h"
}

// Variáveis globais para gerenciamento de eventos
static std::vector<std::unique_ptr<Evento>> eventos_ativos;
static Alarme alarme_global = {false, false};

static const char* ladoToStr(LadoCorpo l) {
    switch(l){
        case LadoCorpo::DIREITO:  return "DIREITO";
        case LadoCorpo::ESQUERDO: return "ESQUERDO";
        default: return "??";
    }
}

static const char* movToStr(TipoMovimento m) {
    switch(m){
        case TipoMovimento::FLEXAO:   return "FLEXAO";
        case TipoMovimento::ABDUCAO:  return "ABDUCAO";
        case TipoMovimento::ROTACAO:  return "ROTACAO";
        case TipoMovimento::NORMAL:   return "NORMAL";
        default: return "??";
    }
}

/***
 * @brief Função "local" que verifica se um evento está aberto para determinada 
 * perna e tipo de movimento.
 * @param[in] perna   Identificador da perna analisada (enum LadoCorpo).
 * @param[in] perigo  Tipo de movimento considerado perigoso (enum TipoMovimento).
 * @return `true` se existe algum evento aberto para a perna e perigo informados;  
 *         `false` caso contrário.
 */
static bool isEventOpen(LadoCorpo perna, TipoMovimento perigo) {
    // Para cada evento dentro do vetor eventos (variável global):
    for (const auto& evento : eventos_ativos) {
        // é a mesma perna?
        if (evento->getLado() == perna) {
            // tem perigo?
            if (evento->getPerigo() != TipoMovimento::NORMAL) {
                // é o mesmo perigo?
                if (evento->getPerigo() == perigo) {
                    return true; // evento aberto para esta perna e perigo
                }
            } else {
                // não tem perigo - evento aberto, mas já saiu do perigo
                return true;
            }
        }
    }
    // Se passou por todos os eventos da lista, retorna false
    return false;
}

// RELATIVO A FUNÇÕES PRINCIPAIS:
/***
 * @brief ORGANIZA a coleta dos parâmetros passados pelas MPUs, até a
 * saída de Orientação
 * @param[in] mpu_list[2] Recebe uma lista com 2 objetos da MPU9250 (tronco e coxa)
 * @return Orientacao - ângulos em graus de flexão, rotação e abdução
 * dentro de uma struct Orientacao
 */
Orientacao getPosition(mpu9250_t mpu_list[2]) {
    /*
        Implementação do algoritmo conforme especificação:
        (1) Requisita os dados "brutos" dos dois sensores da lista de sensores
        (2) Transforma eles em quaternions com algoritmo Madgwick
        (3) Descobre o quaternion relativo entre eles 
        (4) Descobre os ângulos relativos aos movimentos (Orientacao)
        (5) Retorna esta Orientação   
        
        Utiliza mpu_list[0] (tronco) e mpu_list[1] (coxa/perna)
        Não há terceiro sensor nesta implementação
    */

    /*
        Implementação do algoritmo baseado em quaternions:
        (1) Requisita os dados "brutos" dos dois sensores da lista de sensores
        (2) Transforma eles em quaternions com o algoritmo de Madgwick AHRS
        (3) Descobre o quaternion relativo entre eles.
    */
    
    Orientacao orientacao_quat = {0};
    
    // (1) Leitura dos dados brutos dos sensores
    mpu9250_data_t data_tronco, data_coxa;
    
    // Lê dados do sensor do tronco (mpu_list[0])
    mpu9250_read_data(&mpu_list[0], &data_tronco);
    
    // Lê dados do sensor da coxa (mpu_list[1])
    mpu9250_read_data(&mpu_list[1], &data_coxa);
    

    // (2) Transformação em quaternions usando algoritmo Madgwick
    // Estruturas AHRS para os sensores (locais para esta função)
    static AHRS_data_t imu_tronco, imu_coxa;
    static bool initialized = false;
    
    // Inicialização dos algoritmos Madgwick (apenas na primeira chamada)
    if (!initialized) {
        MadgwickAHRSinit(&imu_tronco, 100.0f); // 100 Hz
        MadgwickAHRSinit(&imu_coxa, 100.0f);   // 100 Hz
        initialized = true;
    }
    
    // Preparação dos dados para o algoritmo Madgwick (tronco)
    imu_tronco.accel[0] = data_tronco.accel[0];
    imu_tronco.accel[1] = data_tronco.accel[1];
    imu_tronco.accel[2] = data_tronco.accel[2];
    imu_tronco.gyro[0] = deg_to_rad(data_tronco.gyro[0]);
    imu_tronco.gyro[1] = deg_to_rad(data_tronco.gyro[1]);
    imu_tronco.gyro[2] = deg_to_rad(data_tronco.gyro[2]);
    imu_tronco.mag[0] = data_tronco.mag[0];
    imu_tronco.mag[1] = data_tronco.mag[1];
    imu_tronco.mag[2] = data_tronco.mag[2];
    
    // Preparação dos dados para o algoritmo Madgwick (coxa)
    imu_coxa.accel[0] = data_coxa.accel[0];
    imu_coxa.accel[1] = data_coxa.accel[1];
    imu_coxa.accel[2] = data_coxa.accel[2];
    imu_coxa.gyro[0] = deg_to_rad(data_coxa.gyro[0]);
    imu_coxa.gyro[1] = deg_to_rad(data_coxa.gyro[1]);
    imu_coxa.gyro[2] = deg_to_rad(data_coxa.gyro[2]);
    imu_coxa.mag[0] = data_coxa.mag[0];
    imu_coxa.mag[1] = data_coxa.mag[1];
    imu_coxa.mag[2] = data_coxa.mag[2];
    
    // Atualização dos algoritmos Madgwick
    MadgwickAHRSupdate(&imu_tronco);
    MadgwickAHRSupdate(&imu_coxa);
    
    // Conversão dos quaternions do Madgwick para a estrutura Quaternion
    Quaternion q_tronco = { 
        .w = imu_tronco.orientation.q0, 
        .x = imu_tronco.orientation.q1, 
        .y = imu_tronco.orientation.q2, 
        .z = imu_tronco.orientation.q3 
    };
    
    Quaternion q_coxa = { 
        .w = imu_coxa.orientation.q0, 
        .x = imu_coxa.orientation.q1, 
        .y = imu_coxa.orientation.q2, 
        .z = imu_coxa.orientation.q3 
    };
    
    // * Metodo 2
    // Normalização
    q_tronco = quaternion_normalize(q_tronco);
    q_coxa   = quaternion_normalize(q_coxa);

    // q_leg_t_ref = conj(tronco) * coxa
    Quaternion q_conj = quaternion_conjugate(q_tronco);
    Quaternion q_leg_t_ref = quaternion_multiply(q_conj, q_coxa);
    q_leg_t_ref = quaternion_normalize(q_leg_t_ref);

    // Partes do quaternio
    double qw = q_leg_t_ref.w;
    double v_leg_t_ref[3] = {q_leg_t_ref.x, q_leg_t_ref.y, q_leg_t_ref.z};

    // Ângulos (como no MATLAB)
    double theta_flexion   = 2.0 * atan2( v_leg_t_ref[0], qw);
    double theta_abduction = 2.0 * atan2( v_leg_t_ref[2], qw);
    double theta_rotation  = 2.0 * atan2( v_leg_t_ref[1], qw);

    orientacao_quat.flexao   = (180.0 / M_PI)*(theta_flexion);
    orientacao_quat.abducao  = (180.0 / M_PI)*(theta_abduction);
    orientacao_quat.rotacao  = (180.0 / M_PI)*(theta_rotation);

    // Imprime os ângulos continuamente
    printf("Ângulos (2): Flexão=%.2f° | Adução=%.2f° | Rotação=%.2f° \n", 
           orientacao_quat.flexao, orientacao_quat.abducao, orientacao_quat.rotacao);

    return orientacao_quat;
}

/***
 * @brief Função auxiliar para salvar evento no SDCard
 * @param evento Ponteiro para o evento a ser salvo
 */
static void salvarEventoSDCard(const std::unique_ptr<Evento>& evento) {
    // FUNCIONALIDADE SDCARD COMENTADA - não usando por enquanto
    /*
    // Converte timestamps para formato ISO
    auto inicio_time_t = std::chrono::system_clock::to_time_t(evento->getInicio());
    auto fim_time_t = std::chrono::system_clock::to_time_t(evento->getFim());
    
    char inicio_str[25], fim_str[25];
    std::strftime(inicio_str, sizeof(inicio_str), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&inicio_time_t));
    std::strftime(fim_str, sizeof(fim_str), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&fim_time_t));
    
    // Converte enums para strings
    const char* lado_str = (evento->getLado() == LadoCorpo::DIREITO) ? "direita" : "esquerda";
    const char* movimento_str = movToStr(evento->getPerigo());
    
    // Salva no SDCard
    bool sucesso = add_csv_record(inicio_str, fim_str, lado_str, movimento_str, evento->getMaxAngulo());
    
    if (sucesso) {
        printf("Evento salvo no SDCard: %s, %s, %.2f graus, duração: %lld ms\n", 
               lado_str, movimento_str, evento->getMaxAngulo(), evento->getDuracaoMS());
    } else {
        printf("Erro ao salvar evento no SDCard\n");
    }
    */
    
    // Por enquanto, apenas mostra no console
    const char* lado_str = (evento->getLado() == LadoCorpo::DIREITO) ? "direita" : "esquerda";
    const char* movimento_str = movToStr(evento->getPerigo());
    
    printf("Evento finalizado (SDCard desabilitado): %s, %s, %.2f graus, duração: %lld ms\n", 
           lado_str, movimento_str, evento->getMaxAngulo(), evento->getDuracaoMS());
}

/***
 * @brief Função auxiliar para gerenciar alarme
 * @param ligar Se true, liga o alarme; se false, verifica se deve desligar
 */
static void gerenciarAlarme(bool ligar) {
    if (ligar && !alarme_global.ligado) {
        alarme_global.ligado = true;
        alarme_global.silenciado = false;
        buzzer_alarm_on(); // Liga o buzzer
        printf("ALARME LIGADO - Postura perigosa detectada!\n");
    } else if (!ligar && alarme_global.ligado) {
        // Verifica se ainda há eventos ativos
        if (eventos_ativos.empty()) {
            alarme_global.ligado = false;
            alarme_global.silenciado = false;
            buzzer_alarm_off(); // Desliga o buzzer
            printf("ALARME DESLIGADO - Postura normalizada\n");
        }
    }
}

/***
 * @brief ORGANIZA / IMPLEMENTA o algorítmo de checkagem de risco de posição,
 * incluindo o gerenciamento dos alarmes e da gravação dos eventos no SDCard.
 * @param Orientacao - Recebe os ângulos de flexão, rotação e abdução.
 * @return void. 
 */
void dangerCheck(Orientacao orientacao) {
    // Array de tipos de movimento para verificar (ignoramos NORMAL)
    TipoMovimento tipos_movimento[] = {TipoMovimento::FLEXAO, TipoMovimento::ABDUCAO, TipoMovimento::ROTACAO};
    
    // Para cada tipo de movimento
    for (TipoMovimento tipo : tipos_movimento) {
        bool posicao_perigosa = false;
        float angulo_atual = 0.0f;
        
        // Verifica se a posição é perigosa baseada no tipo de movimento
        switch (tipo) {
            case TipoMovimento::FLEXAO:
                angulo_atual = orientacao.flexao;
                posicao_perigosa = (angulo_atual > LIMITACOES[static_cast<int>(TipoMovimento::FLEXAO)]);
                break;
            case TipoMovimento::ABDUCAO:
                angulo_atual = orientacao.abducao;
                posicao_perigosa = (angulo_atual > LIMITACOES[static_cast<int>(TipoMovimento::ABDUCAO)]);
                break;
            case TipoMovimento::ROTACAO:
                angulo_atual = orientacao.rotacao;
                posicao_perigosa = (angulo_atual > LIMITACOES[static_cast<int>(TipoMovimento::ROTACAO)]);
                break;
            default:
                continue; // Pula tipos inválidos
        }
        
        // Assumindo que estamos monitorando a perna direita por enquanto
        // Em uma implementação completa, isso seria determinado pela entrada
        LadoCorpo perna_atual = LadoCorpo::DIREITO;
        
        if (posicao_perigosa) {
            // Posição perigosa detectada
            if (!isEventOpen(perna_atual, tipo)) {
                // Não há evento aberto para esta perna e tipo de movimento
                // Cria um novo evento
                auto novo_evento = std::make_unique<Evento>(tipo, perna_atual, angulo_atual);
                eventos_ativos.push_back(std::move(novo_evento));
                
                // Liga o alarme
                gerenciarAlarme(true);
                
                printf("NOVO EVENTO CRIADO: %s - %s (%.2f graus)\n", 
                       ladoToStr(perna_atual), movToStr(tipo), angulo_atual);
            } else {
                // Evento já existe, atualiza o ângulo máximo
                for (auto& evento : eventos_ativos) {
                    if (evento->getLado() == perna_atual && evento->getPerigo() == tipo) {
                        evento->setAngulo(angulo_atual);
                        break;
                    }
                }
            }
        } else {
            // Posição segura
            // Verifica se algum evento está aberto para esta posição
            auto it = eventos_ativos.begin();
            while (it != eventos_ativos.end()) {
                if ((*it)->getLado() == perna_atual && (*it)->getPerigo() == tipo) {
                    // Encontrou evento aberto para esta perna e tipo
                    // Encerra o evento
                    (*it)->closeEvent();
                    
                    // Salva no SDCard
                    salvarEventoSDCard(*it);
                    
                    printf("EVENTO ENCERRADO: %s - %s (%.2f graus max, %lld ms)\n", 
                           ladoToStr((*it)->getLado()), movToStr((*it)->getPerigo()), 
                           (*it)->getMaxAngulo(), (*it)->getDuracaoMS());
                    
                    // Remove da lista
                    it = eventos_ativos.erase(it);
                    
                    // Verifica se deve desligar o alarme
                    gerenciarAlarme(false);
                    break;
                } else {
                    ++it;
                }
            }
        }
    }
    
    // Debug: mostra status atual
    if (!eventos_ativos.empty()) {
        printf("Eventos ativos: %zu\n", eventos_ativos.size());
        for (const auto& evento : eventos_ativos) {
            printf("  - %s %s: %.2f graus, %lld ms\n", 
                   ladoToStr(evento->getLado()), movToStr(evento->getPerigo()),
                   evento->getMaxAngulo(), evento->getDuracaoMS());
        }
    }
};
