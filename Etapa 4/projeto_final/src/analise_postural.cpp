#include "analise_postural.h"   // Declarações das funções e estruturas principais de análise postural
#include "evento.h"             // Definição da classe Evento para controle de eventos de postura
#include <vector>                // STL: Estrutura de dados dinâmica para lista de eventos
#include <memory>                // STL: Gerenciamento automático de memória (smart pointers)
#include <ctime>                 // Funções de data e hora padrão C/C++
#include <cmath>                 // Funções matemáticas padrão (ex: trigonometria)
#include "pico/time.h"          // Funções de tempo específicas do Pico SDK

// ===============================
// Definições e Constantes Globais
// ===============================

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

// Conversão de graus para radianos
static float deg_to_rad(float degrees) {
    return degrees * M_PI_F / 180.0f;
}

// Inclusão de módulos C para integração com hardware e algoritmos externos
extern "C" {
    #include "SDCard.h"           // Manipulação do SDCard para salvar eventos
    #include "sensor_watchdog.h"  // Watchdog para monitoramento dos sensores
    #include "buzzer.h"           // Controle do buzzer (alarme sonoro)
    #include "algoritmo_postura.h"// Algoritmo de análise postural
    #include "MadgwickAHRS.h"     // Filtro Madgwick para orientação
}

// ===============================
// Variáveis Globais de Estado
// ===============================

// Lista de eventos ativos (eventos de postura perigosa em andamento)
static std::vector<std::unique_ptr<Evento>> eventos_ativos;

// Estrutura global para controle do estado do alarme
static Alarme alarme_global = {false, false};

// Controle do período de estabilização dos sensores após inicialização
static bool sistema_inicializado = false;
static uint32_t tempo_inicio_ms = 0;
static const uint32_t TEMPO_ESTABILIZACAO_MS = 5000; // 5 segundos

// ===============================
// Funções Auxiliares de Conversão
// ===============================

// Converte enum LadoCorpo para string
static const char* ladoToStr(LadoCorpo l) {
    switch(l){
        case LadoCorpo::DIREITO:  return "DIREITO";
        case LadoCorpo::ESQUERDO: return "ESQUERDO";
        default: return "??";
    }
}

// Converte enum TipoMovimento para string
static const char* movToStr(TipoMovimento m) {
    switch(m){
        case TipoMovimento::FLEXAO:   return "FLEXAO";
        case TipoMovimento::ABDUCAO:  return "ABDUCAO";
        case TipoMovimento::ROTACAO:  return "ROTACAO";
        case TipoMovimento::NORMAL:   return "NORMAL";
        default: return "??";
    }
}

// ===============================
// Função: Verifica se há evento aberto
// ===============================
/**
 * @brief Verifica se já existe um evento aberto para determinada perna e tipo de movimento.
 *
 * Percorre a lista global de eventos ativos e verifica se há algum evento
 * correspondente ao lado (perna) e ao tipo de movimento perigoso informado.
 * Retorna true se encontrar um evento aberto para a combinação especificada,
 * ou se houver um evento aberto para a perna, mas que já saiu do perigo (NORMAL).
 *
 * @param perna  Enum identificando o lado do corpo (DIREITO/ESQUERDO)
 * @param perigo Enum do tipo de movimento perigoso a ser verificado
 * @return true se existe evento aberto para a perna e perigo informados, false caso contrário
 */
static bool isEventOpen(LadoCorpo perna, TipoMovimento perigo) 
{
    // Percorre todos os eventos ativos
    for (const auto& evento : eventos_ativos) 
    {
        // Verifica se o evento corresponde à perna analisada
        if (evento->getLado() == perna) 
        {
            // Se o evento ainda está em situação de perigo (diferente de NORMAL)
            if (evento->getPerigo() != TipoMovimento::NORMAL) 
            {
                // Verifica se o tipo de perigo é o mesmo solicitado
                if (evento->getPerigo() == perigo) 
                {
                    // Já existe evento aberto para esta perna e perigo
                    return true;
                }
            } 
            else 
            {
                // Existe evento aberto para a perna, mas já saiu do perigo
                // (pode ser usado para lógica de encerramento ou transição)
                return true;
            }
        }
    }
    // Não encontrou evento aberto correspondente
    return false;
}

// ===============================
// Função Principal: getPosition
// ===============================
/**
 * @brief Realiza a leitura dos sensores, processa os dados e retorna a orientação (ângulos) da articulação monitorada.
 *
 * Esta função executa toda a cadeia de processamento dos sensores inerciais:
 *  - Lê dados brutos dos sensores MPU9250 (tronco e coxa)
 *  - Alimenta o watchdog para detecção de travamentos
 *  - Lê dados processados dos sensores
 *  - Aplica o filtro Madgwick para obter orientação em quaternion
 *  - Calcula o quaternion relativo entre tronco e coxa
 *  - Extrai ângulos articulares (flexão, abdução, rotação)
 *  - Converte para graus e retorna na estrutura Orientacao
 *
 * @param mpu_list Array de 2 sensores MPU9250 (mpu_list[0]=tronco, mpu_list[1]=coxa)
 * @return Estrutura Orientacao com ângulos de flexão, abdução e rotação em graus
 */
Orientacao getPosition(mpu9250_t mpu_list[2]) 
{
    Orientacao orientacao = {0}; // Inicializa estrutura de retorno zerada

    // === 1. Leitura dos dados brutos dos sensores ===
    mpu9250_raw_data_t raw_data_tronco, raw_data_coxa;
    mpu9250_read_raw(&mpu_list[0], &raw_data_tronco); // Lê dados do sensor do tronco
    mpu9250_read_raw(&mpu_list[1], &raw_data_coxa);   // Lê dados do sensor da coxa

    // Alimenta o watchdog para ambos sensores (detecção de travamento)
    sensor_watchdog_feed(mpu_list[0].id, &raw_data_tronco);
    sensor_watchdog_feed(mpu_list[1].id, &raw_data_coxa);

    // === 2. Leitura dos dados processados dos sensores ===
    mpu9250_data_t data_tronco, data_coxa;
    mpu9250_read_data(&mpu_list[0], &data_tronco); // Dados filtrados do tronco
    mpu9250_read_data(&mpu_list[1], &data_coxa);   // Dados filtrados da coxa

    // === 3. Processamento dos dados com o filtro Madgwick (quaternion) ===
    // Estruturas estáticas para manter estado do filtro entre chamadas
    static AHRS_data_t imu_tronco, imu_coxa;
    static bool initialized = false;
    if (!initialized) 
    {
        // Inicializa o filtro Madgwick para ambos sensores (100Hz)
        MadgwickAHRSinit(&imu_tronco, 100.0f);
        MadgwickAHRSinit(&imu_coxa, 100.0f);
        initialized = true;
    }

    // Preenche estrutura do filtro Madgwick com dados do tronco
    imu_tronco.accel[0] = data_tronco.accel[0];
    imu_tronco.accel[1] = data_tronco.accel[1];
    imu_tronco.accel[2] = data_tronco.accel[2];
    imu_tronco.gyro[0] = deg_to_rad(data_tronco.gyro[0]); // Converte para rad/s
    imu_tronco.gyro[1] = deg_to_rad(data_tronco.gyro[1]);
    imu_tronco.gyro[2] = deg_to_rad(data_tronco.gyro[2]);
    imu_tronco.mag[0] = data_tronco.mag[0];
    imu_tronco.mag[1] = data_tronco.mag[1];
    imu_tronco.mag[2] = data_tronco.mag[2];

    // Preenche estrutura do filtro Madgwick com dados da coxa
    imu_coxa.accel[0] = data_coxa.accel[0];
    imu_coxa.accel[1] = data_coxa.accel[1];
    imu_coxa.accel[2] = data_coxa.accel[2];
    imu_coxa.gyro[0] = deg_to_rad(data_coxa.gyro[0]);
    imu_coxa.gyro[1] = deg_to_rad(data_coxa.gyro[1]);
    imu_coxa.gyro[2] = deg_to_rad(data_coxa.gyro[2]);
    imu_coxa.mag[0] = data_coxa.mag[0];
    imu_coxa.mag[1] = data_coxa.mag[1];
    imu_coxa.mag[2] = data_coxa.mag[2];

    // Atualiza o filtro Madgwick para ambos sensores
    MadgwickAHRSupdate(&imu_tronco);
    MadgwickAHRSupdate(&imu_coxa);

    // === 4. Calcula o quaternion relativo (tronco -> coxa) ===
    // Constrói quaternions a partir dos dados do filtro
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
    // Calcula o quaternion relativo entre tronco e coxa
    Quaternion q_rel = relative_quaternion(q_tronco, q_coxa);

    // === 5. Extrai ângulos articulares relativos (flexão, abdução, rotação) ===
    float flexao_rad, aducao_rad, rotacao_rad;
    
    // Função quaternion_to_hip_angles extrai os ângulos articulares principais a partir do quaternion relativo
    quaternion_to_hip_angles(q_rel, &flexao_rad, &aducao_rad, &rotacao_rad);

    // === 6. Converte ângulos para graus e preenche estrutura de retorno ===
    const float RAD2DEG = 180.0f / M_PI_F;
    orientacao.flexao   =  flexao_rad * RAD2DEG; 
    orientacao.rotacao  =  rotacao_rad * RAD2DEG;
    orientacao.abducao  =  aducao_rad * RAD2DEG; 

    // Log dos ângulos para depuração e acompanhamento em tempo real
    printf("Ângulos: Flexão=%.2f° | Adução=%.2f° | Rotação=%.2f°\n", 
           orientacao.flexao, orientacao.abducao, orientacao.rotacao);

    // === 7. Inicializa controle de tempo na primeira chamada ===
    if (!sistema_inicializado) 
    {
        tempo_inicio_ms = to_ms_since_boot(get_absolute_time());
        sistema_inicializado = true;
        printf("Sistema iniciado - período de estabilização de %d segundos\n", TEMPO_ESTABILIZACAO_MS / 1000);
    }

    return orientacao;
}

// ===============================
// Função Auxiliar: salvarEventoSDCard
// ===============================
/**
 * @brief Salva um evento encerrado no SDCard, registrando lado, tipo de movimento e ângulo máximo.
 * @param evento Ponteiro para o evento a ser salvo
 */
/**
 * @brief Salva um evento de postura no SDCard, registrando lado, tipo de movimento e ângulo máximo.
 *
 * Esta função converte os enums do evento para strings legíveis, chama a função de registro
 * (que já gera os timestamps automaticamente) e faz o log do sucesso ou falha da operação.
 *
 * @param evento Ponteiro único para o evento a ser salvo
 */
static void salvarEventoSDCard(const std::unique_ptr<Evento>& evento) 
{
    // Determina o lado do corpo como string para registro
    const char* lado_str = (evento->getLado() == LadoCorpo::DIREITO) ? "direita" : "esquerda";

    // Converte o tipo de movimento perigoso para string
    const char* movimento_str = movToStr(evento->getPerigo());

    // Chama a função de registro, que salva o evento no SDCard e gera timestamps automaticamente
    bool sucesso = register_movement_with_timestamps(lado_str, movimento_str, evento->getMaxAngulo());

    // Loga o resultado da operação para depuração e acompanhamento
    if (sucesso) 
    {
        printf("[SDCard] Evento salvo: Lado=%s, Movimento=%s, Ângulo=%.2f°, Duração=%lld ms\n", 
               lado_str, movimento_str, evento->getMaxAngulo(), evento->getDuracaoMS());
    } 
    else 
    {
        printf("[SDCard] ERRO ao salvar evento: Lado=%s, Movimento=%s, Ângulo=%.2f°\n", 
               lado_str, movimento_str, evento->getMaxAngulo());
    }
}

// ===============================
// Função Auxiliar: gerenciarAlarme
// ===============================
/**
 * @brief Liga ou desliga o alarme sonoro conforme o estado do sistema e eventos ativos.
 * @param ligar Se true, liga o alarme; se false, verifica se deve desligar
 */
/**
 * @brief Gerencia o estado do alarme sonoro (buzzer) conforme a situação do sistema e dos eventos ativos.
 *
 * Esta função centraliza toda a lógica de ativação e desativação do alarme:
 *  - Liga o alarme se solicitado e ainda não estiver ligado
 *  - Garante que o buzzer só toca se não estiver silenciado
 *  - Desliga o alarme quando não há mais eventos ativos
 *  - Garante que o buzzer permanece desligado se o alarme estiver silenciado
 *
 * @param ligar Se true, solicita ligar o alarme; se false, solicita desligar
 */
static void gerenciarAlarme(bool ligar) 
{
    // Caso 1: Solicitação para ligar o alarme e ele ainda não está ligado
    if (ligar && !alarme_global.ligado) 
    {
        alarme_global.ligado = true;        // Marca o alarme como ligado
        alarme_global.silenciado = false;   // Garante que não está silenciado
        if (!alarme_global.silenciado) 
        {
            buzzer_alarm_on();              // Ativa o buzzer
        }
        printf("[ALARME] LIGADO - Postura perigosa detectada!\n");
    }

    // Caso 2: Solicitação para ligar, já está ligado, mas pode ter sido desilenciado
    else if (ligar && alarme_global.ligado && !alarme_global.silenciado) 
    {
        buzzer_alarm_on();                  // Garante que o buzzer está ativo
    }

    // Caso 3: Solicitação para desligar o alarme
    else if (!ligar && alarme_global.ligado) 
    {
        // Só desliga se não houver mais eventos ativos
        if (eventos_ativos.empty()) 
        {
            alarme_global.ligado = false;
            alarme_global.silenciado = false;
            buzzer_alarm_off();
            printf("[ALARME] DESLIGADO - Postura normalizada\n");
        }
    }

    // Caso 4: Se o alarme está ligado mas silenciado, garante que o buzzer está desligado
    if (alarme_global.ligado && alarme_global.silenciado) 
    {
        buzzer_alarm_off();
    }
}

// ===============================
// Função Principal: dangerCheck
// ===============================
/**
 * @brief Verifica se a postura está em situação de risco, gerencia alarmes e eventos, e salva eventos no SDCard.
 * @param orientacao Estrutura com ângulos de flexão, abdução e rotação
 */
/**
 * @brief Analisa a orientação atual e gerencia eventos e alarmes de postura perigosa.
 *
 * Esta função executa a lógica principal de detecção de risco postural:
 *  - Aguarda o período de estabilização dos sensores antes de iniciar a análise
 *  - Para cada tipo de movimento relevante (flexão, abdução, rotação):
 *      - Verifica se o ângulo atual ultrapassa o limite seguro
 *      - Se sim, abre ou atualiza um evento e liga o alarme
 *      - Se não, encerra o evento (se houver) e salva no SDCard
 *  - Ao final, exibe o status dos eventos ativos para depuração
 *
 * @param orientacao Estrutura com ângulos de flexão, abdução e rotação
 */
void dangerCheck(Orientacao orientacao) 
{
    // === 1. Aguarda estabilização dos sensores após inicialização ===
    if (sistema_inicializado) 
    {
        uint32_t tempo_atual_ms = to_ms_since_boot(get_absolute_time());
        uint32_t tempo_decorrido_ms = tempo_atual_ms - tempo_inicio_ms;

        // Se ainda está no período de estabilização, exibe tempo restante e retorna
        if (tempo_decorrido_ms < TEMPO_ESTABILIZACAO_MS) 
        {
            uint32_t tempo_restante_ms = TEMPO_ESTABILIZACAO_MS - tempo_decorrido_ms;
            printf("Estabilizando sensores... %d.%d segundos restantes\n", 
                   tempo_restante_ms / 1000, (tempo_restante_ms % 1000) / 100);
            return;
        }

        // Primeira vez após estabilização: sinaliza sistema ativo
        else if (tempo_decorrido_ms == TEMPO_ESTABILIZACAO_MS || (tempo_decorrido_ms > TEMPO_ESTABILIZACAO_MS && tempo_decorrido_ms < TEMPO_ESTABILIZACAO_MS + 1000)) 
        {
            printf("Período de estabilização concluído - sistema ativo!\n");
            buzzer_beep();
        }
    }

    // === 2. Verifica cada tipo de movimento relevante (exceto NORMAL) ===
    TipoMovimento tipos_movimento[] = {TipoMovimento::FLEXAO, TipoMovimento::ABDUCAO, TipoMovimento::ROTACAO};
    for (TipoMovimento tipo : tipos_movimento) 
    {
        bool posicao_perigosa = false;
        float angulo_atual = 0.0f;

        // Determina o ângulo atual e se está em situação perigosa
        switch (tipo) 
        {
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
                continue;
        }

        // Por padrão, monitorando a perna direita (pode ser adaptado para o outro lado)
        LadoCorpo perna_atual = LadoCorpo::DIREITO;

        if (posicao_perigosa) 
        {
            // === Situação perigosa detectada ===
            // Se não há evento aberto para este tipo, cria novo evento e liga o alarme
            if (!isEventOpen(perna_atual, tipo)) 
            {
                auto novo_evento = std::make_unique<Evento>(tipo, perna_atual, angulo_atual);
                eventos_ativos.push_back(std::move(novo_evento));
                gerenciarAlarme(true);
                printf("NOVO EVENTO CRIADO: %s - %s (%.2f graus)\n", 
                       ladoToStr(perna_atual), movToStr(tipo), angulo_atual);
            } 
            else 
            {
                // Se já existe evento aberto, apenas atualiza o ângulo máximo
                for (auto& evento : eventos_ativos) 
                {
                    if (evento->getLado() == perna_atual && evento->getPerigo() == tipo) 
                    {
                        evento->setAngulo(angulo_atual);
                        break;
                    }
                }
            }
        } 
        else 
        {
            // === Situação normalizada: encerra evento se houver ===
            auto it = eventos_ativos.begin();
            while (it != eventos_ativos.end()) 
            {
                if ((*it)->getLado() == perna_atual && (*it)->getPerigo() == tipo) 
                {
                    (*it)->closeEvent();
                    salvarEventoSDCard(*it);
                    printf("EVENTO ENCERRADO: %s - %s (%.2f graus max, %lld ms)\n", 
                           ladoToStr((*it)->getLado()), movToStr((*it)->getPerigo()), 
                           (*it)->getMaxAngulo(), (*it)->getDuracaoMS());
                    it = eventos_ativos.erase(it);
                    gerenciarAlarme(false);
                    break;
                } 
                else 
                {
                    ++it;
                }
            }
        }
    }

    // === 3. Log de eventos ativos para depuração e acompanhamento ===
    if (!eventos_ativos.empty()) 
    {
        printf("Eventos ativos: %zu\n", eventos_ativos.size());
        for (const auto& evento : eventos_ativos) 
        {
            printf("  - %s %s: %.2f graus, %lld ms\n", 
                   ladoToStr(evento->getLado()), movToStr(evento->getPerigo()),
                   evento->getMaxAngulo(), evento->getDuracaoMS());
        }
    }
}

// -------------------------------------------------------------------
// Funções de Controle Manual do Alarme Sonoro (Buzzer)
// -------------------------------------------------------------------

/**
 * @brief Silencia manualmente o alarme sonoro (buzzer).
 *
 * Esta função permite ao usuário silenciar o alarme, mesmo que uma condição de risco persista.
 * O estado de "silenciado" é registrado na estrutura global do alarme, e o buzzer é desligado imediatamente.
 * Um log é impresso para indicar a ação do usuário.
 */
void silenciar_alarme(void) 
{
    // Marca o alarme como silenciado (não emitirá som até ser desilenciado)
    alarme_global.silenciado = true;
    // Garante que o buzzer está desligado fisicamente
    buzzer_alarm_off();
    // Log para depuração e rastreabilidade
    printf("[ALARME] Silenciado manualmente pelo usuário\n");
}

/**
 * @brief Desfaz o silenciamento do alarme, religando o buzzer se necessário.
 *
 * Esta função remove o estado de "silenciado" do alarme. Caso o alarme ainda esteja ativo (ligado),
 * o buzzer é religado automaticamente para alertar o usuário. Um log é impresso para indicar a ação.
 */
void desilenciar_alarme(void) 
{
    // Remove o estado de silenciado
    alarme_global.silenciado = false;
    // Se o alarme está ativo, religa o buzzer imediatamente
    if (alarme_global.ligado) 
    {
        buzzer_alarm_on();
        printf("[ALARME] Desilenciado - buzzer religado\n");
    }
}

/**
 * @brief Consulta se o alarme está atualmente ligado (ativo).
 *
 * @return true se o alarme está ligado (situação de risco detectada), false caso contrário.
 */
bool alarme_esta_ligado(void) 
{
    return alarme_global.ligado;
}

/**
 * @brief Consulta se o alarme está silenciado manualmente.
 *
 * @return true se o alarme está silenciado pelo usuário, false caso contrário.
 */
bool alarme_esta_silenciado(void) 
{
    return alarme_global.silenciado;
}