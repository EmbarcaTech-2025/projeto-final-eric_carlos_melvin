#include "evento.h"

/******************************************************************
 Função: Evento
 Finalidade: Construtor da classe Evento para inicializar eventos de monitoramento de postura
 Entradas: TipoMovimento movimento - FLEXAO, ABDUCAO, ROTACAO
           LadoCorpo lado - DIREITO, ESQUERDO
           CallBack - função a ser chamada para salvar o evento. Esta função deve receber um 
                único parâmetro (String) com formatação de JSON 
           Float anguloInicial - registra o ângulo inicial ou zero se não for passado este arg.
 Saídas: Void
 ******************************************************************/
Evento::Evento(TipoMovimento movimento, LadoCorpo lado, float anguloInicial)
    : movimento_(movimento),            // salva as informações passadas 
      lado_(lado),                      //     nas variáveis do objeto
      angulo_(anguloInicial),            
      inicio_(std::chrono::system_clock::now()),  
      start_(std::chrono::steady_clock::now()),
      fim_()
{ 
    // não precisa fazer nada...
}


/******************************************************************
 Função : setAngulo
 Finalidade : Esta função registra ou atualiza o ângulo do evento
        apenas se este for maior que o registrado anteriormente.
 Entradas : Nenhuma
 Saídas : float: Medição do ângulo em graus
 ******************************************************************/
void Evento::setAngulo(float a)
{
    if(!closed_) {
        if (angulo_ < a) angulo_ = a;
    }    
}

/******************************************************************
 Função : getDuracao
 Finalidade : Esta função retorna a duração, até este momento, do 
              evento registrado. Se já fechou o Evento, retorna o 
              tempo que esteve aberto.
 Entradas : Nenhuma
 Saídas : std::string: Duração do evento como string
 ******************************************************************/
std::int64_t Evento::getDuracaoMS(void) const 
{
    if (!closed_) {
        auto agora = std::chrono::steady_clock::now();
        auto duracao = std::chrono::duration_cast<std::chrono::milliseconds>(agora - start_);
        return duracao.count();
    }
    else {
        // Se chegou aqui é porque já está fechado
        auto duracao = std::chrono::duration_cast<std::chrono::milliseconds>(fim_ - inicio_);
        return duracao.count();  // retorna inteiro em ms
    }
}

void Evento::closeEvent(void)
{
    if (closed_) return;
    /* Aqui ele vai:
     (1) Registrar em si próprio a hora do término do evento
     (2) Fechar o evento para impedir modificações
    */
    auto agora = std::chrono::steady_clock::now();
    auto duracao_total_ = std::chrono::duration_cast<std::chrono::milliseconds>(agora - start_);
    fim_ = inicio_ + std::chrono::duration_cast<std::chrono::system_clock::duration>(duracao_total_);
    closed_ = true;
}

// std::string Evento::buildJson(void) const
// {
    
// }

/******************************************************************
 Função : buildJson
 Finalidade : Constrói uma string JSON com os dados do evento
 Entradas : Nenhuma
 Saídas : std::string - JSON formatado com os dados do evento
 ******************************************************************/
std::string Evento::buildJson(void) const
{
    // OPÇÃO 1: Formato brasileiro (dd/mm/aaaa hh:mm:ss)
    auto formatoBrasileiro = [](const std::chrono::system_clock::time_point& tp) -> std::string {
        auto time_t = std::chrono::system_clock::to_time_t(tp);
        auto tm = *std::localtime(&time_t);
        
        char buffer[72];
        snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d %02d:%02d:%02d",
                tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
        
        return std::string(buffer);
    };

    // // Função auxiliar simples para converter time_point para timestamp Unix
    // auto timePointToUnix = [](const std::chrono::system_clock::time_point& tp) -> std::int64_t {
    //     return std::chrono::duration_cast<std::chrono::milliseconds>(
    //         tp.time_since_epoch()
    //     ).count();
    // };
    
    // Função auxiliar para converter TipoMovimento para string
    const char* movimentoStr;
    switch(movimento_) {
        case TipoMovimento::FLEXAO: movimentoStr = "FLEXAO"; break;
        case TipoMovimento::ABDUCAO: movimentoStr = "ABDUCAO"; break;
        case TipoMovimento::ROTACAO: movimentoStr = "ROTACAO"; break;
        case TipoMovimento::NORMAL: movimentoStr = "NORMAL"; break;
        default: movimentoStr = "DESCONHECIDO"; break;
    }
    
    // Função auxiliar para converter LadoCorpo para string
    const char* ladoStr;
    switch(lado_) {
        case LadoCorpo::DIREITO: ladoStr = "DIREITO"; break;
        case LadoCorpo::ESQUERDO: ladoStr = "ESQUERDO"; break;
        default: ladoStr = "DESCONHECIDO"; break;
    }
    
    // Calcular timestamps
    // std::int64_t inicioMs = timePointToUnix(inicio_);
    // std::int64_t fimMs;
    std::string inicioStr = formatoBrasileiro(inicio_);
    std::string fimStr = formatoBrasileiro(fim_);
    
    // if (closed_) {
    //     fimMs = timePointToUnix(fim_);
    // } else {
    //     // Calcular fim estimado
    //     auto agora = std::chrono::steady_clock::now();
    //     auto duracao = std::chrono::duration_cast<std::chrono::milliseconds>(agora - start_);
    //     auto fim_estimado = inicio_ + std::chrono::duration_cast<std::chrono::system_clock::duration>(duracao);
    //     fimMs = timePointToUnix(fim_estimado);
    // }
    
    // Converter ângulo para string manualmente (evita <iomanip>)
    std::string anguloStr = std::to_string(angulo_);
    // Limitar a 2 casas decimais manualmente
    size_t pos = anguloStr.find('.');
    if (pos != std::string::npos && pos + 3 < anguloStr.length()) {
        anguloStr = anguloStr.substr(0, pos + 3);
    }
    
    // Construir JSON usando concatenação simples
    std::string json = "{";
    json += "\"inicio\":\"" + inicioStr + "\",";
    json += "\"fim\":\"" + fimStr + "\",";
    json += "\"perna\":\"" + std::string(ladoStr) + "\",";
    json += "\"movimento\":\"" + std::string(movimentoStr) + "\",";
    json += "\"angulo_maximo\":" + anguloStr;
    json += "}";
    
    return json;
}

