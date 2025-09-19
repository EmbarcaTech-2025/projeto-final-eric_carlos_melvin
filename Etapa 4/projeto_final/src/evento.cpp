#include "evento.h"

/******************************************************************
 Função    : Evento (Construtor)
 Finalidade: Inicializa um novo evento de monitoramento de postura,
             registrando tipo de movimento, lado do corpo e ângulo inicial.
 Entradas  :
   - TipoMovimento movimento: Tipo do movimento (FLEXAO, ABDUCAO, ROTACAO, NORMAL)
   - LadoCorpo lado: Lado do corpo (DIREITO, ESQUERDO)
   - float anguloInicial: Ângulo inicial do evento (em graus)
 Saídas    : Nenhuma
 Observações:
   - Inicializa os marcadores de tempo de início (sistema e steady_clock) e o ângulo máximo.
   - O evento começa aberto (modificável).
 ******************************************************************/
Evento::Evento(TipoMovimento movimento, LadoCorpo lado, float anguloInicial)
    : movimento_(movimento),            // salva as informações passadas 
      lado_(lado),                      //     nas variáveis do objeto
      angulo_(anguloInicial),            
      inicio_(std::chrono::system_clock::now()),  
      start_(std::chrono::steady_clock::now()),
      fim_()
{ 

}


/******************************************************************
 Função : setAngulo
 Finalidade: Atualiza o ângulo máximo do evento, caso o novo valor seja maior.
 Entradas  :
   - float a: Novo valor de ângulo medido (em graus)
 Saídas    : Nenhuma
 Observações:
   - Só atualiza se o evento estiver aberto e o novo ângulo for maior que o atual.
 ******************************************************************/
void Evento::setAngulo(float a)
{
    if(!closed_) 
    {
        if (angulo_ < a) angulo_ = a;
    }    
}

/******************************************************************
 Função : getDuracaoMS
 Finalidade: Retorna a duração do evento em milissegundos.
 Entradas  : Nenhuma
 Saídas    : std::int64_t - Duração do evento em ms
 Observações:
   - Se o evento estiver aberto, calcula a duração até o momento atual.
   - Se fechado, retorna a duração total registrada no fechamento.
 ******************************************************************/
std::int64_t Evento::getDuracaoMS(void) const 
{
    if (!closed_) 
    {
        auto agora = std::chrono::steady_clock::now();
        auto duracao = std::chrono::duration_cast<std::chrono::milliseconds>(agora - start_);
        return duracao.count();
    }
    else 
    {
        // Se chegou aqui é porque já está fechado
        auto duracao = std::chrono::duration_cast<std::chrono::milliseconds>(fim_ - inicio_);
        return duracao.count();  // retorna inteiro em ms
    }
}

/******************************************************************
 Função    : closeEvent
 Finalidade: Fecha o evento, registrando o horário de término e impedindo novas alterações.
 Entradas  : Nenhuma
 Saídas    : Nenhuma
 Observações:
   - Após o fechamento, o evento não pode mais ser modificado.
   - O horário de término é calculado com base na duração desde o início.
 ******************************************************************/
/******************************************************************
 Função    : closeEvent
 Finalidade: Fecha o evento, registrando o horário de término e impedindo novas alterações.
 Entradas  : Nenhuma
 Saídas    : Nenhuma
 Observações:
   - Após o fechamento, o evento não pode mais ser modificado.
   - O horário de término é calculado com base na duração desde o início.
 ******************************************************************/
void Evento::closeEvent(void)
{
    if (closed_) return; // Evita fechar duas vezes
    // Calcula a duração total do evento usando clock monotônico
    auto agora = std::chrono::steady_clock::now();
    auto duracao_total_ = std::chrono::duration_cast<std::chrono::milliseconds>(agora - start_);
    // Ajusta o horário de término no clock do sistema
    fim_ = inicio_ + std::chrono::duration_cast<std::chrono::system_clock::duration>(duracao_total_);
    closed_ = true;
}

/******************************************************************
 Função : buildJson
 Finalidade: Constrói uma string JSON contendo os dados do evento.
 Entradas  : Nenhuma
 Saídas    : std::string - String JSON com informações do evento
 Observações:
   - Datas são formatadas no padrão brasileiro (dd/mm/aaaa hh:mm:ss).
   - Inclui início, fim, lado, movimento e ângulo máximo.
 ******************************************************************/
/******************************************************************
 Função    : buildJson
 Finalidade: Constrói uma string JSON contendo os dados do evento.
 Entradas  : Nenhuma
 Saídas    : std::string - String JSON com informações do evento
 Observações:
   - Datas são formatadas no padrão brasileiro (dd/mm/aaaa hh:mm:ss).
   - Inclui início, fim, lado, movimento e ângulo máximo.
 ******************************************************************/
std::string Evento::buildJson(void) const
{
  // Função lambda para formatar data/hora no padrão brasileiro
  auto formatoBrasileiro = [](const std::chrono::system_clock::time_point& tp) -> std::string {
    auto time_t = std::chrono::system_clock::to_time_t(tp);
    auto tm = *std::localtime(&time_t);
    char buffer[72];
    snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d %02d:%02d:%02d",
         tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900,
         tm.tm_hour, tm.tm_min, tm.tm_sec);
    return std::string(buffer);
  };

  // Conversão do tipo de movimento para string
  const char* movimentoStr;
  switch (movimento_) 
  {
    case TipoMovimento::FLEXAO:   movimentoStr = "FLEXAO"; break;
    case TipoMovimento::ABDUCAO:  movimentoStr = "ABDUCAO"; break;
    case TipoMovimento::ROTACAO:  movimentoStr = "ROTACAO"; break;
    case TipoMovimento::NORMAL:   movimentoStr = "NORMAL"; break;
    default:                      movimentoStr = "DESCONHECIDO"; break;
  }

  // Conversão do lado do corpo para string
  const char* ladoStr;
  switch (lado_) 
  {
    case LadoCorpo::DIREITO:  ladoStr = "DIREITO"; break;
    case LadoCorpo::ESQUERDO: ladoStr = "ESQUERDO"; break;
    default:                  ladoStr = "DESCONHECIDO"; break;
  }

  // Formatação das datas de início e fim
  std::string inicioStr = formatoBrasileiro(inicio_);
  std::string fimStr = formatoBrasileiro(fim_);

  // Conversão do ângulo máximo para string com até 2 casas decimais
  std::string anguloStr = std::to_string(angulo_);
  size_t pos = anguloStr.find('.');
  if (pos != std::string::npos && pos + 3 < anguloStr.length()) 
  {
    anguloStr = anguloStr.substr(0, pos + 3);
  }

  // Montagem do JSON manualmente
  std::string json = "{";
  json += "\"inicio\":\"" + inicioStr + "\",";
  json += "\"fim\":\"" + fimStr + "\",";
  json += "\"perna\":\"" + std::string(ladoStr) + "\",";
  json += "\"movimento\":\"" + std::string(movimentoStr) + "\",";
  json += "\"angulo_maximo\":" + anguloStr;
  json += "}";

  return json;
}

