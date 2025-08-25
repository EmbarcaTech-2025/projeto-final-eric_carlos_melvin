#include "evento.h"

/******************************************************************
 Função : Evento
 Finalidade : Construtor da classe Evento para inicializar eventos de monitoramento de postura
 Entradas Globais : Nenhuma
 Saídas Globais : Nenhuma
 Entradas : TipoMovimento movimento - Tipo de movimento detectado
           LadoCorpo lado - Lado do corpo onde o movimento foi detectado
           float angulo - Medição do ângulo em graus
           std::string duracao - Duração do evento detectado
 Saídas : Nenhuma
 ******************************************************************/
Evento::Evento(TipoMovimento movimento, LadoCorpo lado, float angulo, std::string duracao)
    : movimento_(movimento), lado_(lado), angulo_(angulo), duracao_(duracao) 
{
    // Implementação do construtor - Inicializa todas as variáveis membro
    // Nenhuma validação adicional necessária pois os parâmetros são validados pelo chamador
}

/******************************************************************
 Função : getAngulo
 Finalidade : Esta função retorna a medição do ângulo registrado
 Entradas Globais : Nenhuma
 Saídas Globais : Nenhuma
 Entradas : Nenhuma
 Saídas : float: Medição do ângulo em graus
 ******************************************************************/
float Evento::getAngulo(void) const 
{
    return angulo_;
}

/******************************************************************
 Função : getDuracao
 Finalidade : Esta função retorna a duração do evento registrado
 Entradas Globais : Nenhuma
 Saídas Globais : Nenhuma
 Entradas : Nenhuma
 Saídas : std::string: Duração do evento como string
 ******************************************************************/
std::string Evento::getDuracao(void) const 
{
    return duracao_;
}

/******************************************************************
 Função : getDescricao
 Finalidade : Esta função gera uma descrição legível do evento incluindo
          tipo de movimento, lado do corpo e informações do ângulo
 Entradas Globais : Nenhuma
 Saídas Globais : Nenhuma
 Entradas : Nenhuma
 Saídas : std::string: Descrição completa do evento de postura
 ******************************************************************/
std::string Evento::getDescricao(void) const 
{
    std::string desc = "";
    
    // Constrói a descrição do tipo de movimento
    if (movimento_ == TipoMovimento::FLEXAO) {
        desc += "Flexão excessiva ";
    } else {
        desc += "Posição normal ";
    }
    
    // Adiciona informações do lado do corpo
    if (lado_ == LadoCorpo::PERNA_DIR) {
        desc += "perna direita";
    } else {
        desc += "perna esquerda";
    }
    
    // Anexa informações da medição do ângulo
    desc += " (ângulo: " + std::to_string(static_cast<int>(angulo_)) + "°)";
    
    return desc;
}
