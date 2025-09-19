// ======================================================================
//  Arquivo: evento.h
//  Descrição: Definição da classe Evento para controle de eventos de postura
// ======================================================================

#ifndef EVENTO_INC_EVENTO_H_
#define EVENTO_INC_EVENTO_H_

#include <string>      // Para manipulação de strings (ex: buildJson)
#include <chrono>      // Para controle preciso de tempo dos eventos
#include <functional>  // (Reservado para possíveis callbacks ou extensões)
#include "analise_postural.h" // Tipos auxiliares: TipoMovimento, LadoCorpo

// ----------------------------------------------------------------------
// Classe Evento
// ----------------------------------------------------------------------
/**
 * @brief Representa um evento de postura perigosa detectado pelo sistema.
 *
 * Cada instância armazena informações sobre o tipo de movimento, lado do corpo,
 * ângulo máximo atingido, e os instantes de início e fim do evento.
 * Utiliza relógios de alta precisão para medir a duração real, independente de alterações no sistema.
 */
class Evento
{
public:
   //-------------------------------------------------------------------
   // Construtor
   //-------------------------------------------------------------------

   /**
    * @brief Cria um novo evento de postura perigosa.
    * @param movimento Tipo de movimento detectado (ex: FLEXAO, ABDUCAO)
    * @param lado Lado do corpo onde o evento ocorreu (DIREITO/ESQUERDO)
    * @param anguloInicial Valor inicial do ângulo detectado (em graus)
    */
   Evento(TipoMovimento movimento, LadoCorpo lado, float anguloInicial = 0.f);

   //-------------------------------------------------------------------
   // Métodos principais de controle do evento
   //-------------------------------------------------------------------

   /**
    * @brief Encerra o evento, registrando o instante de término.
    * Marca o evento como fechado e armazena o tempo de fim.
    */
   void closeEvent(void);

   /**
    * @brief Retorna a duração total do evento em milissegundos.
    * Calcula a diferença entre início e fim usando relógio monotônico.
    * @return Duração do evento em ms
    */
   std::int64_t getDuracaoMS(void) const;

   /**
    * @brief Gera uma representação JSON do evento (para logs ou exportação).
    * @return String JSON com os dados principais do evento
    */
   std::string buildJson(void) const;

   //-------------------------------------------------------------------
   // Métodos de acesso (getters e setters)
   //-------------------------------------------------------------------

   /**
    * @brief Atualiza o ângulo máximo do evento, se o novo valor for maior.
    * @param a Novo valor de ângulo (em graus)
    */
   void setAngulo(float a);

   /** @brief Retorna o maior ângulo registrado durante o evento. */
   float getMaxAngulo(void) const { return angulo_; }

   /** @brief Retorna o lado do corpo associado ao evento. */
   LadoCorpo getLado(void) const { return lado_; }

   /** @brief Retorna o tipo de movimento perigoso detectado. */
   TipoMovimento getPerigo(void) const { return movimento_; }

   /** @brief Retorna o instante de início do evento (relógio do sistema). */
   std::chrono::system_clock::time_point getInicio(void) const { return inicio_; }

   /** @brief Retorna o instante de fim do evento (relógio do sistema). */
   std::chrono::system_clock::time_point getFim(void) const { return fim_; }

private:
   //-------------------------------------------------------------------
   // Variáveis internas para dados do evento
   //-------------------------------------------------------------------

   bool closed_ = false; ///< Indica se o evento já foi encerrado
   TipoMovimento movimento_; ///< Tipo de movimento perigoso detectado
   LadoCorpo lado_;          ///< Lado do corpo onde o evento ocorreu
   float angulo_;            ///< Maior ângulo registrado durante o evento (graus)

   // Instantes de início e fim (relógio do sistema, para logs e exportação)
   std::chrono::system_clock::time_point inicio_; ///< Início do evento (data/hora)
   std::chrono::system_clock::time_point fim_;    ///< Fim do evento (data/hora)

   // Instantes de início e fim (relógio monotônico, para cálculo preciso de duração)
   std::chrono::steady_clock::time_point start_; ///< Início da contagem (imune a alterações do relógio do sistema)
   std::chrono::steady_clock::time_point end_;   ///< Fim da contagem

   // (Funções internas auxiliares podem ser declaradas aqui, se necessário)
};

#endif /* EVENTO_INC_EVENTO_H_ */
