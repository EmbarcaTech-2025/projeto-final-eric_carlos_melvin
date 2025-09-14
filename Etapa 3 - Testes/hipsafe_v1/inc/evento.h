#ifndef EVENTO_INC_EVENTO_H_ 
#define EVENTO_INC_EVENTO_H_

#include <string>
#include <chrono>
#include <functional>
#include "analise_postural.h"


/*
 *       Classe Evento
 */
class Evento   
{
public:
   /*
    *       Estrutura de Dados do Evento
    */

   /*Construtor*/
   Evento(TipoMovimento movimento, LadoCorpo lado, float anguloInicial = 0.f);

   /*Outras Funções do Evento*/
   void closeEvent(void);       // Fecha o evento
   std::int64_t getDuracaoMS(void) const;  // verifica o tempo que está levando o evento
   std::string buildJson(void) const;

   /*Gets and Sets*/
   void setAngulo(float a);    // Salva o angulo se ele for maior que o registrado no evento.
   float getMaxAngulo(void) const { return angulo_; };
   LadoCorpo getLado(void) const { return lado_; };
   TipoMovimento getPerigo(void) const { return movimento_; };
   std::chrono::system_clock::time_point getInicio(void) const { return inicio_; };
   std::chrono::system_clock::time_point getFim(void) const { return fim_; };

private:
   /*
    *       Variáveis internas para dados do evento
    */
   bool closed_ = false;
   TipoMovimento movimento_;    // Tipo de movimento detectado
   LadoCorpo lado_;            // Lado do corpo onde o movimento ocorreu
   float angulo_;              // Medição do ângulo em graus
   std::chrono::system_clock::time_point inicio_;     // Início do evento em date/time
   std::chrono::system_clock::time_point fim_;        // fim do evento em date/time  
   std::chrono::steady_clock::time_point start_;      // Início da contagem (mudar o relógio não interfere)
   std::chrono::steady_clock::time_point end_;        // fim da contagem (sem interferẽncia da hora do relógio)  

   /*Funções internas*/

}; /*fim da Classe*/

#endif /* EVENTO_INC_EVENTO_H_ */
