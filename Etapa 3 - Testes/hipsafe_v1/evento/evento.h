#ifndef EVENTO_INC_EVENTO_H_ // polyspace MISRA-CPP:16-2-1 :No Functional side effect
#define EVENTO_INC_EVENTO_H_ // polyspace MISRA-CPP:16-2-1 :No Functional side effect

#include <string>

/* Definições */

/* Definições de tipo de movimento para monitoramento de postura */
#define  MOVIMENTO_FLEXAO                    (0U)           
#define  MOVIMENTO_NORMAL                    (1U)           

/* Definições de lado do corpo para rastreamento de localização */
#define  LADO_PERNA_DIREITA                  (0U)           
#define  LADO_PERNA_ESQUERDA                 (1U)           

/* Definições gerais */
#define  INIT_VAL_ZERO                       (0U)           
#define  INIT_VAL_ONE                        (1U)           
#define  MAX_DESCRICAO_LENGTH                (100U)         
#define  MAX_DURACAO_LENGTH                  (50U)          
// Definições de tipo de movimento para monitoramento de postura
enum class TipoMovimento
{
    FLEXAO = MOVIMENTO_FLEXAO,    // Flexão excessiva detectada
    NORMAL = MOVIMENTO_NORMAL     // Posição normal
};

// Definições de lado do corpo para rastreamento de localização
enum class LadoCorpo
{
    PERNA_DIR = LADO_PERNA_DIREITA,  // Perna direita
    PERNA_ESQ = LADO_PERNA_ESQUERDA  // Perna esquerda
};

/* Fim das Definições */


/*
 *       Classe Evento
 */
class Evento   
{
public:
   /*
    *       Estrutura de Dados do Evento
    */
   typedef struct
   {
      TipoMovimento movimento;    // Tipo de movimento detectado
      LadoCorpo lado;            // Lado do corpo onde o movimento ocorreu  
      float angulo;              // Medição do ângulo em graus
      std::string duracao;       // Duração do evento
   }st_EventoData;

   /*Construtor*/
   Evento(TipoMovimento movimento, LadoCorpo lado, float angulo, std::string duracao);

   std::string getDescricao(void) const;
   float getAngulo(void) const;
   std::string getDuracao(void) const;

private:
   /*
    *       Variáveis internas para dados do evento
    */
   TipoMovimento movimento_;    // Tipo de movimento detectado
   LadoCorpo lado_;            // Lado do corpo onde o movimento ocorreu
   float angulo_;              // Medição do ângulo em graus
   std::string duracao_;       // Duração do evento

}; /*fim da Classe*/

#endif /* EVENTO_INC_EVENTO_H_ */
