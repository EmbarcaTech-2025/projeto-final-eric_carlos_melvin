#include "analise_postural.h"

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
    // Melvin ficou de implementar esse algoritmo
    // Resumindo é isso:
    /*
        Para cada evento dentro do vetor eventos (variável global):
            é a mesma perna?
                sim -> tem perigo?
                    sim -> é o mesmo perigo?
                        sim -> retorna true
                não (não tem perigo) -> retorna true (evento aberto, mas já saiu do perigo)
        Se passou por todos os eventos da lista, retorna false
    */
   return false;
}

// RELATIVO A FUNÇÕES PRINCIPAIS:
/***
 * @brief ORGANIZA a coleta dos parâmetros passados pelas MPUs, até a
 * saída de Orientação
 * @param[in] mpu_list[3] Recebe uma lista com o objeto da MPU3590
 * @return Orientacao - ângulos em graus de flexão, rotação e abdução
 * dentro de uma struct Orientacao
 */
Orientacao getPosition(mpu6050_t mpu_list[3]) {
    /*
        Eric ficou de fazer essa parte do algoritmo
        Resumindo ficaria algo assim:
        Utilizando mpu_list[0] e mpu_list[1], desconsidera o mpu_list[2] que seria o da perna esquerda
        (1) Requisita os dados "brutos" dos dois sensores da lista de sensores com readSensor()
        (2) Transforma eles em quaternions com sensorToQuaternion()
        (3) Descobre o quaternion relativo entre eles com relativeQuaternion()
        (4) Descobre os ângulos relativo aos movimento (Orientacao) 
        (5) Retorna esta Orientação   
    */
    Orientacao orientacao;
    orientacao.flexao = 75.3;
    orientacao.rotacao = 10.1;
    orientacao.abducao = 10.0;
    return orientacao;
};

/***
 * @brief ORGANIZA / IMPLEMENTA o algorítmo de checkagem de risco de posição,
 * incluindo o gerenciamento dos alarmes e da gravação dos eventos no SDCard.
 * @param Orientacao - Recebe os ângulos de flexão, rotação e abdução.
 * @return void. 
 */
void dangerCheck(Orientacao orientacao) {
    // O Melvin ficou de implementar esse algoritmo
    // Resumindo os passos:
    /*
        Para cada tipo de movimento (type:TipoMovimento)
            Posição perigosa?
            Se sim: Evento aberto? isEventOpen()
                Se sim: não faz nada
                Se não: Instancia/Cria um objeto Evento;
                        Coloca ele na listagem de eventos (variável global)
                        Liga alarme
            Se não: (posição segura)
                Algum Evento está aberto para essa posição? isEventOpen()
                    Se sim: Encerra o evento
                            Armazena ele no SDcard (função específica para isso)
                            Remove o evento da listagem de eventos
                            Remove ele da memória (??? precisa? evitar memory 
                                            leak - acho que não precisa, pois 
                                            a lista não vai conter ponteiro e
                                            sim o próprio objeto.)
                            Sobrou algum evento na listagem?
                                se sim -> não faz nada
                                Se não -> Desliga o alarme, e desliga o 'mute'

    */
    // Abaixo, tem que apagar... é só 'bogus' (mock)
    if (orientacao.flexao > LIMITACOES[static_cast<int>(TipoMovimento::FLEXAO)]) {
        printf("  flexao > 90 graus \n");
    }
    if (orientacao.rotacao > LIMITACOES[static_cast<int>(TipoMovimento::ROTACAO)]) {
        printf("  rotacao > 15 graus  \n");
    }
    if (orientacao.abducao > LIMITACOES[static_cast<int>(TipoMovimento::ABDUCAO)]) {
        printf("  abducao > 10 graus \n");
    }
};