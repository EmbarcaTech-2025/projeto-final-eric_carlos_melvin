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

// RELATIVO A FUNÇÕES PRINCIPAIS:
Orientacao getPosition(mpu6050_t mpu_list[3]) {
    Orientacao orientacao;
    orientacao.flexao = 75.3;
    orientacao.rotacao = 10.1;
    orientacao.abducao = 10.0;
    return orientacao;
};

void dangerCheck(Orientacao orientacao) {
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