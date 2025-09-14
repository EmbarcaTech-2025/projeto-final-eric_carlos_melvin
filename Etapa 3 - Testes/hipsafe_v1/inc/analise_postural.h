#ifndef ANALISE_POSTURAL_H_
#define ANALISE_POSTURAL_H_

#include <array>
#include <iostream>
#include "estruturas_de_dados.hpp"
#include "mpu9250_i2c.h"
//#include "bogus.h"

// Definições gerais 
constexpr std::array<int,3> LIMITACOES = {90, 10, 15};  // {flexão, abdução, rotação}

// #define  INIT_VAL_ZERO                       (0U)           
// #define  INIT_VAL_ONE                        (1U)           
// #define  MAX_DESCRICAO_LENGTH                (100U)         
// #define  MAX_DURACAO_LENGTH                  (50U)   


// Funções
static const char* ladoToStr(LadoCorpo l);
static const char* movToStr(TipoMovimento m);
Orientacao getPosition(mpu9250_t mpu_list[3]);
void dangerCheck(Orientacao orientacao);

#endif