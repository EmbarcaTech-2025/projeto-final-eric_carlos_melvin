// ======================================================================
//  Arquivo: analise_postural.h
//  Descrição: Interface para análise postural, detecção de risco e controle de alarme
// ======================================================================

#ifndef ANALISE_POSTURAL_H_
#define ANALISE_POSTURAL_H_

#include <array>                    // Para uso de std::array em constantes
#include <iostream>                 // Para logs e depuração
#include "estruturas_de_dados.hpp" // Tipos e estruturas auxiliares
#include "mpu9250_i2c.h"           // Interface do sensor MPU9250

// ----------------------------------------------------------------------
// Constantes de Limite para Movimentos Articulares
// ----------------------------------------------------------------------
/**
 * @brief Limites máximos permitidos para cada tipo de movimento articular (em graus).
 *        Utilizado para detecção de situações de risco postural.
 *        Ordem: {flexão, abdução, rotação}
 */
constexpr std::array<int,3> LIMITACOES = {90, 60, 45};

// ----------------------------------------------------------------------
// Protótipos das Funções Principais de Análise Postural
// ----------------------------------------------------------------------

/**
 * @brief Realiza a leitura dos sensores, processa os dados e retorna os ângulos articulares.
 * @param mpu_list Array de 2 sensores MPU9250 (tronco e coxa)
 * @return Estrutura Orientacao com ângulos de flexão, abdução e rotação
 */
Orientacao getPosition(mpu9250_t mpu_list[2]);

/**
 * @brief Analisa a orientação atual, gerencia eventos e alarmes de postura perigosa.
 * @param orientacao Estrutura com ângulos de flexão, abdução e rotação
 */
void dangerCheck(Orientacao orientacao);

// ----------------------------------------------------------------------
// Funções de Controle Manual do Alarme Sonoro
// ----------------------------------------------------------------------

/** @brief Silencia manualmente o alarme sonoro (buzzer). */
void silenciar_alarme(void);

/** @brief Desfaz o silenciamento do alarme, religando o buzzer se necessário. */
void desilenciar_alarme(void);

/** @brief Consulta se o alarme está atualmente ligado (ativo). */
bool alarme_esta_ligado(void);

/** @brief Consulta se o alarme está silenciado manualmente. */
bool alarme_esta_silenciado(void);

#endif // ANALISE_POSTURAL_H_