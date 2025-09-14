#include <iostream>
#include <cassert>
#include "analise_postural.h"
#include "evento.h"
#include <thread>

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

int main() {
    // Arrange: cria um evento de flexão na perna direita com ângulo inicial 0
    Evento e(TipoMovimento::FLEXAO, LadoCorpo::DIREITO, 0.0f);

    // Act: simula leituras crescendo
    e.setAngulo(12.3f);
    e.setAngulo(9.8f);   // não deve reduzir o máximo
    e.setAngulo(33.9f);  // novo máximo
    e.setAngulo(15.6f);
    // Fecha o evento (ajuste o nome se for closeEvent/finish/etc.)
    std::this_thread::sleep_for(std::chrono::milliseconds(2542));
    e.closeEvent();

    // Assert: checa o máximo
    // assert(e.getMaxAngulo() == 33.9f);

    // Imprime alguns campos
    std::cout << "Lado: " << ladoToStr(e.getLado()) << "\n";
    std::cout << "Perigo : " << movToStr(e.getPerigo()) << "\n";
    std::cout << "Angulo Max : " << e.getMaxAngulo() << "\n";
    std::cout << "Tempo (ms) : " << e.getDuracaoMS() << "\n";

    // Retorna como será gravado em estilo 'JSON':
    std::cout << "JSON: " << e.buildJson() << "\n";

    std::cout << "OK\n";
    return 0;
}
