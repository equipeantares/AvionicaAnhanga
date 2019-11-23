#ifndef voo_h
#define voo_h
#include "dados.h"
class Voo {
  public:
    Dados altura;
    Dados velocidade;
    Dados aceleracao;
    int tempo = 2;
    
    void addAltura(float dado){
      if (altura.isFirst() != 0) {
        altura.adicionarValor(dado);
        float v = (altura.getValor(0) - altura.getValor(-1)) / tempo;
        if (velocidade.isFirst() != 0){
          velocidade.adicionarValor(v);
          float a = (velocidade.getValor(0) - velocidade.getValor(-1)) / tempo;
          aceleracao.adicionarValor(a);
        } else {
          velocidade.adicionarValor(v);
        }
      } else {
        altura.adicionarValor(dado);
      }
    }
};

#endif
