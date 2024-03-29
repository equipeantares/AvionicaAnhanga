#ifndef voo_h
#define voo_h
#include "dados.h"
class Voo {
  public:
    Dados altura;
    Dados velocidade;
    Dados aceleracao;
    int tempo;
    
    Voo(int t){
      tempo = t;
    }
    
    void addAltura(float dado){
      if (altura.isFirst() != 0) {
        altura.adicionarValor(dado);
        float v = (altura.getValor(-1) - altura.getValor(-2)) / tempo;
        if (velocidade.isFirst() != 0){
          velocidade.adicionarValor(v);
          float a = (velocidade.getValor(-1) - velocidade.getValor(-2)) / tempo;
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
