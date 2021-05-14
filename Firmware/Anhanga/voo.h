#ifndef voo_h
#define voo_h
#include "dados.h"
class Voo {
  public:
    Dados altura;         // [m]
    Dados alturaF;        // Altura filtrada [m]
    Dados velocidade;     // [m/s]
    Dados velocidadeF;    // Velocidade filtrada [m/s]
    float dt;             // Intervalo de tempo entre medidas [s]
    float tau;            // Constante de tempo do filtro de primeira ordem
    float pressao;        // Armazena ultimo valor de pressao

    
    Voo(float dTime, float fTau){
      dt = dTime;
      tau = fTau;
    }
    
    void addAltura(float dado){
      if (!altura.isFirst()) {
        altura.adicionarValor(dado);
        // Filtro 1a ordem
        float fh = (tau*alturaF.getValor(0) + dt*altura.getValor(0)) / (dt+tau);
        alturaF.adicionarValor(fh);
        // Derivada Euler explicito
        float v = (alturaF.getValor(0) - alturaF.getValor(-1)) / dt;
        velocidade.adicionarValor(v);
        // Media deslizante
        velocidadeF.adicionarValor(velocidade.getMedia());
      } else {
        altura.adicionarValor(dado);
        alturaF.adicionarValor(dado);
      }
    }
    
};

#endif
