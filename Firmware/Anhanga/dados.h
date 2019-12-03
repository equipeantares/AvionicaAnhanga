#ifndef dados_h
#define dados_h

class Dados{
  public:
    float a[6] = {0, 0, 0, 0, 0, 0};
    byte index = 0;
    bool first = true;
    
    bool isFirst(){
      return first;
    }
    
    void adicionarValor(float dado){
      a[index] = dado;
      index = atualizaIndex(index);
      first = false;
    }
    
    int atualizaIndex(byte i){
      if (i < 5){
        i++;
      } else {
        i = 0;
      }
      return i;
    }
    
    float getMedia(){
      int i = 0;
      float soma = 0;
      for (i = 0; i < 6; i++){
        soma += a[i];
      }
      
      return soma / 6;
    }
    
    float getValor(byte deslocamento){    // Recupera valor com deslocamento dado a partir do ultimo valor inserido
      byte i = index + deslocamento - 1;
      if(i > 5 || i < 0){
         i = (i+6)%6;
      }
      return a[i];
    }
};

#endif
