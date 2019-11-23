#ifndef dados_h
#define dados_h

class Dados{
  public:
    float a[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int index = 0;
    int first = 0;
    
    int isFirst(){
      return first;
    }
    
    void adicionarValor(float dado){
      a[index] = dado;
      index = atualizaIndex(index);
      first = 1;
    }
    
    int atualizaIndex(int i){
      if (i < 9){
        i++;
      } else {
        i = 0;
      }
      return i;
    }
    
    float getMedia(){
      int i = 0;
      float soma = 0;
      for (i = 0; i < 10; i++){
        soma += a[i];
      }
      
      return soma / 10;
    }
    
    float getValor(int deslocamento){
      int i = index;
      if (deslocamento == 1) {
        if (i != 9){
          i += deslocamento;
        } else {
          i = 0;
        }
      } else if (deslocamento == -1) {
        if (i != 0){
          i += deslocamento;
        } else {
          i = 9;
        }
      }
      
      return a[i];
    }
};

#endif
