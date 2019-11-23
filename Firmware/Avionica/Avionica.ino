/*
#include <Wire.h>
#include <SPI.h>
//#include <Adafruit_BMP280.h>
#include <SD.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp, bmp2; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

const int chipSelect = 10;
SDFile dataFile;

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  if (!bmp2.begin(0x77)) {
    Serial.println(F("Could not find a valid BMP2802 sensor, check wiring!"));
    while (1);
  }

  if(!SD.begin(chipSelect)){ //SE O CARTÃO DE MEMÓRIA NÃO ESTIVER PRESENTE OU FALHAR, FAZ
    Serial.println("Cartão de memória falhou ou não está presente!"); //IMPRIME O TEXTO NO MONITOR SERIAL
    return; //NÃO FAZ MAIS NADA
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
                  
  bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void salva(String bmp, float h){
  dataFile = SD.open("teste.txt", FILE_WRITE);
  dataFile.print(bmp);
  dataFile.print("Altura: ");
  dataFile.print(h);
  dataFile.println(" metros");
  dataFile.close();
}

void loop() {
    Serial.println("bmp1:");
    Serial.print(F("Approx altitude = "));
    float h1 = bmp.readAltitude(1013.25);
    Serial.print(h1);
    Serial.println(" m");
    salva("bmp1: ", h1);

    Serial.println();
    Serial.println("bmp2:");
    Serial.print(F("Approx altitude = "));
    float h2 = bmp2.readAltitude(1013.25);
    Serial.print(h2);
    Serial.println(" m");
    salva("bmp2: ", h2);

    Serial.println();
    delay(2000);
}
*/

#include "voo.h"
int i = 1;
Voo voo;
void setup(){
  
}

void loop(){
  voo.addAltura(i);
  i++;
  Serial.print(voo.altura.getValor(0));
  Serial.print(voo.velocidade.getValor(0));
  Serial.print(voo.aceleracao.getValor(0));
  delay(100);
}

