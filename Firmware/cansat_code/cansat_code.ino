#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <SPI.h> //INCLUSÃO DE BIBLIOTECA
#include <SD.h> //INCLUSÃO DE BIBLIOTECA
//#include "song.h"

const int chipSelect = 9;

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp;

double minimumHeight = 0, cont = 0;
byte estado_apogee = 0, pinBuzzer = 4, pinMosfet = 7, estado_ground = 0, vccBuz = 3, vccBMP = 6;
float offSetHeight = 0, hFly = 0, forecast = 1022;
boolean isF = false, isAR = false;
SDFile dataFile;

void setup() {
  Serial.begin(9600);
  pinMode(12, OUTPUT);
  pinMode(vccBMP, OUTPUT);
  digitalWrite(vccBMP, HIGH);
  
  pinMode(SS, OUTPUT); //DEFINE O PINO COMO SAÍDA
  pinMode(pinMosfet, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(vccBuz, OUTPUT);
  digitalWrite(vccBuz, HIGH);

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  if(!SD.begin(chipSelect)){
    Serial.println("Cartão de memória falhou ou não está presente!");
    return;
  }

  offSetHeight = setOffSetHeight();

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
}

void saveToFile(float h){
  dataFile = SD.open("arquivo.txt", FILE_WRITE);
  dataFile.print("Tempo: ");
  dataFile.print(millis());
  dataFile.print("\t\t|\t");
  dataFile.print("Altura: ");
  dataFile.print(h);
  dataFile.println(" metros");
  dataFile.close();
}

float setOffSetHeight(){
  int j = 0;
  float sum = 0;
  for(j = 0; j < 100; j+=1){
    sum += bmp.readAltitude(forecast);
  }
  return sum/(j+1);
}

float readV(){
  float media = 0;
  byte m = 0;
  for(m = 0; m < 3; m++){
    float v = bmp.readAltitude(forecast);
    v = filter(v);
    saveToFile(v);
    media += v/3;
  }
  return media;
}

boolean isFlying(){
  hFly = readV();
  saveToFile(hFly);
  if((hFly - offSetHeight) > 3){
    return true;
  } else {
    return false;
  }
}

float filter(float v){
  float filteredData = 0;
  v = abs(v);
  if(v > 1000){
    filteredData = filter(bmp.readAltitude(forecast));
  } else {
    filteredData = v;
  }
  return filteredData;
}

boolean detectApogee(){
  float pHeight = 0, nHeight = 0;
  boolean isReached = false;
  pHeight = bmp.readAltitude(forecast);
  saveToFile(pHeight);
  delay(50);
  nHeight = bmp.readAltitude(forecast);
  saveToFile(nHeight);
  if((nHeight - pHeight) < 0.3){
    isReached = true;
    digitalWrite(pinMosfet, HIGH);
    delay(400);
    digitalWrite(pinMosfet, LOW);
  }
  return isReached;
}

boolean detectGround(){
  boolean ground = false;
  float pHeight = 0, nHeight = 0;
  pHeight = readV();
  saveToFile(pHeight);
  delay(150);
  nHeight = readV();
  saveToFile(nHeight);
  float dif = nHeight - pHeight;
  if(dif >= -0.03 && dif <= 0.03){
    ground = true;
  }
  return ground;
}

void loop() {
  if(!isF){
    while(!isFlying());
    isF = true;
  } else {
    if(!isAR){
      while(!detectApogee());
      isAR = true;
    } else {
      while(!detectGround());
    }
  }
}
