#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
//Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void SalvarParaArquivo(float h){
  dataFile = SD.open("arquivo.txt", FILE_WRITE);
  dataFile.print("Tempo: ");
  dataFile.print(millis());
  dataFile.print("\t\t|\t");
  dataFile.print("Altura: ");
  dataFile.print(h);
  dataFile.println(" metros");
  dataFile.close();
}

void loop() {
  float Pressao = bmp.readPressure();
  float Altura = bmp.readAltitude(1013.25);
  //SalvarParaArquivo(Pressao);
  //SalvarParaArquivo(Altura);
  
  Serial.print(F("Pressão = "));
  Serial.print(Pressao);
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(Altura); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println();
  delay(500);
}
