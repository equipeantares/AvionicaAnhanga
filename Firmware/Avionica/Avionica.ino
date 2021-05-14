/***************************************************************************
  ------------- Embedded firmware for the Anhangá Rocket -----------------
  Authors: Batata, Joan, Gabriel
  Version: 0.3 - 26/11/2019 [Teste paraquedas CEPETRO]

  Description:
    This firmware implements a state machine, keeping track of the rocket
    flight phases and acting accordingly. 
    There are 5 main flight states:
      E1 - Ground before launch
      E2 - Propulsive flight
      E3 - Ballistic flight
      E4 - Descent with parachute
      E5 - Ground after flight
    Between each of these states, there are main transitions and actions 
    on the rocket, the parachute launch being the most important one.
    
    The rocket phases and state transitions, as well as the fluxogram of this
    code are presented in detail in the firmware development report, located 
    in Antares Google drive repository.
  
 ***************************************************************************/

/* ----- Library Inclusion ----- */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>
#include "voo.h"


/* ----- Macro definitions ----- */

#define IGN       (2)       // I/O pin for ignitor digital output
#define BUZZ      (8)       // I/O pin for buzzer digital output
#define IGN_READ  (A0)      // I/O pin for analogic ignitor read
#define ADD_BMP1  0x76      // Address of BMP no. 1
#define ADD_BMP2  0x77      // Address of BMP no. 2
#define CS_PIN    9         // Chip Select pin for SPI communication with SD card
#define TIMER_TKS 200       // Timer ticks in a period (Timer 1 runs at 2MHz freq.) - [200 for period of 0.1ms]
#define T_MEASURE 56        // Measure period (x 0.1ms) - [56 for 181.8 Hz]
#define LED_TEST  (13)      // Digital output pin for testing


/* ----- Type definitions ----- */

//typedef long long signed int BMP280_S64_t;
//typedef unsigned long int BMP280_U32_t;
//typedef signed long int BMP280_S32_t;


/* ----- Global variables definitions ----- */

const int chipSelect = CS_PIN;            // CS for SPI communication with SD card
volatile unsigned long timeCount = 0UL;   // A long is 32 bits long = 4294967296 levels
                                          // Using a dt=100us, it can count up to 119 hours
volatile int measureCount = 0;            // Counter for dividing timer freq. to get measure freq.
volatile byte state = 0;                  // State of the flight (E1 corresponds to 0, and so forth)
volatile bool led_on = false;             // Variable for test LED state

Adafruit_BMP280 bmp1, bmp2;               // Object used as BMP280 driver - I2C use
Voo voo1 = Voo(50);                       // Object containing height, velocity, acceleration - sensor 1
Voo voo2 = Voo(50);                       // Object containing height, velocity, acceleration - sensor 2
volatile unsigned long timeSave1 = 0UL;   // Variable used for storing moment of last measure of sensor 1
volatile unsigned long timeSave2 = 0UL;   // Variable used for storing moment of last measure of sensor 2
volatile float bmpPi1, bmpPi2;            // Initial pressures on sensors 1 and 2
volatile float bmpHi1, bmpHi2;            // Initial height measured on sensors 1 and 2
volatile float bmpD1, bmpD2;              // Derivative of height w.r.t. pressure on initial pressure 



/* ----- Auxiliary function definitions ----- */

/* Timer 1 Interruption Sub-Routine */
ISR(TIMER1_COMPA_vect){
  timeCount++;                    // Increments time counter
  measureCount++;                 // Increments measure counter
  if(measureCount == T_MEASURE)   // 5600us makes a period of 181.8Hz
    measureCount = 0;             // Reset measure counter for main program
}

/* Save data to SD function */
void save_to_SD(String bmp, Voo dados){
  File dataFile;                                        // Object used as SD driver - SPI use
  Serial.println(F("Dados de altura internos:"));
  int vsize = 4;
  float vect [vsize] = {timeCount/10000.0,              // Time since start-up in seconds
                        dados.altura.getValor(-1),      // Last acquired height measure [m]
                        dados.velocidade.getValor(-1),  // Last acquired velocity measure [m/s]
                        dados.aceleracao.getValor(-1)}; // Last acquired acceleration measure [m/s2]                     
  dataFile = SD.open("antares.txt", FILE_WRITE);
  if(!dataFile){
    Serial.println(F("---------> ERRO na abertura do arquivo"));
  }
  dataFile.print(bmp);
  Serial.print(bmp);
  for(int i=0; i<vsize; i++){
    dataFile.print("\t");
    dataFile.print(vect[i]);
    Serial.print("\t");
    Serial.print(vect[i],5);
  }
  dataFile.print("\n");
  Serial.print("\n");
  dataFile.close();
  return;
}

/* BMP height configuration - call on setup */
bool configure_BMP(byte bmp_nb){
  if(bmp_nb <=0 || bmp_nb >=3){
    Serial.println(F("ERROR: configure_BMP -> invalid BMP number!"));
    return(false);
  }
  float bmpPi = 0;
  float bmpHi = 0;
  float bmpDi = 0;
  int address = (bmp_nb == 1) ? ADD_BMP1 : ADD_BMP2;            // Gets bmp adress accordingly
  Adafruit_BMP280 * bmpHandle = (bmp_nb == 1) ? &bmp1 : &bmp2;  // Gets bmp object handle accordingly
  if (!bmpHandle->begin(address)) {                             // Initialize BMP
    Serial.print(F("Could not find BMP280 sensor (add="));
    Serial.print(address);
    Serial.println(F("), check wiring and reset!"));
    return(false);
  }
  bmpHandle->setSampling(Adafruit_BMP280::MODE_NORMAL,         /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,               /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,              /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,                /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500);           /* Standby time. */
  for(int i=0; i<100; i++){
    bmpPi += bmpHandle->readPressure();                        // Read initial pressure
    bmpHi += bmpHandle->readAltitude();                        // Read initial altitude (exponential method)
  }
  bmpPi = bmpPi/100.0;                                        // Get average of pressure in 100 measures
  bmpHi = bmpHi/100.0;                                        // Get average of height in 100 measures
  bmpDi = -2260.209*pow(bmpPi,-0.8097);                       // Calculate derivative in initial pressure
  if (bmp_nb == 1){                                           // Save data in according variables
    bmpPi1 = bmpPi;
    bmpHi1 = bmpHi;
    bmpD1 = bmpDi; 
  }
  else{
    bmpPi2 = bmpPi;
    bmpHi2 = bmpHi;
    bmpD2 = bmpDi;
  }
  Serial.print(F("Finished configuration of BMP "));
  Serial.println(bmp_nb);
  Serial.print(F("bmpPi = "));
  Serial.println(bmpPi,3);
  Serial.print(F("bmpHi = "));
  Serial.println(bmpHi,3);
  Serial.print(F("bmpDi = "));
  Serial.println(bmpDi,3);
  return(true);
}

/* BMP height reading - Linear method */
float read_BMP_h(byte bmp_nb){
  if(bmp_nb <=0 || bmp_nb >=3){
    Serial.println(F("ERROR: read_BMP_h -> invalid BMP number!"));
    return(0);
  }
  float p,h;
  Adafruit_BMP280 * bmpHandle = (bmp_nb == 1) ? &bmp1 : &bmp2;     // Gets bmp object handle accordingly
  float bmpPi = (bmp_nb == 1) ? bmpPi1 : bmpPi2;
  float bmpHi = (bmp_nb == 1) ? bmpHi1 : bmpHi2;
  float bmpDi = (bmp_nb == 1) ? bmpD1 : bmpD2;
  p = bmpHandle->readPressure();
//  Serial.println(F("Pressure reading: "));
//  Serial.println(p,5);
  h = bmpHi + bmpDi * (p - bmpPi);                           // Linearization of height
  return(h);
}

/* BMP average valid reading - call on loop */
float read_avg_BMP(byte bmp_nb){
  if(bmp_nb <=0 || bmp_nb >=3){
    Serial.println(F("ERROR: read_avg_BMP -> invalid BMP number!"));
    return(-1.0);
  }
  float read_h = 0.0;
  float acc_h = 0.0;
  byte measuresDone = 0;                    // Counter for valid height acquisitions from sensors
  byte failed = 0;
  while(measuresDone < 3 && failed < 3){     
    if(measureCount == 0){                  // Waits cycle of measurement (acquisition frequency)
      read_h = read_BMP_h(bmp_nb);
//      Serial.println(F("Dados de altura recebidos interior:"));
//      Serial.println(read_h, 5);
      if(read_h < 1700 && read_h > 0){      // Measure validation - h in launch site is 700m + max of flight is 500m = total of 1200m
        acc_h += read_h;                
        measuresDone++;
      }
      else
        failed++;
    }
  }
  if(failed >= 3)
    return(0.0);
  else
    return(acc_h/3.0);
}

unsigned int get_dt(byte bmp_nb){
  if(bmp_nb <=0 || bmp_nb >=3){
    Serial.println(F("ERROR: get_dt -> invalid BMP number!"));
    return(0);
  }
  unsigned long timeSave = (bmp_nb == 1) ? timeSave1 : timeSave2;
  unsigned int dt;
  noInterrupts();
  dt = (unsigned int)(timeCount - timeSave);
  if(bmp_nb == 1)
    timeSave1 = timeCount;
  else
    timeSave2 = timeCount;
  interrupts();
  return(dt);
}

/* LED configuration - call on setup */
void LED_config(){ 
  pinMode(LED_TEST, OUTPUT);
  digitalWrite(LED_TEST, LOW);
}

/* LED test routine - for testing: call on loop */
void LED_toggle(){
  if(!led_on){
      digitalWrite(LED_TEST, HIGH);
      Serial.println(F("LED ON"));
    }
    if(led_on){
      digitalWrite(LED_TEST, LOW);
      Serial.println(F("LED OFF"));
    }
    led_on = !led_on;
    Serial.print(F("timeCount = "));
    Serial.println(timeCount);
    Serial.print(F("measureCount = "));
    Serial.println(measureCount);
}


/* ----- Setup function definition ----- */

void setup() {
  /* Serial communication Configuration */
  Serial.begin(9600);
  Serial.println(F("Anhanga Firmware Debugging..."));

  /* I/O Configuration */
  pinMode(IGN, OUTPUT);     // Configure ignitor pin as digital output
  pinMode(BUZZ, OUTPUT);    // Configure buzzer pin as digital output
  pinMode(CS_PIN, OUTPUT);  // Configure Chip Select pin as digital output
//  LED_config();             // Configure LED for testing code
  Serial.println(F("I/O configuration done."));

  /* SD Configuration */
  if(!SD.begin(chipSelect)){ 
    Serial.println("Error in SD card configuration!"); 
//    return;
  }
  else
    Serial.println(F("SD configuration done."));

  /* BMP280 Configuration */
  if(configure_BMP(1) && configure_BMP(2))      // Configuration and initialization funcions of BMP1 and BMP2
    Serial.println(F("BMPs configuration done."));
  else
    Serial.println(F("Problems found on BMPs configuration."));

  /* Timer 1 (16-bit) configuration */
  TCCR1A = 0x00;        // Configure timer on internal source and CTC (Clear Timer on Compare) match mode.
  TCCR1B = B00001010;   // Configure internal clock with prescaler = 8 --> Timer frequency = 2MHz. 
                        // Precise CTC mode - clear on OCR1A register match.
  OCR1A = TIMER_TKS;    // Amount of timer ticks to generate period of 100us (10kHz).
  Serial.println(F("Timer 1 registers set."));
  
  /* Timer 1 Interruption configuration */
  TIMSK1 |= 0x02;       // Enable Interrupt on OCR1A Match
  interrupts();         // Enable interruptions

  Serial.println(F("All configurations finished, interruptions enabled"));
}


/* ----- Main function definition ----- */

void loop() {

//    Serial.println("bmp1:");
//    Serial.print(F("Approx altitude = "));
//    float h1 = bmp.readAltitude(1013.25);
//    Serial.print(h1);
//    Serial.println(" m");
//    salva("bmp1: ", h1);
//
//    Serial.println();
//    Serial.println("bmp2:");
//    Serial.print(F("Approx altitude = "));
//    float h2 = bmp2.readAltitude(1013.25);
//    Serial.print(h2);
//    Serial.println(" m");
//    salva("bmp2: ", h2);
//
//    Serial.println();
//    delay(2000);
//  LED_toggle();
  float h1 = read_avg_BMP(1);
  voo1.tempo = get_dt(1);
  voo1.addAltura(h1);
  float h2 = read_avg_BMP(2);
  voo2.tempo = get_dt(2);
  voo2.addAltura(h2);
  save_to_SD("bmp1",voo1);
  save_to_SD("bmp2",voo2);
  
  switch(state){
    case 0:         // E1 = Initialization
      // Check if flying
      break;
    case 1:         // E2 = Propulsive Flight
      // Check if end of propulsion
      break;
    case 2:         // E3 = Ballistic Flight
      // Check if target velocity reached
        digitalWrite(IGN, HIGH);
      break;
    case 3:         // E4 = Parachute Descent
      // Check if reached ground
      // Check igniter
      break;
    case 4:         // E5 = Ground reached
      digitalWrite(BUZZ, HIGH);
      break;
    case 10:        // E11 = Failure mode
      // What to do?
      break;
  }
}
