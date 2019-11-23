#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "voo.h"
/***************************************************************************
  ------------- Embedded firmware for the Anhangá Rocket -----------------
  Authors: Batata, Joan, Gabriel
  Version: 0.1 - 21/11/2019

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
>>>>>>> 228260701e9681ea9654ccb92a12501ee13e2727
#include <SD.h>

Voo voo(50);

/* ----- Macro definitions ----- */

#define IGN       (2)       // I/O pin for ignitor digital output
#define BUZZ      (8)       // I/O pin for buzzer digital output
#define IGN_READ  (A0)      // I/O pin for analogic ignitor read
#define ADD_BMP1  0x76      // Address of BMP no. 1
#define ADD_BMP2  0x77      // Address of BMP no. 2
#define T_MEASURE 10000     // Measure period (x 0.1ms)

#define LED_TEST  (13)      // Digital output pin for testing


/* ----- Type definitions ----- */

//typedef long long signed int BMP280_S64_t;
//typedef unsigned long int BMP280_U32_t;
//typedef signed long int BMP280_S32_t;


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


/* ----- Global variables definitions ----- */

Adafruit_BMP280 bmp, bmp2; // I2C
const int chipSelect = 10;
SDFile dataFile;
volatile unsigned long timeCount = 0UL;   // A long is 32 bits long = 4294967296 levels
                                          // Using a dt=100us, it can count up to 119 hours
volatile int measureCount = 0;
volatile byte state = 0;                       // State of the flight (E1 corresponds to 0, and so forth)
volatile bool led_on = false;


/* ----- Auxiliary function definitions ----- */

/* Timer 1 Interruption Sub-Routine */
ISR(TIMER1_COMPA_vect){
  timeCount++;                    // Increments time counter
  measureCount++;                 // Increments measure counter
  if(measureCount == T_MEASURE)   // 5600us makes a period of 181.8Hz
    measureCount = 0;             // Reset measure counter for main program

}

/* Save data to SD function */
void salva(String bmp, float h){
  dataFile = SD.open("teste.txt", FILE_WRITE);
  dataFile.print(bmp);
  dataFile.print("Altura: ");
  dataFile.print(h);
  dataFile.println(" metros");
  dataFile.close();
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
    Serial.println(timeCount);
    Serial.println(measureCount);
}


/* ----- Setup function definition ----- */

void setup() {
  /* Serial communication Configuration */
  Serial.begin(9600);
  Serial.println(F("Anhanga Firmware Debugging..."));

  
  /* SD Configuration */
  if(!SD.begin(chipSelect)){ 
    Serial.println("Error in SD card configuration!"); 
    return;
  }

  /* I/O Configuration */
  pinMode(IGN, OUTPUT);     // Configure ignitor pin as digital output
  pinMode(BUZZ, OUTPUT);    // Configure buzzer pin as digital output
  
  LED_config();             // Configure LED for testing code
  
  
  /* BMP280 Configuration */
  if (!bmp.begin(ADD_BMP1)) {
    Serial.println(F("Could not find BMP280 sensor 1 (0x76), check wiring and reset!"));
    return;
  }
  if (!bmp2.begin(ADD_BMP2)) {
    Serial.println(F("Could not find BMP280 sensor 2 (0x77), check wiring and reset!"));
    return;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  
  bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
                  
  /* Timer 1 (16-bit) configuration */
  TCCR1A = 0x00;        // Configure timer on internal source and CTC (Clear Timer on Compare) match mode.
  TCCR1B = B00001010;   // Configure internal clock with prescaler = 8 --> Timer frequency = 2MHz. 
                        // Precise CTC mode - clear on OCR1A register match.
  OCR1A = 200;          // Amount of timer ticks to generate period of 100us (10kHz).
  
  /* Timer 1 Interruption configuration */
  TIMSK1 |= 0x02;       // Enable Interrupt on OCR1A Match
  interrupts();         // Enable interruptions

  Serial.println(F("All configurations finished"));
}

/* ----- Main function definition ----- */

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

  
  if(measureCount == 0){
    LED_toggle();         // For testing code
    // Aquisition routine 
  }
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
