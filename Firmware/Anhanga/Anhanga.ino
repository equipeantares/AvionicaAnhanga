/***************************************************************************
  ------------- Embedded firmware for the Anhangá Rocket -----------------
  Authors: Batata, Joan, Gabriel
  Version: 1.0 - 03/12/2019

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

/*  TODO:
 *   - Implement sea pressure definition at startup   5 
 *   - Implement filter not using objects             1   OK - Use 6 values in dados instead of 10
 *   - Implement state transitions                    2   OK - Apogee done / 
 *   - Implement flag diagnostics                     3   OK - ignState and state
 *   - Implement analogic ignitor readings            4   OK - Function written
 *   - Implement SD initial write                     6   OK - two blank lines
 *   - Implement burning finish condition             7   
 */

/* ----- Library Inclusion ----- */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>
#include "voo.h"


/* ----- Macro definitions ----- */

#define IGN           (8)       // I/O pin for ignitor digital output
#define BUZZ          (10)      // I/O pin for buzzer digital output
#define IGN_READ      (A1)      // I/O pin for analogic ignitor read
#define CS_PIN        (A0)      // Chip Select pin - SPI for SD
#define ADD_BMP1      0x76      // Address of BMP no. 1
#define ADD_BMP2      0x77      // Address of BMP no. 2
#define TIMER_TKS     200       // Timer ticks in a period (Timer 1 runs at 2MHz freq.) - [200 for period of 0.1ms]
#define T_MEASURE     56        // Measure period (x 0.1ms) - [56 for 181.8 Hz]
#define IGN_V_OPEN    1.6       // Ignitor analog reading voltage when open [V]
#define IGN_V_PRESENT 4.5       // Ignitor analog reading voltage when present [V]
#define IGN_V_ACTIVE  0.3       // Ignitor analog reading voltage when MOSFET is active [V]
#define DELTA_H_FLY   50.0      // Height difference with respect to the launch site for flight condition [m]
#define DELTA_H_LAND  20.0      // Height difference with respect to the launch site for landing condition [m]
#define DELTA_V_FLY   5.0       // Minimum velocity for flight condition [m/s]
#define DELTA_T_BURN  2.0       // Minimum time after lift-off from which we begin looking for apogee condition [s]
#define T_MAX_IGN     12.0      // Maximum time after lift-off from which we activate parachute no matter what [s]
#define LED_TEST      (13)      // Digital output pin for testing
#define V0            55.0      // Velocity to trigger apogee condition [m/s]
#define V0_MARGIN     0.2       // Margin [%] around V0 value


/* ----- Constant definitions ----- */

const byte chipSelect = CS_PIN;           // Chip Select pin for SPI communication with SD card
const int initialHeight = 603;            // Initial height for getting sea level pressure [m]
const String dFileName = "antares.txt";   // Data filename in SD  

/* ----- Global variables definitions ----- */

volatile unsigned long timeCount = 0UL;   // A long is 32 bits long = 4294967296 levels
                                          // Using a dt=100us, it can count up to 119 hours
volatile unsigned int measureCount = 0;   // Counter for dividing timer freq. to get measure freq.
volatile unsigned int delayCount = 0;     // Counter for dividing measureCount to get delays
volatile byte state = 0;                  // State of the flight (E1 corresponds to 0, and so forth)
volatile byte ignState = 0;               // State of the ignitor
volatile bool led_on = false;             // Variable for test LED state

Adafruit_BMP280 bmp1, bmp2;               // Object used as BMP280 driver - I2C use
Voo voo1 = Voo(0.0056, 1.0);              // Object containing height, velocity, acceleration - sensor 1
Voo voo2 = Voo(0.0056, 1.0);              // Object containing height, velocity, acceleration - sensor 2
volatile unsigned long timeSave1 = 0UL;   // Variable used for storing moment of last measure of sensor 1
volatile unsigned long timeSave2 = 0UL;   // Variable used for storing moment of last measure of sensor 2
volatile unsigned long timeLiftOff = 0UL; // Variable used for storing lift-off moment
volatile float bmpPi1, bmpPi2;            // Initial pressures on sensors 1 and 2
volatile float bmpHi1, bmpHi2;            // Initial height measured on sensors 1 and 2
volatile float bmpD1, bmpD2;              // Derivative of height w.r.t. pressure on initial pressure 



/* ----- Auxiliary function definitions ----- */

/* Timer 1 Interruption Sub-Routine */
ISR(TIMER1_COMPA_vect){
  timeCount++;                    // Increments time counter
  measureCount++;                 // Increments measure counter
  if(measureCount == T_MEASURE){  // 5600us makes a period of 181.8Hz
    measureCount = 0;             // Reset measure counter for main program
    delayCount++;
  }
}

/* Get present time in seconds */
float get_time(){
  noInterrupts();
  float t = timeCount/10000.0;
  interrupts();
  return(t);
}


/* Save data to SD function */
void save_to_SD(int bmp, Voo dados){
  File dataFile;                                        // Object used as SD driver - SPI use
//  Serial.println(F("Dados de altura internos:"));
  int vsize = 6;
  float fVect [vsize] = {get_time(),                    // Time since start-up in seconds
                        dados.pressao,                  // Last acquired pressure [hPa]
                        dados.altura.getValor(0),       // Last acquired height measure [m]
                        dados.alturaF.getValor(0),      // Last acquired filtered height measure [m]
                        dados.velocidade.getValor(0),   // Last acquired velocity measure [m/s]
                        dados.velocidadeF.getValor(0)   // Last acquired flitered velocity measure [m/s]                    
  };
  byte sVect [2] = {state,                              // State of the fligt
                    ignState                            // State of the ignitor
  };
  dataFile = SD.open(dFileName, FILE_WRITE);
  if(!dataFile){
    Serial.println(F("---> ERRO na abertura do arquivo"));
  }
  dataFile.print(bmp);
  Serial.print(bmp);
  for(int i=0; i<vsize; i++){
    dataFile.print(F("\t"));
    dataFile.print(fVect[i]);
    Serial.print(F("\t"));
    Serial.print(fVect[i],1);
  }
  for(int i=0; i<2; i++){
    dataFile.print(F("\t"));
    dataFile.print(sVect[i]);
    Serial.print(F("\t"));
    Serial.print(sVect[i]);
  }
  dataFile.print(F("\n"));
  Serial.print(F("\n"));
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
  bmpHandle->setSampling(Adafruit_BMP280::MODE_NORMAL,        /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,               /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,               /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,                /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);             /* Standby time. */
  for(int i=0; i<100; i++){
    bmpPi += bmpHandle->readPressure();                        // Read initial pressure [Pa]
    bmpHi += bmpHandle->readAltitude();                        // Read initial altitude (exponential method) [m]
  }
  bmpPi = bmpPi/100.0;                                        // Get average of pressure in 100 measures [Pa]
  bmpHi = bmpHi/100.0;                                        // Get average of height in 100 measures [m]
  bmpPi /= 100.0;                                             // Transform pressure to hPa (divide by 100)
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
  File dataFile;
  
  return(true);
}

/* BMP height reading - Linear method */
float read_BMP_h(byte bmp_nb){
  if(bmp_nb <=0 || bmp_nb >=3){
    Serial.println(F("ERROR: read_BMP_h -> invalid BMP number!"));
    return(0);
  }
  float p,h;
  Adafruit_BMP280 * bmpHandle = (bmp_nb == 1) ? &bmp1 : &bmp2;      // Get bmp object handle accordingly
  Voo * flightHandle = (bmp_nb == 1) ? &voo1 : &voo2;               // Get flight object handle acconrdingly
  float bmpPi = (bmp_nb == 1) ? bmpPi1 : bmpPi2;
  float bmpHi = (bmp_nb == 1) ? bmpHi1 : bmpHi2;
  float bmpDi = (bmp_nb == 1) ? bmpD1 : bmpD2;
  p = bmpHandle->readPressure();                                    // Read pressure [Pa]
  flightHandle->pressao = p;                                        // Save in flight object
  p /= 100.0;                                                       // Transform pressure to hPa
//  h = 44330 * (1.0 - pow(p / 1013.25, 0.1903));                     // Normal calculation of height
  h = bmpHi + bmpDi * (p - bmpPi);                                  // Linearization of height
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
  byte failed = 0;                          // Counter for failed acquisitions from sensors
  while(measuresDone < 3 && failed < 3){     
    if(measureCount == 0){                  // Waits cycle of measurement (acquisition frequency)
      read_h = read_BMP_h(bmp_nb);
//      Serial.println(F("Dados de altura recebidos interior:"));
//      Serial.println(read_h, 5);
      if(read_h < 1700 && read_h > 0){      // Measure validation: h in launch site is 700m + max of flight is 500m = total of 1200m
        acc_h += read_h;                
        measuresDone++;
      }
      else
        failed++;
    }
  }
  if(failed >= 3){                          // If the measure has failed more than 3 times, returns 0.
//    Serial.print(F("ERROR: BMP failed more than 3 times!"));
    return(0.0);
  }
  else
    return(acc_h/3.0);
}

/* Function to get time interval between sensor readings */
float get_dt(byte bmp_nb){
  if(bmp_nb <=0 || bmp_nb >=3){
    Serial.println(F("ERROR: get_dt -> invalid BMP number!"));
    return(0);
  }
  unsigned long timeSave; 
  unsigned int deltaT;
  timeSave = (bmp_nb == 1) ? timeSave1 : timeSave2; // Get last saved time instant
  noInterrupts();                                   // Disable interrupts because of manipulation of timeCount
  deltaT = (unsigned int)(timeCount - timeSave);    // Effective dT calculation
  if(bmp_nb == 1)                                   // Update saved time instant
    timeSave1 = timeCount;
  else
    timeSave2 = timeCount;
  interrupts();                                     // Re-enable interrupts
  return(deltaT/10000.0);
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
      digitalWrite(IGN, HIGH);
      Serial.println(F("LED ON"));
    }
    if(led_on){
      digitalWrite(LED_TEST, LOW);
      digitalWrite(IGN, LOW);
      Serial.println(F("LED OFF"));
    }
    led_on = !led_on;
    Serial.print(F("time = "));
    Serial.println(get_time());
    Serial.print(F("measureCount = "));
    Serial.println(measureCount);
}

/* Ignitor Analog reading - Get ignitor state */
byte get_ignitor_state(){
  int value = analogRead(IGN_READ);
  float voltage = 5*value/1023.0;
  Serial.print(F("Voltage reading is = "));
  Serial.println(voltage,1);
  byte state;
  if(voltage > 0.9*IGN_V_OPEN && voltage < 1.1*IGN_V_OPEN){
     state = 0;
  }
  else if(voltage > 0.9*IGN_V_PRESENT && voltage < 1.1*IGN_V_PRESENT){
     state = 1;
  }
  else if(voltage < 2*IGN_V_ACTIVE){
     state = 2;
  }
  return(state);
}

/* Identify Flight Condition */
bool flight_condition(){
  float h1 = voo1.alturaF.getValor(0);
  float h2 = voo2.alturaF.getValor(0);
  float v1 = voo1.velocidadeF.getValor(0);
  float v2 = voo2.velocidadeF.getValor(0);
  if( (h1 > bmpHi1 + DELTA_H_FLY && v1 > DELTA_V_FLY) ||
      (h2 > bmpHi2 + DELTA_H_FLY && v2 > DELTA_V_FLY) )
  {
    return(true);
  }
  else{
    return(false);
  }
}

/* Identify end of propulsion Condition */
bool burn_end_condition(){
  noInterrupts();
  int timeSinceLO = timeCount - timeLiftOff;
  interrupts();
  if(timeSinceLO/10000.0 > DELTA_T_BURN){
    return(true);
  }
  else{
    return(false);
  }
}

/* Identify Apogee Condition */
bool apogee_condition(){
  float v1 = voo1.velocidadeF.getValor(0);
  float v2 = voo2.velocidadeF.getValor(0);
  noInterrupts();
  long timeSinceLO = timeCount - timeLiftOff;
  interrupts();
  if( ((1-V0_MARGIN)*V0 < v1 && v1 < (1+V0_MARGIN)*V0) || 
      ((1-V0_MARGIN)*V0 < v2 && v2 < (1+V0_MARGIN)*V0) ||
      timeSinceLO/10000.0 > T_MAX_IGN)
  {
    return(true);
  }
  else{
    return(false);
  }
}

/* Identify Landing Condition */
bool landing_condition(){
  float h1 = voo1.alturaF.getValor(0);
  float h2 = voo2.alturaF.getValor(0);
  float v1 = voo1.velocidadeF.getValor(0);
  float v2 = voo2.velocidadeF.getValor(0);
  if( (h1 < bmpHi1 + DELTA_H_LAND && h1 > bmpHi1 - DELTA_H_LAND && v1 < DELTA_V_FLY)  ||    // TODO: Change for absolute velocity!!!
      (h2 < bmpHi2 + DELTA_H_LAND && h2 > bmpHi2 - DELTA_H_LAND && v2 < DELTA_V_FLY) )
  {
    return(true);
  }
  else{
    return(false);
  }
}



/* ----- Setup function definition ----- */

void setup() {
  /* Serial communication Configuration */
  Serial.begin(9600);
  Serial.println(F("Anhanga Firmware Debugging..."));

  /* I/O Configuration */
  pinMode(IGN, OUTPUT);         // Configure ignitor pin as digital output
  pinMode(BUZZ, OUTPUT);        // Configure buzzer pin as digital output
  pinMode(chipSelect, OUTPUT);  // Configure Chip Select pin as digital output
  LED_config();                 // Configure LED for testing code
  Serial.println(F("I/O configuration done."));

  /* SD Configuration */
  if(!SD.begin(chipSelect)){ 
    Serial.println(F("Error in SD card configuration!"));
  }
  else{
    Serial.println(F("SD configuration done."));
    Serial.print(F("Saving data in: "));
    Serial.println(dFileName);
    File dataFile;
    dataFile = SD.open(dFileName, FILE_WRITE);
    if(!dataFile){
      Serial.println(F("---> ERRO na abertura do arquivo"));
    }
    dataFile.println(F("\n"));
    dataFile.close();
  }
  
  /* BMP280 Configuration */
  if(configure_BMP(1) && configure_BMP(2))      // Configuration and initialization funcions of BMP1 and BMP2
    Serial.println(F("BMPs configuration done."));
  else
    Serial.println(F("Problems on BMPs configuration."));

  /* Timer 1 (16-bit) configuration */
  TCCR1A = 0x00;        // Configure timer on internal source and CTC (Clear Timer on Compare) match mode.
  TCCR1B = B00001010;   // Configure internal clock with prescaler = 8 --> Timer frequency = 2MHz. 
                        // Precise CTC mode - clear on OCR1A register match.
  OCR1A = TIMER_TKS;    // Amount of timer ticks to generate period of 100us (10kHz).
  Serial.println(F("Timer 1 configured."));
  
  /* Timer 1 Interruption configuration */
  TIMSK1 |= 0x02;       // Enable Interrupt on OCR1A Match
  interrupts();         // Enable interruptions

  Serial.println(F("Configurations finished."));
  Serial.print(F("Running with Tau = "));
  Serial.println(voo1.tau,2);
}



/* ----- Main function definition ----- */

void loop() {
  // Read height and pressure
  float h1 = read_avg_BMP(1);
  voo1.dt = get_dt(1);
  float h2 = read_avg_BMP(2);
  voo2.dt = get_dt(2);
  // Insert on arrays and do calculations
  voo1.addAltura(h1);
  voo2.addAltura(h2);
  // Save data to SD card
  save_to_SD(1,voo1);
  save_to_SD(2,voo2);
  ignState = get_ignitor_state();
//  switch(state){
//    case 0:
//      if(delayCount > 3636){
//        if(ignState == 1){
//          state = 1;
//        }
//        else if(ignState == 0){
//          state = 3;
//        }
//      }
//      break;
//    case 1:
//      if(delayCount > 1818){
//        digitalWrite(IGN,LOW);
//        digitalWrite(BUZZ,LOW);
//        delayCount = 0;
//        state = 2;
//      }
//      break;
//    case 2:
//      if(delayCount > 1818){
//        digitalWrite(IGN,HIGH);
//        digitalWrite(BUZZ,HIGH);
//        delayCount = 0;
//        state = 1;
//      }
//      break;
//    case 3:
//      digitalWrite(BUZZ,HIGH);
//  }
  switch(state){
    case 0:         // E1 = Initialization
      if(ignState != 1){
        Serial.println(F("PROBLEM: Ignitor not nominal!"));
        state = 10;
      }
      else if (flight_condition()){
        Serial.println(F("E1 --> E2"));
        noInterrupts();
        timeLiftOff = timeCount;
        interrupts();
        state = 1;
      }
      break;
    case 1:         // E2 = Propulsive Flight
      if(burn_end_condition()){
        Serial.println(F("E2 --> E3"));
        state = 2;
      }
      break;
    case 2:         // E3 = Ballistic Flight
      if(apogee_condition()){
        Serial.println(F("E3 --> E4"));
        digitalWrite(IGN, HIGH);    // Activate Ignition
        state = 3;
        delayCount = 0;
      }
      break;
    case 3:         // E4 = Parachute 
      if(ignState == 2 && delayCount > 1818){   // Wait for approximately 10s
        Serial.println(F("Deactivate ignitor."));
        digitalWrite(IGN, LOW);                 // Deactivate Ignition
      }
      else if (ignState == 1){                  // Re-activate Ignition if NC is still present (resistance unchanged)
        Serial.println(F("PROBLEM: Re-activate ignition!"));
        digitalWrite(IGN, HIGH);
        delayCount = 0;
      }
      if(landing_condition()){
        Serial.println("E4 --> E5");
        state = 4;
      }
      break;
    case 4:         // E5 = Ground reached
      digitalWrite(IGN, LOW);                 // Deactivate Ignition
      digitalWrite(BUZZ, HIGH);               // Activate Buzzer
      break;
    case 10:        // E11 = Failure mode
      Serial.println(F("Failure state!"));
      while(1){                               // Continuous loop of buzzer beeps (1s)
        if(delayCount > 362){
          digitalWrite(BUZZ, HIGH);           // Activate Buzzer
          delayCount = 0;
        }
        else if(delayCount > 181){
          digitalWrite(BUZZ, LOW);            // De-activate Buzzer
        }
      }
      return;
      break;
  }
}
