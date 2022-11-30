/***************************************************************************
  ------------- Embedded firmware for LASC 2021/2022 -----------------
  Authors: Batata, Joan, Gabriel
  Adapted by: Andre, Reis
  Version: 3.0 - 08/2022
  Description:
    This firmware implements a state machine, keeping track of the rocket
    flight phases and acting accordingly.
    There are 6 main flight states:
      E1 - Ground before launch
      E2 - Propulsive flight
      E3 - Ballistic flight
      E4 - Descent with parachute
      E5 - Ground after flight
      E10- Failure mode
    Between each of these states, there are main transitions and actions 
    on the rocket, the parachute launch being the most important one.
    
    The rocket phases and state transitions, as well as the fluxogram of this
    code are presented in detail in the firmware development report, located 
    in Antares Google drive repository.
  
 ***************************************************************************/

/*  TODO:
 *   - Implement sea pressure definition at startup   5
 *   - Implement burning finish condition             7   
 */

/* ----- Library Inclusion ----- */

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>
#include <Ticker.h>
#include "voo.h"


/* ----- Macro definitions ----- */

#define IGN           10        // I/O pin for ignitor digital output
#define BUZZ          9         // I/O pin for buzzer digital output
#define PWR_READ      A0        // I/O pin for analogic power read
#define ADD_BMP       0x76      // Address of BMP
#define TIMER_TKS     500       // Timer ticks in a period (Timer runs at 5MHz freq.) - [500 for period of 0.1ms]
#define T_MEASURE     56        // Measure period (x 0.1ms) - [56 for 181.8 Hz]
#define DELTA_H_FLY   50.0      // Height difference with respect to the launch site for flight condition [m]
#define DELTA_H_LAND  20.0      // Height difference with respect to the launch site for landing condition [m]
#define DELTA_V_FLY   20.0      // Minimum velocity for flight condition [m/s]
#define DELTA_T_BURN  4.0       // Minimum time after lift-off from which we begin looking for apogee condition [s]
#define T_MAX_IGN     20.0      // Maximum time after lift-off from which we activate parachute no matter what [s]
#define BLUE_LED      0         // Digital output blue led for debbugin
#define GREEN_LED     2         // Digital output green led for debbugin
#define V0            15.0      // Velocity to trigger apogee condition [m/s]
#define V0_MARGIN     0.2       // Margin [%] around V0 value
#define CS_PIN        15        // Chip Select pin - SPI for SD


/* ----- Constant definitions ----- */

const byte chipSelect = CS_PIN;         // Chip Select pin for SPI communication with SD card
const int initialHeight = 645;            // Initial height for getting sea level pressure [m]
const String dFileName = "antares.txt";   // Data filename in SD  

/* ----- Global variables definitions ----- */

volatile unsigned long timeCount = 0UL;   // A long is 32 bits long = 4294967296 levels
                                          // Using a dt=100us, it can count up to 119 hours
volatile unsigned int measureCount = 0;   // Counter for dividing timer freq. to get measure freq.
volatile unsigned int delayCount = 0;     // Counter for dividing measureCount to get delays
volatile byte state = 0;                  // State of the flight (E1 corresponds to 0, and so forth)
volatile bool led_on = false;             // Variable for test LED state

Ticker timer;                             // Object used as interrupt timer
Adafruit_BMP280 bmp;                      // Object used as BMP280 driver - I2C use
Voo voo = Voo(0.0056, 1.0);               // Object containing height, velocity, acceleration - sensor 
volatile unsigned long timeSave = 0UL;    // Variable used for storing moment of last measure of sensor 
volatile unsigned long timeLiftOff = 0UL; // Variable used for storing lift-off moment
volatile float bmpPi;                     // Initial pressures on sensor
volatile float bmpHi;                     // Initial height measured on sensor
volatile float bmpDi;                      // Derivative of height w.r.t. pressure on initial pressure 



/* ----- Auxiliary function definitions ----- */

/* Timer 1 Interruption Sub-Routine for AVR1*/
/*
ISR(TIMER1_COMPA_vect){
  timeCount++;                    // Increments time counter
  measureCount++;                 // Increments measure counter
  if(measureCount == T_MEASURE){  // 5600us makes a period of 181.8Hz
    measureCount = 0;             // Reset measure counter for main program
    delayCount++;
  }
}
*/

/* Timer Interruption Sub-Routine for ESP8266 */
void ICACHE_RAM_ATTR onTime() {
  timeCount++;
  measureCount++;
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
void save_to_SD(Voo dados){
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
  byte sVect [1] = {state};                              // State of the flight
         
  dataFile = SD.open(dFileName, FILE_WRITE);
  if(!dataFile){
    Serial.println(F("---> ERRO na abertura do arquivo"));
  }
;
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
bool configure_BMP(){
  
  bmpPi = 0;
  bmpHi = 0;
  bmpDi = 0;
  int address = ADD_BMP;                                        // Gets bmp adress accordingly
  //Adafruit_BMP280 * bmpHandle = bmp;                          // Gets bmp object handle accordingly
  if (!bmp.begin(0x76)) {                                       // Initialize BMP
    Serial.print(F("Could not find BMP280 sensor (add="));
    Serial.print(address);
    Serial.println(F("), check wiring and reset!"));
    return(false);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,        /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,               /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,               /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,                /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);             /* Standby time. */
  for(int i=0; i<100; i++){
    bmpPi += bmp.readPressure();                        // Read initial pressure [Pa]
    bmpHi += bmp.readAltitude();                        // Read initial altitude (exponential method) [m]
  }
  bmpPi = bmpPi/100.0;                                        // Get average of pressure in 100 measures [Pa]
  bmpHi = bmpHi/100.0;                                        // Get average of height in 100 measures [m]
  bmpPi /= 100.0;                                             // Transform pressure to hPa (divide by 100)
  bmpDi = -2260.209*pow(bmpPi,-0.8097);                       // Calculate derivative in initial pressure
  
  Serial.println(F("Finished configuration of BMP"));
  Serial.print(F("bmpPi = "));
  Serial.println(bmpPi,3);
  Serial.print(F("bmpHi = "));
  Serial.println(bmpHi,3);
  Serial.print(F("bmpDi = "));
  Serial.println(bmpDi,3);
  //File dataFile;
  
  return(true);
}

/* BMP height reading - Linear method */
float read_BMP_h(){
  
  float p,h;
  //Adafruit_BMP280 * bmpHandle = bmp;                            // Get bmp object handle accordingly
  //Voo * flightHandle = voo;                                     // Get flight object handle acconrdingly
  p = bmp.readPressure();                                // Read pressure [Pa]
  voo.pressao = p;                                    // Save in flight object
  p /= 100.0;                                                   // Transform pressure to hPa
//  h = 44330 * (1.0 - pow(p / 1013.25, 0.1903));               // Normal calculation of height
  h = bmpHi + bmpDi * (p - bmpPi);                              // Linearization of height
  return(h);
}

/* BMP average valid reading - call on loop */
float read_avg_BMP(){
  
  float read_h = 0.0;
  float acc_h = 0.0;
  byte measuresDone = 0;                                        // Counter for valid height acquisitions from sensors
  byte failed = 0;                                              // Counter for failed acquisitions from sensors
  while(measuresDone < 3 && failed < 3){     
    if(measureCount == 0){                                      // Waits cycle of measurement (acquisition frequency)
      read_h = read_BMP_h();
      if(read_h < 2000 && read_h > 0){                          // Measure validation: h in launch site is 645m + max of flight is 1000m = total of 1645m
        acc_h += read_h;                
        measuresDone++;
      }
      else
        failed++;
    }
  }
  if(failed >= 3){                                              // If the measure has failed more than 3 times, returns 0.
//    Serial.print(F("ERROR: BMP failed more than 3 times!"));
    return(0.0);
  }
  else
    return(acc_h/3.0);
}

/* Function to get time interval between sensor readings */
float get_dt(){
  unsigned long timeSave; 
  unsigned int deltaT;
  timeSave = timeSave;                                          // Get last saved time instant
  noInterrupts();                                               // Disable interrupts because of manipulation of timeCount
  deltaT = (unsigned int)(timeCount - timeSave);                // Effective dT calculation
  timeSave = timeCount;                                         // Update saved time instant
  interrupts();                                                // Re-enable interrupts
  return(deltaT/10000.0);
}

/* LED configuration - call on setup */
void LED_config(){ 
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, LOW);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);
}

/* test routine*/
void Toggle(){
  digitalWrite(BUZZ, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(BUZZ, LOW);
  digitalWrite(GREEN_LED, LOW);
  delay(400);
  digitalWrite(BUZZ, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  delay(100);
  digitalWrite(BUZZ, LOW);
  digitalWrite(BLUE_LED, LOW);
}

/* Ignitor Analog reading - Get ignitor state */
/*
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
*/

/* Identify Flight Condition */
bool flight_condition(){
  float h = voo.alturaF.getValor(0);
  float v = voo.velocidadeF.getValor(0);
  if( (h > bmpHi + DELTA_H_FLY && v > DELTA_V_FLY))
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
  float v = voo.velocidadeF.getValor(0);
  noInterrupts();
  long timeSinceLO = timeCount - timeLiftOff;
  interrupts();
  if( ((1-V0_MARGIN)*V0 < v && v < (1+V0_MARGIN)*V0) ||
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
  float h = voo.alturaF.getValor(0);
  float v = voo.velocidade.getValor(0);
  if((h < bmpHi + DELTA_H_LAND && h > bmpHi - DELTA_H_LAND && v < DELTA_V_FLY))
  {
    return(true);
  }
  else{
    return(false);
  }
}

/*Create an header for the text file in the SD card*/
void Header(){
    File dataFile;                                        // Object used as SD driver - SPI use
//  Serial.println(F("Dados de altura internos:"));
         
  dataFile = SD.open(dFileName, FILE_WRITE);
  if(!dataFile){
    Serial.println(F("---> ERRO na abertura do arquivo"));
  }
  else {
      dataFile.print(F("\t"));
      dataFile.print(F("Time [s]"));
      dataFile.print(F("\t"));
      dataFile.print(F("Pressure [Pa]"));
      dataFile.print(F("\t"));
      dataFile.print(F("Height [m]"));
      dataFile.print(F("\t"));
      dataFile.print(F("fHeight [m]"));
      dataFile.print(F("\t"));
      dataFile.print(F("Velocity [m/s]"));
      dataFile.print(F("\t"));
      dataFile.print(F("fVelocity [m/s]"));
   }
}


/* ----- Setup function definition ----- */

void setup() {
  delay(5000);
  /* Serial communication Configuration */
  Serial.begin(9600);
  Serial.println(F("Anhanga Firmware Debugging..."));

  /* I/O Configuration */
  pinMode(IGN, OUTPUT);                                     // Configure ignitor pin as digital output
  digitalWrite(IGN, HIGH);                                  //The ignition pin activates as LOW, so it is initialized HIGH for safety reasons
  pinMode(BUZZ, OUTPUT);                                    // Configure buzzer pin as digital output
  pinMode(chipSelect, OUTPUT);                              // Configure Chip Select pin as digital output
  LED_config();                                             // Configure LED for testing code
  Serial.println(F("I/O configuration done."));
  Toggle();
  delay(1000);
  /* SD Configuration */
  if(!SD.begin(chipSelect)){ 
    Serial.println(F("Error in SD card configuration!"));
    state = 10;
    return;
  }
  else{
    Serial.println(F("SD configuration done."));
    Serial.print(F("Saving data in: "));
    Serial.println(dFileName);
    File dataFile;
    dataFile = SD.open(dFileName, FILE_WRITE);
    if(!dataFile){
      Serial.println(F("---> ERRO na abertura do arquivo"));
      state = 10;
      return;
    }
  Header();
    dataFile.println(F("\n"));
    dataFile.close();
  }
  Toggle();
  delay(1000);
  
  /* BMP280 Configuration */
  if(configure_BMP())                                       // Configuration and initialization funcions of BMP1 and BMP2
    Serial.println(F("BMP configuration done."));
  else{
    Serial.println(F("Problems on BMP configuration."));
    state = 10;
    return;
    }
  Toggle();
  delay(1000);
    
  /* Timer 1 (16-bit) configuration */
//  TCCR1A = 0x00;                                            // Configure timer on internal source and CTC (Clear Timer on Compare) match mode.
//  TCCR1B = B00001010;                                       // Configure internal clock with prescaler = 8 --> Timer frequency = 2MHz. 
//                                                            // Precise CTC mode - clear on OCR1A register match.
//  OCR1A = TIMER_TKS;                                        // Amount of timer ticks to generate period of 100us (10kHz).
//  Serial.println(F("Timer 1 configured."));
//  
//  /* Timer 1 Interruption configuration */
//  TIMSK1 |= 0x02;                                           // Enable Interrupt on OCR1A Match
//  interrupts();                                             // Enable interruptions

  timer1_attachInterrupt(onTime);                             // Add ISR Function
  
  /* Dividers:
    TIM_DIV1 = 0,   //80MHz (80 ticks/us - 104857.588 us max)
    TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
    TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
  Reloads:
    TIM_SINGLE  0 //on interrupt routine you need to write a new value to start the timer again
    TIM_LOOP  1 //on interrupt the counter will start with the same value again
  */
  
  timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP);               // Configure timer1  
  timer1_write(TIMER_TKS);                                    // Amount of timer ticks to generate period of 100us (10kHz).
  interrupts();                                               // Enable interruptions
  Serial.println(F("Timer 1 configured."));

  Serial.println(F("Configurations finished."));
  Toggle();
  Serial.print(F("Running with Tau = "));
  Serial.println(voo.tau,2);
}



/* ----- Main function definition ----- */

void loop() {
  
  // Read height and pressure
  float h = read_avg_BMP();
  voo.dt = get_dt();
  
  // Insert on arrays and do calculations
  voo.addAltura(h);
  
  // Save data to SD card
  save_to_SD(voo);

  switch(state){
    case 0:         // E1 = Initialization
    
      if (flight_condition()){
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
        digitalWrite(IGN, LOW);    // Activate Ignition
        state = 3;
        delayCount = 0;
      }
      break;
    case 3:         // E4 = Parachute 
      if(delayCount > 1818){   // Wait for approximately 10s
        Serial.println(F("Deactivate ignitor."));
        digitalWrite(IGN, HIGH);                 // Deactivate Ignition
      }
      else if (voo.velocidadeF.getValor(0) > 10){                  // Re-activate Ignition if NC is still present (resistance unchanged)
        Serial.println(F("PROBLEM: Re-activate ignition!"));
        digitalWrite(IGN, LOW);
        delayCount = 0;
      }
      if(landing_condition()){
        Serial.println("E4 --> E5");
        state = 4;
      }
      break;
    case 4:         // E5 = Ground reached
      digitalWrite(IGN, HIGH);                 // Deactivate Ignition
      digitalWrite(BUZZ, HIGH);               // Activate Buzzer
      break;
    
    case 10:        // E11 = Failure mode
      Serial.println(F("Failure state!"));
      while(1){
        yield();
        // Continuous loop of buzzer beeps (1s)
        digitalWrite(BUZZ, HIGH);           // Activate Buzzer
        delay(1000);
        digitalWrite(BUZZ, LOW);            // De-activate Buzzer
        delay(1000);
      }
      return;
      break;
  }
}
