/*AMS Rotary Sensor AS5147
 Measures absolute angle position referenced at a set NORTH

 Required Pins
 
 UNO: MOSI pin 11
      MISO pin 12 
      CLK  pin 13
      CSN  pin 10

 Mega: MOSI pin 51
       MISO pin 50    
       CLK  pin 52
       CSN  pin 53  

  Due: MOSI pin 75
       MISO pin 74    
       CLK  pin 76
       CSN  pin 52 (or any other chip select pin)  
 */

#include <SPI.h>

//Set Slave Select Pin
//MOSI, MISO, CLK are handeled automatically
int CSN = 52 ;
int SO = 74;
int SI = 75;
int CLK = 76; 
int redLED = 22;
unsigned int angle;
float angleCorrection = -26
; 
float prevWindDirection = 1;
float boatDirection = 0;
float prevSinWind = sin(270*PI/180);
float prevCosWind = sin(270*PI/180);

void setup() {
  //Pour a bowl Serial
  Serial.begin(9600); //PC
  Serial1.begin(9600); //Xbee

  //Set Pin Modes
  pinMode(36, OUTPUT);
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, LOW);
  
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN, HIGH);
  //Turn green LED On
  digitalWrite(36, HIGH);
  
  //Initialize SPI 
  SPI.begin();
}

void loop() {


  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));
  
  //Send the Command Frame
  digitalWrite(CSN, LOW);
  delayMicroseconds(1);
  SPI.transfer16(0xFFFF);
  digitalWrite(CSN,HIGH);

  //Read data frame
  digitalWrite(CSN, LOW);
  delayMicroseconds(1);
  angle = SPI.transfer16(0xC000);
  digitalWrite(CSN, HIGH);
  SPI.endTransaction();
 
  //mask the MSB and 14th bit
  angle = (angle & (0x3FFF));

  //covert to a 360 degree scale
  float reading = ( (unsigned long) angle)*360UL/16384UL;
  reading += angleCorrection;
  reading = (reading<0)?(reading+360):reading;
  reading  = (int) reading%360;
  float reading_wrtN = ((int)(reading + boatDirection))%360;

   //---filter wind---
    float newSinWind = ( (sin(prevWindDirection*PI/180) + (0.125)*sin(reading_wrtN*PI/180)) / (1.125) );
    float newCosWind = ( (cos(prevWindDirection*PI/180) + (0.125)*cos(reading_wrtN*PI/180)) / (1.125) );
    float wind = atan2(newSinWind, newCosWind);
    wind = wind*180/PI;
    wind = (wind<0)?wind+360:wind;

    prevSinWind = newSinWind;
    prevCosWind = newCosWind;

    
    
  prevWindDirection = wind;
  
  //print angle to the screen
  Serial.print("Wind: ");Serial.println(wind);
  Serial.print("reading: ");Serial.println(reading_wrtN);
  Serial1.println(reading);
  
  delay(1000);
}


