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

void setup() {
  //Pour a bowl Serial
  Serial.begin(9600); //PC
  Serial1.begin(9600); //Xbee

  //Set Pin Modes
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  pinMode(redLED, OUTPUT);
  digitalWrite(redLED, LOW);
  
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN, HIGH);
  
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
  int pos = ( (unsigned long) angle)*360UL/16384UL;

  //print angle to the screen
  Serial.println(pos);
  Serial1.println(pos);
  
  delay(1000);
}


