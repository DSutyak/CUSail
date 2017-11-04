#include "sensors.h"
#include <SPI.h>
#include "TinyGPS++.h"
#include <PixyI2C.h>
#include "navigation.h"
#include "navigation_helper.h"

//Begin Pixy
PixyI2C pixy;

TinyGPSPlus gps;
data_t sensorData;

float prevSinWind = sin(270);
float prevCosWind = sin(270);

double objectVals[2] = {400.0,400.0};
float prevWindDirection = 270;
//float angleCorrection = 27;//Small sail angle correction
float angleCorrection = -26; //Big sail angle correction
float averageWeighting = 0.0625;

// Type to convert the bytes from SPI to float (Used as part of the IMU code)
union u_types {
    byte b[4];
    float fval;
} imu_data[3];  // Create 3 unions, one for each euler angle

/*Transfers commands through SPI*/
byte transferByte(byte byteToWrite) {
  byte Result = 0x00;
  digitalWrite(IMU_CSN,LOW);
  delay(1);
  Result = SPI.transfer(byteToWrite);
  delay(1);
  digitalWrite(IMU_CSN,HIGH);
  return Result;
}

/*Returns a byte with the endian swapped*/
void endianSwap(byte temp[4]) {
  byte myTemp = temp[0];
  temp[0] = temp[3];
  temp[3] = myTemp;
  myTemp = temp[1];
  temp[1] = temp[2];
  temp[2] = myTemp;
}

/* Updates objectVals array
 * Only updates x-position
 * NOTE: This only detects objects set to
 * signature 1 on the pixy cam
 */
void addObjects(void) {
    Serial.println("gettting pixy objects");
    uint16_t blocks = pixy.getBlocks();
    if (blocks != 0) {
        for (int j = 0; j < blocks; j++) {
            if (pixy.blocks[j].signature == 1) {
              Serial.println("object detected");
                int32_t xLocation = pixy.blocks[j].x; // range: 0 to 319
                double tmp = objectVals[1];
                objectVals[0] = tmp;
                objectVals[1] = xLocation;
              Serial.println(objectVals[0]);
              Serial.println(objectVals[1]);
            }
        }
    }
    else {
        Serial.println("No object detected");
        double tmp = objectVals[1];
        objectVals[0] = tmp;
        objectVals[1] = 400.0;
    }
}


/* Returns servo command for sail servo for inputted sail angle
 * Precondition: Sail Angle in 0.. 360 w.r.t boat
 */
double sailMap(double sailAngle){
  double newSailAngle;
  if (sailAngle <= 90){
    newSailAngle = map(sailAngle, 0, 90, 142, 125);
  }
  else if (sailAngle <= 180){
    newSailAngle = map(sailAngle, 90, 180, 125, 108);
  }
  else if (sailAngle <= 270){
    newSailAngle = map(sailAngle, 180, 270, 108, 91);
  }
  else{
   newSailAngle = map(sailAngle, 270, 360,91, 74);
  }
  return newSailAngle;
}

/* Returns servo command tail servo for inputted sail angle and tail angle
 * Precondition: Sail Angle in 0.. 360 w.r.t boat, Tail Angle in -180.. 180 w.r.t boat
 */
double tailMap(double sailAngle, double tailAngle){

  if (sailAngle > 180){ //convert sail angle to -180.. 180
    sailAngle -= 360;
  }

  double newTailAngle=tailAngle-sailAngle; //calculate position of tail with respect to sail

  //make sure tail angle is in range -180.. 180
  if(newTailAngle<-180){
    newTailAngle+=360;
  }
  else if(newTailAngle>180){
    newTailAngle-=360;
  }

  //map to servo commands
  if (newTailAngle <= 0 ){
    newTailAngle=map(newTailAngle,-30,0,160,100);
  }

  else if (newTailAngle > 0 ){
    newTailAngle=map(newTailAngle,0,30,100,60);
  }

  return newTailAngle;

}



double sailMapBench( double sailAngle){
  double newSailAngle;
  if (sailAngle <= 90){
    newSailAngle = map(sailAngle, 0, 90, 72, 77);
  }
  else if (sailAngle <= 180){
    newSailAngle = map(sailAngle, 90, 180, 77, 83);
  }
  else if (sailAngle <= 270){
    newSailAngle = map(sailAngle, 180, 270, 83, 88);
  }
  else{
   newSailAngle = map(sailAngle, 270, 360, 88, 94);
  }
  return newSailAngle;
}

double tailMapBench( double sailAngle, double tailAngle){
  if (sailAngle > 180){ //convert sail angle to -180.. 180
    sailAngle -= 360;
  }

  double newTailAngle=tailAngle-sailAngle; //calculate position of tail with respect to sail

  //make sure tail angle is in range -180.. 180
  if(newTailAngle<-180){
    newTailAngle+=360;
  }
  else if(newTailAngle>180){
    newTailAngle-=360;
  }
  //map to servo commands
  if (newTailAngle <= 0 ){
    newTailAngle=map(newTailAngle,-30,0,85,45);
  }
  else if (newTailAngle > 0 ){
    newTailAngle=map(newTailAngle,0,30,45,5);
  }
  return newTailAngle;
}

/*Sensor setup*/
void initSensors(void) {
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial1.begin(9600);
  Serial3.begin(9600);

  //Initialize data structure
  sensorData = *(data_t*) malloc(sizeof(data_t));
  sensorData = {};

  //Set Pin Modes
  pinMode(RS_CSN, OUTPUT);
  pinMode(IMU_CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(orangeLED, OUTPUT);

  //Set Slave Select signals High i.e disable chips
  digitalWrite(RS_CSN, HIGH);
  digitalWrite(IMU_CSN, HIGH);
  digitalWrite(greenLED, HIGH);

  //Initialize SPI
  SPI.begin();

  //Initialize pixycam
  pixy.init();

  sensorData.boatDir = 0; //Boat direction w.r.t North
  sensorData.sailAngleBoat = 0; //Sail angle for use of finding wind wrt N
  sensorData.tailAngleBoat = 0; //Tail angle for use of finding wind wrt N
  sensorData.pitch = 0;
  sensorData.roll = 0;
  sensorData.windDir = 0; // Wind direction w.r.t North
  sensorData.x = 0; // Longitude of current global position;
  sensorData.y = 0; // Longitude of current global position;
  sensorData.x = 0;
  sensorData.y = 0;
}

/*----------LED control----------*/

void lightAllLEDs(){
  digitalWrite(redLED, HIGH);
  digitalWrite(orangeLED, HIGH);
  digitalWrite(greenLED, HIGH);
}

void lowAllLEDs(){
  digitalWrite(redLED, LOW);
  digitalWrite(orangeLED, LOW);
  digitalWrite(greenLED, LOW);
}

/*-------------------------------*/

/*Sets value of sensorData.windDir to current wind direction w.r.t North*/
void sRSensor(void) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

  //Send the Command Frame
  digitalWrite(RS_CSN, LOW);
  delayMicroseconds(1);
  SPI.transfer16(0xFFFF);
  digitalWrite(RS_CSN,HIGH);

  //Read data frame
  digitalWrite(RS_CSN, LOW);
  delayMicroseconds(1);
  unsigned int angle = SPI.transfer16(0xC000);
  digitalWrite(RS_CSN, HIGH);
  SPI.endTransaction();

  //mask the MSB and 14th bit
  angle = (angle & (0x3FFF));

  //convert to a 360 degree scale
  int reading = ( (unsigned long) angle)*360UL/16384UL;
  reading += angleCorrection;
  reading = (reading<0)?(reading+360):reading;

//  Serial1.print("----------Rotary Sensor----------\n");
//  Serial1.print("Current wind reading w.r.t Sail: ");
//  Serial1.println(reading);

  //get angle with respect to North
//  Serial1.print("sailAngle: ");
//  Serial1.println(sensorData.sailAngleBoat);
  float wind_wrtN = ((int)(reading + sensorData.sailAngleBoat))%360;
  wind_wrtN = ((int)(wind_wrtN + sensorData.boatDir))%360;

  //filter wind
  float newSinWind = ( (sin(prevWindDirection*PI/180) + (averageWeighting)*sin(wind_wrtN*PI/180)) / (1 + averageWeighting) );
  float newCosWind = ( (cos(prevWindDirection*PI/180) + (averageWeighting)*cos(wind_wrtN*PI/180)) / (1 + averageWeighting) );
  wind_wrtN = atan2(newSinWind, newCosWind);
  wind_wrtN = wind_wrtN*180/PI;
  wind_wrtN = (wind_wrtN<0)?wind_wrtN+360:wind_wrtN;

//  Serial1.print("Raw Wind WRT NORTH: ");
//  Serial1.println(wind_wrtN);

  sensorData.windDir = wind_wrtN;
  prevWindDirection = wind_wrtN;
  }

/*Sets value of sensorData.lati, sensorData.longi and sensorData.dateTime
* to current lattitude, current longitude and current date/time respectively*/
coord_xy point;
void sGPS(void) {
  while (Serial3.available() > 0) {
    gps.encode(Serial3.read());
      if( origin.x != 0){
      point = xyPoint( coord_t({gps.location.lng(),gps.location.lat()}));
      sensorData.x=point.x;
      sensorData.y=point.y;}

      sensorData.dateTime.hour = gps.time.hour();
      sensorData.dateTime.minute = gps.time.minute();
      sensorData.dateTime.seconds = gps.time.second();
    }

}

/*Sets value of sensorData.boatDir, sensorData.pitch and sensorData.roll
* to current boat direction w.r.t North, current boat pitch and current boat roll*/
void sIMU(void) {
  SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0 ));

  // Clear the internal data buffer on the IMU
  byte result = transferByte(0x01);
  // Send start of packet:
  result = transferByte(0xF6);
  delay(1);


  // Send command (tared euler angles)
  result = transferByte(0x01);
  delay(1);

  // Get status of device:
  result = transferByte(0xFF);
  delay(1);


  while (result != 0x01) {  // Repeat until device is Ready
    delay(1);
    result = transferByte(0xFF);
//    Serial1.println("IMU Stuck!");
    digitalWrite(redLED, HIGH);
    }

  digitalWrite(redLED, LOW);

  // Get the 12 bytes of return data from the device:
  for (int ii=0; ii<3; ii++) {
    for (int jj=0; jj<4; jj++) {
      imu_data[ii].b[jj] =  transferByte(0xFF);
      delay(1);
    }
  }

  SPI.endTransaction();

  for( int mm=0; mm<3; mm++) {
    endianSwap(imu_data[mm].b);
  }

  float boatDir =  ((imu_data[1].fval)*(180/PI));
  if (boatDir < 0) {
    boatDir += 360;
  }

  sensorData.boatDir = boatDir;
//  Serial1.print("BoatDIR raw:");
//  Serial1.println(boatDir);
  sensorData.pitch  = (imu_data[0].fval)*(180/PI);
  sensorData.roll = (imu_data[2].fval)*(180/PI);
}
