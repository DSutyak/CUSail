#include "sensors.h"
#include <SPI.h>
#include "TinyGPS++.h"
#include <Pixy.h>
TinyGPSPlus gps;
data_t sensorData;
float boatDirections[numBoatDirReads];
float windDirections[numWindDirReads];
float prevWindDirection = 270;

// Type to convert the bytes from SPI to float (Used as part of the IMU code)
union u_types {
    byte b[4];
    float fval;
} imu_data[3];  // Create 3 unions, one for each euler angle

int boatDirArrayNum = 0; // ints helping in replacing array elements
int windDirArrayNum = 0;

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

/* Returns servo command for sail servo for inputted sail angle
 * Precondition: Sail Angle in 0.. 360 w.r.t boat
 */
double sailMap(double sailAngle){
  double newSailAngle;
  if (sailAngle <= 90){
    newSailAngle = map(sailAngle, 0, 90, 71, 91.5);
  }
  else if (sailAngle <= 180){
    newSailAngle = map(sailAngle, 90, 180, 91.5, 111.5);
  }
  else if (sailAngle <= 270){
    newSailAngle = map(sailAngle, 180, 270, 111.5, 131);
  }
  else{
   newSailAngle = map(sailAngle, 270, 360, 131, 146.5);
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
    newTailAngle=map(newTailAngle,-30,0,38,84);
  }
  else if (newTailAngle > 0 ){
    newTailAngle=map(newTailAngle,0,30,85,144);
  }
  return newTailAngle;
}



double sailMapBench( double sailAngle){
  double newSailAngle;
  if (sailAngle <= 90){
    newSailAngle = map(sailAngle, 0, 90, 72, 78.5);
  }
  else if (sailAngle <= 180){
    newSailAngle = map(sailAngle, 90, 180, 78.5, 84);
  }
  else if (sailAngle <= 270){
    newSailAngle = map(sailAngle, 180, 270, 84, 90);
  }
  else{
   newSailAngle = map(sailAngle, 270, 360, 90, 96);
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
    newTailAngle=map(newTailAngle,-30,0,69,71);
  }
  else if (newTailAngle > 0 ){
    newTailAngle=map(newTailAngle,0,30,71,73);
  }
  return newTailAngle;
}

/*Sensor setup*/
void initSensors(void) {
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial1.begin(9600);
  Serial3.begin(9600);

  // initialize data structure
  sensorData = *(data_t*) malloc(sizeof(data_t));
  sensorData = {};

  //Set Pin Modes
  pinMode(RS_CSN, OUTPUT);
  pinMode(IMU_CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  pinMode(redLED1, OUTPUT);
  pinMode(redLED2, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);

  //Set Slave Select signals High i.e disable chips
  digitalWrite(RS_CSN, HIGH);
  digitalWrite(IMU_CSN, HIGH);

  //Initialize SPI
  SPI.begin();

  //Begin Pixy
  Pixy pixy;
  pixy.init();

}

float dirAverage(int numToAverage, float arrayToAverage[]) {
    float sum = 0;
    for(int i=0; i < numToAverage; i++) {
      sum += arrayToAverage[i];
    }
  float avgDir = sum/numToAverage;
  return avgDir;
}

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
  int pos = ( (unsigned long) angle)*360UL/16384UL;

  //get angle with respect to North
  int wind_wrtN = ((int)(pos + sensorData.boatDir))%360;

  //---filter wind---
  float newSinWind = ( (sin(prevWindDirection) + (1/16)*sin(wind_wrtN)) / (1+ (1/16)) );
  float newCosWind = ( (cos(prevWindDirection) + (1/16)*cos(wind_wrtN)) / (1+ (1/16)) );
  float newWind = atan2(newSinWind, newCosWind);
  //------
  sensorData.windDir = newWind;
  prevWindDirection = wind_wrtN;

}

/*Sets value of sensorData.lati, sensorData.longi and sensorData.dateTime
* to current lattitude, current longitude and current date/time respectively*/
void sGPS(void) {
  while (Serial3.available() > 0) {
    gps.encode(Serial3.read());
    sensorData.longi = gps.location.lng();
    sensorData.lati = gps.location.lat();
    sensorData.dateTime.year = gps.date.year();
    sensorData.dateTime.month = gps.date.month();
    sensorData.dateTime.day = gps.date.day();
    sensorData.dateTime.hour = gps.time.hour();       // Hour (0-23) (u8)
    sensorData.dateTime.minute = gps.time.minute();   // Minute (0-59) (u8)
    sensorData.dateTime.seconds = gps.time.second();  // Second (0-59) (u8)
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
    }

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

  boatDirections[boatDirArrayNum%numBoatDirReads] = boatDir;
  boatDirArrayNum += 1;
  float averageBoatDirection = dirAverage(numBoatDirReads, boatDirections);
  sensorData.boatDir = boatDir; //Not using average right now
  sensorData.pitch  = (imu_data[0].fval)*(180/PI);
  sensorData.roll = (imu_data[2].fval)*(180/PI);
}





