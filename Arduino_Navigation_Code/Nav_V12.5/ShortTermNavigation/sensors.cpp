#include "sensors.h"
#include <SPI.h>
#include "TinyGPS++.h"
#include <PixyI2C.h>

//Begin Pixy
PixyI2C pixy;
  
TinyGPSPlus gps;
data_t sensorData;

double objectVals[2] = {400.0,400.0};
float prevWindDirection = 100;
float angleCorrection = -121;

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
      Serial.println("object detected");
        for (int j = 0; j < blocks; j++) {
            if (pixy.blocks[j].signature == 1) {
                int32_t xLocation = pixy.blocks[j].x; // range: 0 to 319
                double tmp = objectVals[1];
                objectVals[0] = tmp;
                objectVals[1] = xLocation;
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

  //Initialize data structure
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

  //Initialize pixycam
  pixy.init();
}

/*----------LED control----------*/

void lightAllLEDs(){
  digitalWrite(redLED1, HIGH);
  digitalWrite(redLED2, HIGH);
  digitalWrite(yellowLED, HIGH);
  digitalWrite(blueLED, HIGH);
  digitalWrite(greenLED, HIGH);
}

void lowAllLEDs(){
  digitalWrite(redLED1, LOW);
  digitalWrite(redLED2, LOW);
  digitalWrite(yellowLED, LOW);
  digitalWrite(blueLED, LOW);
  digitalWrite(greenLED, HIGH);
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

  Serial1.print("----------Rotary Sensor----------");
  Serial1.print("Current wind reading w.r.t Boat: ");
  Serial1.print(reading);
  Serial1.println();
  
  //get angle with respect to North
  float wind_wrtN = ((int)(reading + sensorData.boatDir))%360;

  Serial1.print("Current wind reading w.r.t North: ");
  Serial1.print(wind_wrtN);
  Serial1.println();

  float newSinWind = ( (sin(prevWindDirection*PI/180) + (0.125)*sin(reading*PI/180)) / (1.125) );
  float newCosWind = ( (cos(prevWindDirection*PI/180) + (0.125)*cos(reading*PI/180)) / (1.125) );
  wind_wrtN = atan2(newSinWind, newCosWind);
  wind_wrtN *= 180/PI;
  wind_wrtN = (wind_wrtN<0)?wind_wrtN+360:wind_wrtN;

  Serial1.print("Averaged Wind w.r.t North: ");
  Serial1.print(wind_wrtN);
  Serial1.println();
  
  sensorData.windDir = wind_wrtN;
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
    Serial1.println("IMU Stuck!");
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
  
  sensorData.boatDir = boatDir;
  sensorData.pitch  = (imu_data[0].fval)*(180/PI);
  sensorData.roll = (imu_data[2].fval)*(180/PI);
}



