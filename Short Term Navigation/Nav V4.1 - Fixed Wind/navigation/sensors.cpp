#include "sensors.h"
#include <SPI.h>
#include "TinyGPS++.h"
 
float windData[10];
int numCallRS;     
int RScount;
unsigned int angle;
TinyGPSPlus gps;

// initialize data structure
data_t sensorData;

// Needed to convert the bytes from SPI to float
union u_types {
    byte b[4];
    float fval;
} imu_data[3];  // Create 3 unions, one for each euler angle

//function to transfer commands through SPI
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
 * Precondition: Sail Angle in 0.. 360
 */
double sailMap(double sailAngle){
  double newSailAngle;
  if (sailAngle <= 90){
    newSailAngle = map(sailAngle, 0, 90, 71, 91.5);
  }
  else if (sailAngle <= 180){
    newSailAngle = map(sailAngle, 90, 180, 91.5, 110.5);
  }
  else if (sailAngle <= 270){
    newSailAngle = map(sailAngle, 180, 270, 110.5, 128);
  }
  else{
   newSailAngle = map(sailAngle, 270, 360, 128, 146.5);
  }
  return newSailAngle;
}

/* Returns servo command tail servo for inputted sail angle and tail angle 
 * Precondition: Sail Angle in 0.. 360, Tail Angle in -180.. 180
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

/* Initializes sensors */
void initSensors(void) {
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial1.begin(9600);
  Serial3.begin(9600);
  
  // initialize data structure
  sensorData = *(data_t*) malloc(sizeof(data_t));
  sensorData = {}; // {NULL}; 

  //Set Pin Modes
  pinMode(RS_CSN, OUTPUT);
  pinMode(IMU_CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  //Set Slave Select signals High i.e disable chips
  digitalWrite(RS_CSN, HIGH);
  digitalWrite(IMU_CSN, HIGH);
  //Initialize SPI 
  SPI.begin();

  numCallRS = 0;     
  RScount = 0;
  sensorData.windDir = 0;
}

/* uses rotary sensor to update wind direction */
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
  angle = SPI.transfer16(0xC000);
  digitalWrite(RS_CSN, HIGH);
  SPI.endTransaction();

  //mask the MSB and 14th bit
  angle = (angle & (0x3FFF));

  //covert to a 360 degree scale
  int pos = ( (unsigned long) angle)*360UL/16384UL;

  //get angle with respect to North
  int wind_wrtN = ((int)(pos + sensorData.boatDir))%360;

  //sensorData.windDir = wind_wrtN;
  sensorData.windDir = 270;
  sensorData.boatDir = 350;

  //Averaging the wind reading (Incomplete!)
  
//
//  // TO DO: error -- average will give 180 if wind hovers between 0 and 359; need avg to wrap around
//  //    looked up how to average circular data; need to implement 
//  
//  // average wind over past 10 function calls 
//  if (numCallRS >= 10) {
//    // sensorData.windDir currently stores average of previous 10 
//    // we want (sensorData.windDir * 10) - irrelevant data + latest reading / 10
//    sensorData.windDir = ((sensorData.windDir * 10) - windData[RScount] + wind_wrtN) / 10; 
//    
//    // replace oldest data with current data 
//    windData[RScount] = wind_wrtN;
//    // increment count 
//    RScount = (RScount + 1) % 10;
//    
//  }
//  else { 
//    // if less than 10, average over available wind data
//    sensorData.windDir = (sensorData.windDir * numCallRS + wind_wrtN) / (numCallRS + 1);
//    // save current reading 
//    windData[numCallRS] = wind_wrtN; 
//    // increment number of calls 
//    numCallRS++;
//  }
//
//  float avgWind = sensorData.windDir;
  
  //print angle to the screen
  Serial.println("\n --- Rotary Sensor ---");   Serial1.println("\n --- Rotary Sensor ---");
  Serial1.println("\n --- Rotary Sensor ---");   Serial1.println("\n --- Rotary Sensor ---");
  
  Serial.print("Current reading: "); Serial.println(pos);
  Serial.print("Current angle wrt N: "); Serial.println(sensorData.windDir);
  
  Serial1.print("Current reading: "); Serial1.println(pos);
  Serial1.print("Current angle wrt N: "); Serial1.println(sensorData.windDir);

  Serial.println("\n --- Boat Direction ---");   Serial1.println("\n ---Boat Direction ---");
  Serial1.println("\n --- Boat Direction ---");   Serial1.println("\n --- Boat Direction ---");

  Serial.print("yaw:   "), Serial.print(sensorData.boatDir,4);  Serial.println("  <-- boat direction");
  Serial1.print("yaw:   "), Serial1.print(sensorData.boatDir,4);  Serial1.println("  <-- boat direction");

  
  
}


void sGPS(void) {
  Serial.println("\n  ----- GPS -----");   Serial1.println("\n  ----- GPS -----");
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
    sensorData.boatDir = gps.course.deg();}  

  
    Serial.print("Latitude                      "); Serial.println(sensorData.lati, 6);
    Serial.print("Longitude                     "); Serial.println(sensorData.longi, 6);
    Serial.print("Course in degrees             "); Serial.println(sensorData.boatDir);
    Serial.print("Speed (mps)                   "); Serial.println(gps.speed.mps());
    Serial.print("Time (hr:min:sec)             "); Serial.print(sensorData.dateTime.hour);    Serial.print(":"); 
                                                    Serial.print(sensorData.dateTime.minute);  Serial.print(":");
                                                    Serial.print(sensorData.dateTime.seconds); Serial.print(":");
                                                    Serial.println(gps.time.centisecond());      // 100ths of a second (0-99) (u8)
    Serial.print("Date                          "); Serial.print(sensorData.dateTime.month); Serial.print("/"); 
                                                    Serial.print(sensorData.dateTime.day);   Serial.print("/");
                                                    Serial.println(sensorData.dateTime.year);
    Serial.print("Number of Satelites in use    "); Serial.println(gps.satellites.value());     // Number of satellites in use (u32)
    
    Serial1.print("Latitude                      "); Serial1.println(sensorData.lati, 6);
    Serial1.print("Longitude                     "); Serial1.println(sensorData.longi, 6);
    Serial1.print("Course in degrees             "); Serial1.println(sensorData.boatDir);
    Serial1.print("Speed (mps)                   "); Serial1.println(gps.speed.mps());
    Serial1.print("Time (hr:min:sec)             "); Serial1.print(sensorData.dateTime.hour);    Serial1.print(":"); 
                                                     Serial1.print(sensorData.dateTime.minute);  Serial1.print(":");
                                                     Serial1.print(sensorData.dateTime.seconds); Serial1.print(":");
                                                     Serial1.println(gps.time.centisecond());      // 100ths of a second (0-99) (u8)
    Serial1.print("Date                          "); Serial1.print(sensorData.dateTime.month); Serial1.print("/"); 
                                                    Serial1.print(sensorData.dateTime.day);   Serial1.print("/");
                                                    Serial1.println(sensorData.dateTime.year);
    Serial1.print("Number of Satelites in use    "); Serial1.println(gps.satellites.value());     // Number of satellites in use (u32)
}

void sIMU(void) {
  Serial.println("\n  ----- IMU -----"); Serial1.println("\n  ----- IMU -----");
  SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0 ));

  // Clear the internal data buffer on the IMU
  byte result = transferByte(0x01);
      Serial.print("Cleared internal buffer. Result: "),Serial.println(result);
      Serial1.print("Cleared internal buffer. Result: "),Serial1.println(result);

  // Send start of packet:
  result = transferByte(0xF6);
       Serial.print("Send start of packet. Result: "),Serial.println(result);
       Serial1.print("Send start of packet. Result: "),Serial1.println(result);
  
  
  // Send command (tared euler angles)
  result = transferByte(0x01);
       Serial.print("Send commmand 0x01. Result: "),Serial.println(result);
       Serial1.print("Send commmand 0x01. Result: "),Serial1.println(result);
  
  // Get status of device:
  result = transferByte(0xFF);
       Serial.print("Status of device. Result: "),Serial.println(result);
       Serial1.print("Status of device. Result: "),Serial1.println(result);

  while (result != 0x01) {  // Repeat until device is Ready
    delay(1);
    result = transferByte(0xFF);
    Serial.print("Status of device. Result: "),Serial.println(result);
    Serial1.print("Status of device. Result: "),Serial1.println(result);
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

  sensorData.boatDir =  ((imu_data[1].fval)*(180/PI));   // yaw
  if (sensorData.boatDir < 0) {
    sensorData.boatDir += 360; 
  }

  sensorData.pitch  = (imu_data[0].fval)*(180/PI);
  sensorData.roll = (imu_data[2].fval)*(180/PI);
  
  Serial.print("pitch: "), Serial.println(sensorData.pitch,4);
  Serial.print("yaw:   "), Serial.print(sensorData.boatDir,4);  Serial.println("  <-- boat direction");
  Serial.print("roll:  "), Serial.println(sensorData.roll,4);
  
  Serial1.print("pitch: "), Serial1.println(sensorData.pitch,4);
  Serial1.print("yaw:   "), Serial1.print(sensorData.boatDir,4);  Serial1.println("  <-- boat direction");
  Serial1.print("roll:  "), Serial1.println(sensorData.roll,4);

}

