#include "sensors.h"
#include <SPI.h>
#include "TinyGPS++.h"

/*AMS Rotary Sensor AS5147
 Measures absolute angle position referenced at a set NORTH

 Circuit
 UNO: MOSI pin 11
      MISO pin 12 
      CLK  pin 13
      CSN  pin 10

 Mega: MOSI pin 51
       MISO pin 50    
       CLK  pin 52
       CSN  pin 53  
 */

float IMUCorrection = 0; 

// TO DO: replace 10 with variable, determine best value from testing 
float windData[10];
int numCallRS;     
int RScount;

//Set Slave Select Pin
//MOSI, MISO, CLK are handeled automatically
//int CSN = 10;
//int SO = 74;
//int SI = 75;
//int CLK = 76 ; 
unsigned int angle;

//Set Slave Select Pin
//MOSI, MISO, CLK are handeled automatically
//const int CSN = 4;
//const int SO = 74;
//const int SI = 75;
//const int CLK = 76;

//SPISettings settings(6000000, MSBFIRST, SPI_MODE0 );

// initialize gps
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

//function to swap endian
void endianSwap(byte temp[4]) {
  byte myTemp = temp[0];
  temp[0] = temp[3];
  temp[3] = myTemp;
  myTemp = temp[1];
  temp[1] = temp[2];
  temp[2] = myTemp;
}

// initializes sensors
void initSensors(void) {
  Serial.begin(9600);
  Serial2.begin(9600);
  // set the data rate for the SoftwareSerial port
  Serial1.begin( 9600 );
  Serial3.begin( 9600 );
  
  // initialize data structure
  sensorData = *(data_t*) malloc(sizeof(data_t));
  sensorData = {}; // {NULL}; 

  //Set Pin Modes
  pinMode(RS_CSN, OUTPUT);
  pinMode(IMU_CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(RS_CSN, HIGH);
  digitalWrite(IMU_CSN, HIGH);
  //Initialize SPI 
  SPI.begin();
  
  // set up XBees: 
  //Serial.begin(9600);
  Serial.println( "Arduino started sending bytes via XBee" );
  Serial1.println( "Arduino started sending bytes via XBee" );

  numCallRS = 0;     
  RScount = 0;
  sensorData.windDir = 0;

}

// uses rotary sensor to update wind direction
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
  //SPI.endTransaction();

  //mask the MSB and 14th bit
  angle = (angle & (0x3FFF));

  //covert to a 360 degree scale
  int pos = ( (unsigned long) angle)*360UL/16384UL;
  //sensorData.windDir = pos;
  
  int wind_wrtN = ((int)(pos + sensorData.boatDir))%360;

  // TO DO: error -- average will give 180 if wind hovers between 0 and 359; need avg to wrap around
  //    looked up how to average circular data; need to implement 
  
  // TO DO: figure out good number to average wind direction over (currently 10)
  
  // average wind over past 10 function calls 
  if (numCallRS >= 10) {
    // sensorData.windDir currently stores average of previous 10 
    // we want (sensorData.windDir * 10) - irrelevant data + latest reading / 10
    sensorData.windDir = ((sensorData.windDir * 10) - windData[RScount] + wind_wrtN) / 10; 
    
    // replace oldest data with current data 
    windData[RScount] = wind_wrtN;
    // increment count 
    RScount = (RScount + 1) % 10;

    
  }
  else { 
    // if less than 10, average over available wind data
    sensorData.windDir = (sensorData.windDir * numCallRS + wind_wrtN) / (numCallRS + 1);
    // save current reading 
    windData[numCallRS] = wind_wrtN; 
    // increment number of calls 
    numCallRS++;
  }

  float avgWind = sensorData.windDir;
  sensorData.windDir = wind_wrtN;   // currently not using average function
  
  //print angle to the screen
  Serial.println("\n --- Rotary Sensor ---");   Serial1.println("\n --- Rotary Sensor ---");
  Serial.print("Current reading: "); Serial.println(pos);
  Serial.print("Current angle wrt N: "); Serial.println(wind_wrtN);
  Serial1.print("Current reading: "); Serial1.println(pos);
  Serial1.print("Current angle wrt N: "); Serial1.println(wind_wrtN);
  Serial.print("Wind direction (averaged, needs fixing): "); Serial.println(avgWind);
  Serial1.print("Wind direction (averaged, needs fixing): "); Serial1.println(avgWind);
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
    sensorData.boatDir = gps.course.deg();
  }  

//    Serial.println(gps.date.year());
//    Serial.println(sensorData.dateTime.year);
  
    Serial.print("Latitude                      "); Serial.println(sensorData.lati, 6);
    Serial.print("Longitude                     "); Serial.println(sensorData.longi, 6);
    Serial.print("Course in degrees             "); Serial.println(sensorData.boatDir);
    Serial.print("Speed (mps)                   "); Serial.println(gps.speed.mps());
//    Serial.print("Hours                         "); Serial.println(sensorData.dateTime.hour); 
//    Serial.print("Minutes                       "); Serial.println(sensorData.dateTime.minute); 
//    Serial.print("Seconds                       "); Serial.println(sensorData.dateTime.seconds); 
//    Serial.print("Centiseconds                  "); Serial.println(gps.time.centisecond());     // 100ths of a second (0-99) (u8)
    Serial.print("Time (hr:min:sec)             "); Serial.print(sensorData.dateTime.hour);    Serial.print(":"); 
                                                    Serial.print(sensorData.dateTime.minute);  Serial.print(":");
                                                    Serial.print(sensorData.dateTime.seconds); Serial.print(":");
                                                    Serial.println(gps.time.centisecond());      // 100ths of a second (0-99) (u8)
//    Serial.print("Date                          "); Serial.println(gps.date.value());           // Raw date in DDMMYY format (u32)
    Serial.print("Date                          "); Serial.print(sensorData.dateTime.month); Serial.print("/"); 
                                                    Serial.print(sensorData.dateTime.day);   Serial.print("/");
                                                    Serial.println(sensorData.dateTime.year);
    Serial.print("Number of Satelites in use    "); Serial.println(gps.satellites.value());     // Number of satellites in use (u32)
    // Serial.println();
    
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
  
//  int count = 0;
//  //GPS.wakeup();
//  while (count < GPSRep) {
//    while (!GPS.newNMEAreceived()) {
//      char c = GPS.read();
//    }
//    //Serial.println(GPS.lastNMEA());
//    if (GPS.parse(GPS.lastNMEA()))
//      count++;
//  }
//  //GPS.standby();
//  sensorData.dateTime.year = GPS.year;
//  sensorData.dateTime.month = GPS.month;
//  sensorData.dateTime.day = GPS.day;
//  sensorData.dateTime.hour = GPS.hour;
//  sensorData.dateTime.minute = GPS.minute;
//  sensorData.dateTime.seconds = GPS.seconds;
//  if (GPS.fix) {
//    sensorData.longi = GPS.longitude;
//    sensorData.lati = GPS.latitude;
//  }
//  Serial.print("longitude: ");
//  Serial.print(sensorData.longi,6);
//  Serial.print(", latitude: ");
//  Serial.print(sensorData.lati,6);
//  Serial.print(", year: ");
//  Serial.print(sensorData.dateTime.year);
//  Serial.print(", month: ");
//  Serial.print(sensorData.dateTime.month);
//  Serial.print(", day: ");
//  Serial.print(sensorData.dateTime.day);
//  Serial.print(", hour: ");
//  Serial.print(sensorData.dateTime.hour);
//  Serial.print(", minute: ");
//  Serial.print(sensorData.dateTime.minute);
//  Serial.print(", seconds: ");
//  Serial.println(sensorData.dateTime.seconds);
//  Serial.println();
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
  
  
//  Serial.print("pitch: "), Serial.println(imu_data[0].fval);
//  Serial.print("yaw:   "), Serial.print(imu_data[1].fval);  Serial.println("  <-- boat direction");
//  Serial.print("roll:  "), Serial.println(imu_data[2].fval);


//  sensorData.boatDir = imu_data[1].fval; //* (180 / PI); // - declination;
//  sensorData.roll = imu_data[2].fval * (180 / PI);

  sensorData.boatDir =  ((imu_data[1].fval)*(180/PI)) + IMUCorrection;   // yaw
  //sensorData.boatDir = (imu_data[1].fval + IMUCorrection) * (180 / PI);
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



  // delay(3000);
}

