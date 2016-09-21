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
float IMUCorrection = -17.1; // -21.18; // -0.47; // -0.139; // 0.04; // -12.6; // -21.180 + 7.25;
 
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
  // Arjan pouring his bowl of Serial
  Serial.begin(9600);
  Serial2.begin(9600);
  
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
  sensorData.windDir = pos;
  
  //print angle to the screen
  Serial.println("\n --- Rotary Sensor ---");
  Serial.print("Wind direction wrt boat: "); Serial.println(sensorData.windDir);
  Serial.print("Wind  direction wrt North: "); 
  Serial.println(((int)(sensorData.windDir + sensorData.boatDir))%360);
}


void sGPS(void) {
  Serial.println("\n  ----- GPS -----");
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
    sensorData.longi = gps.location.lng();
    sensorData.lati = gps.location.lat();
    sensorData.dateTime.year = gps.date.year();
    sensorData.dateTime.month = gps.date.month();
    sensorData.dateTime.day = gps.date.day();
    sensorData.dateTime.hour = gps.time.hour();       // Hour (0-23) (u8)
    sensorData.dateTime.minute = gps.time.minute();   // Minute (0-59) (u8)
    sensorData.dateTime.seconds = gps.time.second();  // Second (0-59) (u8)
    // sensorData.boatDir = gps.course.deg();
  }  

//    Serial.println(gps.date.year());
//    Serial.println(sensorData.dateTime.year);
  
    Serial.print("Latitude                      "); Serial.println(sensorData.lati, 6);
    Serial.print("Longitude                     "); Serial.println(sensorData.longi, 6);
    Serial.print("Course in degrees             "); Serial.println(gps.course.deg());
//    Serial.print("Course in degrees             "); Serial.println(sensorData.boatDir);
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
  Serial.println("\n  ----- IMU -----");
  SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0 ));

  // Clear the internal data buffer on the IMU
  byte result = transferByte(0x01);
      Serial.print("Cleared internal buffer. Result: "),Serial.println(result);

  // Send start of packet:
  result = transferByte(0xF6);
       Serial.print("Send start of packet. Result: "),Serial.println(result);
  
  // Send command (tared euler angles)
  result = transferByte(0x01);
       Serial.print("Send commmand 0x01. Result: "),Serial.println(result);
  
  // Get status of device:
  result = transferByte(0xFF);
       Serial.print("Status of device. Result: "),Serial.println(result);

  while (result != 0x01) {  // Repeat until device is Ready
    delay(1);
    result = transferByte(0xFF);
    Serial.print("Status of device. Result: "),Serial.println(result);
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
  
  //Serial.print("pitch: "), Serial.println(sensorData.pitch,4);
  Serial.print("yaw:   "), Serial.print(sensorData.boatDir,4);  Serial.println("  <-- boat direction");
  //Serial.print("roll:  "), Serial.println(sensorData.roll,4);



  // delay(3000);
}

