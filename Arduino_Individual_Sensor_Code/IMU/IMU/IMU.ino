  /*YEI 3 Space Sensor
  Measures Roll, Pitch and Yaw

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
       CSN  pin A11 (or any other chip select pin) 
 */

#include <SPI.h>

SPISettings settings(6000000, MSBFIRST, SPI_MODE0 ); //variable to hold SPI settings
//Set Slave Select, MOSI, MISO, CLK 
const int CSN = 65;
const int SO = 74;
const int SI = 75;
const int CLK = 76;
int magDeclination = 0;
 
// Needed to convert the bytes from SPI to float
union u_types {
    byte b[4];
    float fval;
} data[3];  // Create 3 unions, one for each euler angle

void setup() {
  //Set Pin Modes
  pinMode(CSN, OUTPUT);
  pinMode(SI, OUTPUT);
  pinMode(SO, INPUT);
  pinMode(CLK, OUTPUT);
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN, HIGH);
  //Initialize SPI 
  SPI.begin();
  //pour a bowl of serial
  Serial.begin(9600);
  Serial1.begin(9600);
}

/*function to transfer commands through SPI*/
byte transferByte(byte byteToWrite) {
  byte Result = 0x00;
  digitalWrite(CSN,LOW);
  delay(1);
  Result = SPI.transfer(byteToWrite);
  delay(1);
  digitalWrite(CSN,HIGH);
  return Result; 
}

/*function to swap endian */
void endianSwap(byte temp[4]) {
  byte myTemp = temp[0];
  temp[0] = temp[3];
  temp[3] = myTemp;
  myTemp = temp[1];
  temp[1] = temp[2];
  temp[2] = myTemp;
}

void loop() {

  digitalWrite(36, HIGH);
  delay(1000);
  digitalWrite(35, HIGH);
  delay(1000);
  digitalWrite(22, HIGH);

  SPI.beginTransaction(settings);
  

  // Clear the internal data buffer on the IMU
  byte result = transferByte(0x01);

  // Send start of packet:
  result = transferByte(0xF6);
  
  // Send command (tared euler angles)
  result = transferByte(0x01);
  
  // Get status of device:
  result = transferByte(0xFF);
  Serial.println (result);
  Serial1.println (result);

  while (result != 0x01) {  // Repeat until device is Ready
    delay(1);
    result = transferByte(0xFF);  
    Serial.println(result);
    Serial1.println("Stuck");}

  
  // Get the 12 bytes of return data from the device:
  for (int ii=0; ii<3; ii++) {
    for (int jj=0; jj<4; jj++) {
      data[ii].b[jj] =  transferByte(0xFF);
      delay(1);
    }
  }  

  SPI.endTransaction();

  //change the endian of the output
  for( int mm=0; mm<3; mm++) {
    endianSwap(data[mm].b);
  }
  
  float yaw = (data[1].fval)*(180/PI) + magDeclination; //yaw angle with respect to North in degrees
  
  if (yaw<0){ //convert to 0.. 360 scale
    yaw += 360;
  }
  
  float pitch  = (data[0].fval)*(180/PI); //pitch angle in radians
  float roll = (data[2].fval)*(180/PI); //roll angle in radians

  //Serial.print("Pitch (-90 to 90):"), Serial.println(pitch, 4); //Uncomment to display
  Serial.print("Yaw:"), Serial.println(yaw, 4);
  Serial1.print("Yaw:"), Serial1.println(yaw, 4);
  //Serial.print("Roll (-180 to 180):"), Serial.println(roll, 4); //Uncommment to display
}
