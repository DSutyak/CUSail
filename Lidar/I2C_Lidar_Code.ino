//This is the fixed lidar-Arduino I2C code
//Ensure in lidar settings, Baud Rate is 9600, and Additional Protocol is set to "I2C OM13518"

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(9600);

}

int reading = 0;

void loop() {
  Wire.beginTransmission(0x66);
  Wire.write(0);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom(0x66, 2);

  if (Wire.available()) {
    reading = Wire.read();
    reading = reading << 8;
    reading |= Wire.read();
    Serial.println(reading);
  }

  //delay(250);

}
