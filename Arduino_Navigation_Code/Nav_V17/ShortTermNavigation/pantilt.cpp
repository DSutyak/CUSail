#include "sensors.h"
#include <Servo.h>
#include <Math.h>




Servo pan;
Servo tilt;

//int pos_p = 0;
//int pos_t = 180;
//bool increasing = true;

void setup() {
  Serial.begin(9600);
  pan.attach(4);
  tilt.attach(3);
  pan.write(0);
  tilt.write(180);
}
/*
void getAngle(){
  //angle of nutation?
  pos_t = acos(cos(sensorData.pitch)*cos(sensorData.roll));
  tilt.write(pos_t);
}

void loop() {
  //double a = sensorData.pitch;
  //Serial.println(acos(cos(sensorData.pitch)*cos(sensorData.roll)));
  getAngle();
}
  // put your main code here, to run repeatedly:
  //Serial.println("Start");
  
  /*
  for(pos_p = 0; pos_p < 180; pos_p += 1)  // goes from 0 degrees to 180 degrees
  {                                  // in steps of 1 degree
    Serial.println(pos_p);
    pan.write(pos_p);              // tell servo to go to position in variable 'pos'
    delay(20);                       // adjust speed (smaller delay = faster)
      
  }
  Serial.println("Stop");
  getAngle();
  Serial.println(pos_t);

  for(pos_p = 180; pos_p > 0; pos_p -= 1)    // goes from 180 degrees to 0 degrees
  {                               
    Serial.println(pos_p);
    pan.write(pos_p);              // tell servo to go to position in variable 'pos'
    delay(20);                       // adjust speed (smaller delay = faster)
  }

  
  Serial.println("Stop2");
  getAngle();
  Serial.println(pos_t);
}*/

