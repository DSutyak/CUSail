#include <Servo.h>
Servo pan;
Servo tilt;

int pos_p = 0;
int pos_t = 90;
bool increasing = true;

void setup() {

  Serial.begin(9600);
  pan.attach(4);
  tilt.attach(3);
  pan.write(pos_p);
  tilt.write(pos_t);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Start");
  for(pos_p = 0; pos_p < 180; pos_p += 1)  // goes from 0 degrees to 180 degrees
  {                                  // in steps of 1 degree
    Serial.println(pos_p);
    pan.write(pos_p);              // tell servo to go to position in variable 'pos'
    delay(20);                       // adjust speed (smaller delay = faster)
      
  }
  Serial.println("Stop");
  pos_t = 89;
  tilt.write(pos_t);
  Serial.println(pos_t);
  /*
  if(increasing){
    tilt.write(++pos_t);
    if(pos_t == 45){
      increasing = false;
    }
  }
  else{
    tilt.write(--pos_t);
    if (pos_t == 0){
      increasing = true;
    }
  }
  */
  for(pos_p = 180; pos_p > 0; pos_p -= 1)    // goes from 180 degrees to 0 degrees
  {                               
    Serial.println(pos_p);
    pan.write(pos_p);              // tell servo to go to position in variable 'pos'
    delay(20);                       // adjust speed (smaller delay = faster)
  }

  
  Serial.println("Stop2");
  pos_t = 90;
  tilt.write(pos_t);
  Serial.println(pos_t);
}
  /*
  if(increasing){
    tilt.write(++pos_t);
    if(pos_t == 45){
      increasing = false;
    }
  }
  else{
    tilt.write(--pos_t);
    if (pos_t == 0){
      increasing = true;
    }
  }
  */
    /*
   for(pos_t = 0; pos_t < 90; pos_t += 1)  // goes from 0 degrees to 180 degrees
  {                                  // in steps of 1 degree
    tilt.write(pos_t);              // tell servo to go to position in variable 'pos'
    delay(15);                       // adjust speed (smaller delay = faster)
  }
  for(pos_t = 90; pos_t>=1; pos_t-=1)     // goes from 180 degrees to 0 degrees
  {                               
    tilt.write(pos_t);              // tell servo to go to position in variable 'pos'
    delay(15);      // adjust speed (smaller delay = faster)
  }*/

