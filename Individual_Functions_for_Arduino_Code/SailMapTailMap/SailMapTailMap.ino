 #include <Servo.h>;
Servo Sail;
Servo Tail;

/* Used to calibrate and test sail and tail servos, using their respective servo 
 *  mappings. Uncomment tests as required.*/

int sailPin = 9; //Sail Servo SERVO 1
int tailPin = 8; //Tail Servo SERVO 2


void setup() {

 
 Sail.attach(sailPin);
 Tail.attach(tailPin); 
 //pour a bowl of Serial
 Serial.begin(9600);
 
}


void loop() {

//  Sail.write(sailMapBench(0)); //Sail to 0
//  //delay(5000);
//  
//  Tail.write(tailMapBench(0, -30));
//  delay(5000);  
//  
//  Tail.write(tailMapBench(0, 0));
//  delay(5000);
//
//  Tail.write(tailMapBench(0, 30));
//  delay(5000);

//  Sail.write(sailMapBench(90)); //Sail to 0
//  delay(5000);
//  
//  Tail.write(tailMapBench(90, 60));
//  delay(5000);  
//  
//  Tail.write(tailMapBench(90, 90));
//  delay(5000);
//
//  Tail.write(tailMapBench(90, 120));
//  delay(5000);
//
//  Sail.write(sailMapBench(180)); //Sail to 180
//  delay(5000);
//  
//  Tail.write(tailMapBench(180, 150));
//  delay(5000);  
//  
//  Tail.write(tailMapBench(180, 180));
//  delay(5000);
//
//  Tail.write(tailMapBench(180, -150));
//  delay(5000);
//
//  Sail.write(sailMapBench(270)); //Sail to 270
//  delay(5000);
//
//  Tail.write(tailMapBench(270, -120));
//  delay(5000);  
//  
//  Tail.write(tailMapBench(270, -90));
//  delay(5000);
//
//  Tail.write(tailMapBench(270, -60));
//  delay(5000);

//  Sail.write(sailMapBench(360)); //Sail to 270
//  delay(5000);

/*Tail and Sail full test for wrt sail*/
//
//  Sail.write(sailMap(0)); //Sail to 0
//  delay(5000);
//
//  Tail.write(tailMap(0, -30));
//  delay(5000);  
//  
//  Tail.write(tailMap(0, 0));
//  delay(5000);
//
//  Tail.write(tailMap(0, 30));
//  delay(5000);
//
//  Sail.write(sailMap(90)); //Sail to 90
//  delay(5000);
//  
//  Tail.write(tailMap(90, 60));
//  delay(5000);  
//  
//  Tail.write(tailMap(90, 90));
//  delay(5000);
//
//  Tail.write(tailMap(90, 120));
//  delay(5000);
//
//  Sail.write(sailMap(180)); //Sail to 180
//  delay(5000);
//  
//  Tail.write(tailMap(180, 150));
//  delay(5000);  
//  
//  Tail.write(tailMap(180, 180));
//  delay(5000);
//
//  Tail.write(tailMap(180, -150));
//  delay(5000);
//
//  Sail.write(sailMap(270)); //Sail to 270
//  delay(5000);
//
//  Tail.write(tailMap(270, -120));
//  delay(5000);  
//  
//  Tail.write(tailMap(270, -90));
//  delay(5000);
//
//  Tail.write(tailMap(270, -60));
//  delay(5000)
}

/*-------------------Sail and Tail Mapping Functions---------------------*/


/* Returns servo command for sail servo for inputted sail angle 
 * Precondition: Sail Angle in 0.. 360
 */
double sailMap(double sailAngle){
  double newSailAngle;
  
  if (sailAngle <= 90){
    newSailAngle = map(sailAngle, 0, 90, 142, 125.5);
  }
  else if (sailAngle <= 180){
    newSailAngle = map(sailAngle, 90, 180, 125.5, 109);
  }
  else if (sailAngle <= 270){
    newSailAngle = map(sailAngle, 180, 270, 109.5, 91);
  }
  else{
   newSailAngle = map(sailAngle, 270, 360,91.5, 74.5);
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
    newTailAngle=map(newTailAngle,-30,0,130,100); 
  }
  
  else if (newTailAngle > 0 ){
    newTailAngle=map(newTailAngle,0,30,100,70); 
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

