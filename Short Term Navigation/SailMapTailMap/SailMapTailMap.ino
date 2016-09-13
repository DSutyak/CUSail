 #include <Servo.h>;
Servo Sail;
Servo Tail;

/** Used to test sail and tail servos, using their respective servo 
 *  mappings. Uncomment tests as required.
*/

int sailPin = 9; //Sail Servo SERVO 1
int tailPin = 8; //Tail Servo SERVO 2


void setup() {

 
 Sail.attach(sailPin);
 Tail.attach(tailPin); 
 Serial.begin(9600);
 
}


void loop() {

/*Sail Basic Test*/
//
//  Sail.write(sailMap(0));
//  Tail.write(tailMap(0,0));
//  delay(10000);
//  Tail.write(tailMap(0,0));   
//  Sail.write(sailMap(90));
//  delay(5000);
//  Sail.write(sailMap(180));
//  delay(5000);
//  Sail.write(sailMap(270)); 
//  delay(5000);
//  Sail.write(sailMap(360));
//  delay(5000);


/*Sail Complex Test*/
//  for (double i = 0; i <= 360; i += 20){
//    Sail.write(sailMap(i));
//
//    delay(5000);
//  }

/*Tail Sweep Raw Servo Test*/
//  Sail.write(sailMap(0));
//  for(int i=54; i<=162; i++){
//    delay(50);
//  }
//
//  for(int i=162; i>=54; i--){
//    Tail.write(i);
//    delay(50);
//  }

  
/*Tail Non-Sweep Servo Test*/
//  Tail.write(38); //-30 degrees
//  delay(5000);
//  Tail.write(85); //0 degrees
//  delay(5000);
//  Tail.write(145); //30 degrees
//  delay(5000);

/*Tail and Sail full test for wrt sail*/

  Sail.write(sailMap(0)); //Sail to 0
  delay(5000);

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
//  delay(5000);


/*Sail-Tail Complex Test*/
//  for(int i=60;i<120;i++){
//    Sail.write(sailMap(i)); 
//    Tail.write(tailMap(i,90));
//    delay(70);
//  }
//  for(int i=120;i>60;i--){
//    Sail.write(sailMap(i));
//    Tail.write(tailMap(i,90));
//    delay(70);
//  }

}

/*-------------------Sail and Tail Mapping Functions---------------------*/


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

