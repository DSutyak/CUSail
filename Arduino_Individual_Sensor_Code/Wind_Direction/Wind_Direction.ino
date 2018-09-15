void setup() {
  Serial.begin(9600);
  Serial.println("Vane Value\tDirection\tHeading");
}
void loop(){
  readVane(0);
}

void readVane(int offset){ //offset must be less than 360 deg
  int rawIn;
  int dir;
  int calDir; 
  int state; 
  rawIn = analogRead(A8); //arduino has a 0-1023, 10 bit ADC converter 
  //Serial.println(rawIn);
  dir = map(rawIn, 0, 1023, 0, 360);
  calDir = dir + offset; 

  if(calDir >= 0) 
    {calDir = calDir%360;}

  else{
    calDir = calDir + 360; 
    }  

Serial.println(calDir);
} 
