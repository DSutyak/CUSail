//void setup() {
//  Serial.begin(9600);
//  Serial.println("Vane Value\tDirection\tHeading");
//}

int readVane(int offset){ //offset must be less than 360 deg
  int rawIn;
  int dir;
  int calDir; 
  int state; 
  rawIn = analogRead(A1); //arduino has a 0-1023, 10 bit ADC converter 
  dir = map(rawIn, 0, 1023, 0, 360);
  calDir = dir + offset; 

  if(calDir >= 0) 
    {calDir = calDir%360;}

  else{
    calDir = calDir + 360; 
    }  

  if(dir < 22){
  Serial.println("N"); 
  state = 1; }
  else if (dir < 67) 
  {Serial.println("NE"); 
  state = 2;} 
  else if (dir < 112) 
  {Serial.println("E"); 
  state = 3;}
  else if (dir < 157) 
  {Serial.println("SE"); 
  state = 4; }
  else if (dir < 212) 
  {Serial.println("S"); 
  state = 5; }
  else if (dir < 247) 
  {Serial.println("SW"); 
  state = 6; }
  else if (dir < 292) 
  {Serial.println("W"); 
  state = 7; }
  else if (dir < 337) 
  {Serial.println("NW"); 
  state = 8; }
  else 
  {Serial.println("N");
  state = 9;}
} 
  
  
  
