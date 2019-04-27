/*
 * Code for scanning lidar sensor, collecting data, and comparing data arrays
*/
/*
 * Code To Add in the Future:
 * Incorporate IMU angles (first just add angle w.r.t North, then worry about yaw much later along with servo controls
 * Map to XY plane with boat nav algo

*/
#include "sensors.h"
#include "navigation.h"
#include "navigation_helper.h"
#include "coordinates.cpp"
#include <Servo.h>
#include <Wire.h>

Servo PanServo;  // create a servo object
Servo TiltServo;

float IMU_angle; // - angle of plane of Pan_angle - worry about sensorData.roll & pitch later
float boat_direction; //angle of boat w.r.t north = sensorData.boat_directionfloat Pan_angle; // - angle of pan servo
//int Tilt_angle; //- angle of tilt servo - replaced with Tilt_servo.read()
//int Pan_angle; //angle of pan servo - replaced with Pan_servo.read()
int reading = 0; //for reading data
//index corresponds to degree w.r.t north, element corresponds to distance
float Data1[360];
float Data2[360];
float Filtered_Data[360];
float pi = 3.14159;

 //get_current_angle returns the current angle (phi in spherical) that the sensor is pointing at
  int get_current_angle(){
    //return (Pan_angle);
    return PanServo.read();
    //return (Pan_Servo.read() + sensorData.boat_Direction + 360)%360; //return angle w.r.t north
 }

 //returns float of distance in meters from sensor
  float get_lidar_data() {
    Wire.requestFrom(0x66, 2);
    if (Wire.available()) {
      reading = Wire.read();
      reading = reading << 8;
      reading |= Wire.read();
      if (reading/100.0 > 130.0) {return 130;} //sometimes sensor returns 655.__ for some reason - ignore this data
      return reading/100.0;
    }else{
      Serial.println("Error: Sensor not available");
     }
  }

//returns 1 if passes "percent difference" check, 0 otherwise
int diff_check(float a, float b) {
   if (a == 130.0 || b == 130.0) {return 0;}
   float avg = (a+b)/2.0;
   float diff = abs(a-b);
   if (diff/(avg+.001) < 0.3) {return 1;}
   return 0;
}

//takes in an array of data and creates {x,y} coordinates for all points that aren't 130.0
//assumes the boat has not moved far since the data array was created
void create_xy_object_array(float data[360]) {
  coord_xy objects[360]; //max # of objects detected is 360, objects is an array whose first elements are objects, the rest just 0
  int j = 0;
  for (int i = 0; i < 360; i++){
    if (data[i] != 0 && data[i] != 360) {
      float y = sensorData.y + data[i]*cos(2.*pi*i/180.);
      float x = sensorData.x + data[i]*sin(2.*pi*i/180.);
      objects[j] = coord_xy({x,y});
      j++;
    }
  }
}

     //sweeps pan_servo from sweep_start to sweep_finish deg (relative to North) and stores lidar data in data_sweep arrays have 360 elements
     //Precondition: sweep_start > 10 and sweep_finish < 170 and sweep_start < sweep_finish
  void sweep_get_data(int sweep_start, int sweep_finish, float data_sweep1[], float data_sweep2[]){
    float pan_start = (sweep_start - (int)sensorData.boat_direction + 360) % 360;
    float pan_finish = (sweep_finish - (int)sensorData.boat_direction + 360) % 360;
    for(int i = pan_start; i < pan_finish; i+= 3){
      PanServo.write(i);
      delay(500);
      data_sweep1[get_current_angle()] = get_lidar_data();
      }
      delay(500);
      float pan_angle = pan_start;
      PanServo.write(pan_start);
      delay(1000);
    for(int i = pan_start; i < pan_finish; i+= 3){
      PanServo.write(i);
      delay(500);
      data_sweep2[get_current_angle()] = get_lidar_data();
      }
      delay(500);
     PanServo.write(pan_start);
     delay(1000);
   }

//returns an array, length 360, with a distance in indices where data is confirmed
//currently compares 3 closest degrees
//uses diff_check(a b) 
void compare(float data1[360], float data2[360]){  
  for (int i = 0; i<360; i++) {
    if (i == 0) {                      //check degree 0
      if (diff_check(data2[359], data1[i])) {Filtered_Data[i] = (data2[359]+data1[0])/2.0;}
      for (int j = 0; j<=1; j++) {
        if (diff_check(data2[i+j], data1[i])) {Filtered_Data[i] = (data2[i+j]+data1[i])/2.0;} 
      }
      if (Filtered_Data[i] == 0){Filtered_Data[i]=130.0;}
    }
    else if (i == 359) {              //check degree 359
      for (int j = -1; j<=0; j++) {
        if (diff_check(data2[i+j], data1[i])) {Filtered_Data[i] = (data2[i+j]+data1[i])/2.0;} 
      }
      if (diff_check(data2[0], data1[i])) {Filtered_Data[i] = (data2[0]+data1[i])/2.0;} 
      if (Filtered_Data[i] == 0){Filtered_Data[i]=130.0;}
    }
    else {                            //check all other degrees
      for (int j = -1; j<=1; j++) {
        if (diff_check(data2[i+j], data1[i])) {Filtered_Data[i] = (data2[i+j]+data1[i])/2.0;} 
      }
     if (Filtered_Data[i] == 0){Filtered_Data[i]=130.0;}
    }
  }
}

void init_obstacle() {
  PanServo.attach(4); // attaches the servo on pin 4 to the servo object
  PanServo.write(45); //begin pointing 45 deg to the left of boat's heading
  IMU_angle = 0.;
  Wire.begin();
  Serial.begin(9600); // open a serial connection to your computer
}

void loop_obstacle(){
  Wire.beginTransmission(0x66);
  Wire.write(0);
  Wire.endTransmission(); //Try moving this wire stuff to setup and see if it still works

// TODO: Define Data1 and Data2 before attempting to use them, make sure that you are passing in ALL ARGUMENTS into sweep_get_data
//  sweep_get_data(Data1, Data2); //sweep sensor and record data in two arrays
//  compare(Data1, Data2); // compare two arrays
  
  Serial.println("Printing Array");
  for(int i = 0; i<360;i++){
    if(Data1[i] != 0.0) {
      Serial.print(Filtered_Data[i]);
      Serial.print(" D1: ");
      Serial.print(Data1[i]);
      Serial.print(" D2: ");
      Serial.print(Data2[i]);
      Serial.print("  Angle: ");
      Serial.println(i);
    }
  }
  Serial.println("End printed Array");
  delay(10000);
  
    // wait for the servo to get there
    delay(15);
 }
