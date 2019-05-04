/*
 * Code for scanning lidar sensor, collecting data, and comparing data arrays
*/
/*
 * Code To Add in the Future:
 * Incorporate IMU angles (first just add angle w.r.t North, then worry about yaw much later along with servo controls
 * Map to XY plane with boat nav algo
 * 
 * //SETUP has been moved to function initSensors() sensors.cpp


*/
#include "obstacle_detection.h"
#include <Wire.h>


int reading = 0; //for reading data from lidar sensor
//Data array's index corresponds to degree w.r.t north, element corresponds to distance
float Data1[360];
float Data2[360];
float Filtered_Data[360];
coord_xy recentObjects[360]; //max # of objects detected is 180, recentObjects is an array whose first elements are objects, the rest just 0. 
//contains all objects detected from a single sweep data array
obstacle* object_list[360]; //contains pointers to all objects
int object_list_index = 0; //index is position where no objects have been created in array



//TODO add scanLidar to header file
// updates recentObjects array with new data
void scanLidar() {
  sweep_get_data(45, 135, Data1, Data2); //sweep sensor and record data in two arrays
  compare(Data1, Data2); // compare two arrays and store consistent data in global Filtered_Data
  printLidarData();
  create_xy_object_array(Filtered_Data); //update recentObjects array
 }

     //sweeps pan_servo from sweep_start to sweep_finish deg (relative to North) and stores lidar data in data_sweep arrays have 360 elements
     //Precondition: sweep_start > 10 and sweep_finish < 170 and sweep_start < sweep_finish
  void sweep_get_data(int sweep_start, int sweep_finish, float data_sweep1[], float data_sweep2[]){
    int pan_start = (int)(sweep_start - sensorData.boat_direction + 360) % 360;
    int pan_finish = (int)(sweep_finish - sensorData.boat_direction + 360) % 360;
    for(int i = pan_start; i < pan_finish; i+= 3){
      PanServo.write(i);
      delay(500);
      data_sweep1[get_current_angle()] = get_lidar_data();
      }
      delay(500);
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
    
 
 //get_current_angle returns the current angle (phi in spherical) that the sensor is pointing at
  int get_current_angle(){
    //return PanServo.read();
    return (int)(PanServo.read() + sensorData.boat_direction + 360)%360; //return angle w.r.t north 
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

//returns an array, length 360, with a distance in indices where data is confirmed
//currently compares 3 closest degrees ***This is a problem when the servo rotates by 3 degrees - only compares two closest data points***
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

//returns 1 if passes "percent difference" check, 0 otherwise
int diff_check(float a, float b) {
  float max_percent_difference = 0.3;
   if (a == 130.0 || b == 130.0) {return 0;}
   float avg = (a+b)/2.0;
   float diff = abs(a-b);
   if (diff/(avg+.001) < max_percent_difference) {return 1;}
   return 0;
}
//prints Data arrays with corresponding angles
void printLidarData(){
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
  }

//takes in an array of data and creates {x,y} coordinates for all points that aren't 130.0
//assumes the boat has not moved far since the data array was created
//should be rewritten as a list with the correct number of elements
void create_xy_object_array(float data[360]) {
  int j = 0;
  float pi = 3.14159265359; //is there a math library to use?
  for (int i = 0; i < 360; i++){
    if (data[i] != 0.0 && data[i] != 130.0) { //only add object if dist isn't 0 or 130. The 0 check is redundant because FilteredData should already have the data removed
      float y = sensorData.y + data[i]*cos(2.*pi*i/180.);
      float x = sensorData.x + data[i]*sin(2.*pi*i/180.);
      recentObjects[j] = coord_xy({x,y});
      j++;
    }
  }
  for(int k = j;k < 180 ;k++){ //less than size of recentObjects
    recentObjects[k] = coord_xy({0,0});
  }
}


//_______________________________________________________________________


//inserts objects into sorted global object_list
//should use sorting algo 
void add_to_object_list(coord_xy data[180]){
  int i = 0;
  while (data[i].x != 0 && i < 180){
    for(int j = 0; j< 360; j++) { //less than objectlist.length
      if (diff_check(data[i].x, object_list[j]->location.x) && (diff_check(data[i].y, object_list[j]->location.y))){ //if true then object confirmed
        object_list[j]->location = data[i]; //update location of confirmed object
      } else {
        obstacle* new_obj = new obstacle;
        new_obj->location = data[i];
        new_obj->seen=1;
        new_obj->confirmed = false;
        object_list[object_list_index] = new_obj;
        object_list_index ++;
        if(object_list_index >=360){
          object_list_index = 0; ///need better way to wrap around when fills object_list
        }
      }
    }
  }
}

