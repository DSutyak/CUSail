
#include "sensors.h"
#include "navigation_helper.h"
#include "coordinates.cpp"

// updates recentObjects array with new data
void scanLidar();

//prints Data arrays with corresponding angles
void printLidarData();

//sweeps pan_servo from sweep_start to sweep_finish deg (relative to North) and stores lidar data in data_sweep arrays have 360 elements
     //Precondition: sweep_start > 10 and sweep_finish < 170 and sweep_start < sweep_finish
  void sweep_get_data(int sweep_start, int sweep_finish, float data_sweep1[], float data_sweep2[]);

   //get_current_angle returns the current angle (phi in spherical) that the sensor is pointing at
  int get_current_angle();

   //returns float of distance in meters from sensor
  float get_lidar_data();

  
//returns an array, length 360, with a distance in indices where data is confirmed
//currently compares 3 closest degrees
//uses diff_check(a b) 
void compare(float data1[360], float data2[360]);

//returns 1 if passes "percent difference" check, 0 otherwise
int diff_check(float a, float b);

//takes in an array of data and creates {x,y} coordinates for all points that aren't 130.0
//assumes the boat has not moved far since the data array was created
//should be rewritten as a list with the correct number of elements
void create_xy_object_array(float data[180]);

#ifndef obstacle_h
#define obstacle_h
class obstacle {
public:
  _coord_xy location;
  int seen; //the number of times we see it
  bool confirmed;

  void seen_again() {
    seen++;
  }
  void confirm() {
    confirmed = 1;
  }
};
#endif
