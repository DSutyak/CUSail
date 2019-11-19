/* ************************************************************************** */
// Initialize and read from sensors
/* ************************************************************************** */

// structs for storing the sensor data
#ifndef sensor_h
#define sensor_h
typedef struct date {
  int year;
  int month;
  int day;
  int hour;
  int minute;
  int seconds;
} date_t;

typedef struct dataStructure {
  float boat_direction; //Boat direction w.r.t North (IMU)
  float sailAngleBoat; //Sail angle for use of finding wind wrt boat
  float tailAngleBoat; //Tail angle for use of finding wind wrt boat
  float pitch; // (IMU)
  float roll; // (IMU)
  float wind_dir; // Wind direction w.r.t North
  float wind_speed;
  double x;
  double y;
  double lat;
  double longi;
  int msec;
  int sec;
  int min;
  int hour;
} data_t;

extern data_t* sensorData; //Defines the boat's state, in type data_t

void initSensors(void);

void readIMU(void);

void readAnemometer(void);

#endif