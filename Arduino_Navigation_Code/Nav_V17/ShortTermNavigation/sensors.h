
/*-----------------------------------------------------------------
 CU Sail
 Cornell University Autonomous Sailboat Team

 Sensors
 Gathers boat direction w.r.t North, wind direction w.r.t North, boat pitch, boat yaw,
 date, time and boat position in lattitude and longitude, using an IMU, a GPS and a Rotary Sensor

 Code has been tested and run on an Arduino Due

 Rotary Sensor:  AS5047 by AMS (AS5147 has been tested to work)
 GPS: PAM-7Q GPS Module by Parallax
 IMU: 3 Space Embedded Evaluation Kit by Yost Labs
--------------------------------------------------------------------*/
#include <Servo.h>
#include <Wire.h>

/*------------------------------ Structures ------------------------------*/
#ifndef date_h
#define date_h
typedef struct date {
  int year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned char seconds;
} date_t;

typedef struct dataStructure {
  float boat_direction; //Boat direction w.r.t North
  float sailAngleBoat; //Sail angle for use of finding wind wrt boat
  float tailAngleBoat; //Tail angle for use of finding wind wrt boat
  float pitch;
  float roll;
  float wind_dir; // Wind direction w.r.t North
  double x;
  double y;
  double lat;
  double longi;
  date_t dateTime; // Current date and time, of type date_t
} data_t;
#endif

/*--------------------------- Predefined Variables ---------------------------*/
#define RS_CSN          52
#define IMU_CSN         65
#define SO              74
#define SI              75
#define CLK             76
#define orangeLED       34 //Saffron
#define redLED          35
#define greenLED        36

/*--------------------------- Global Variables ---------------------------*/
extern data_t sensorData; //Defines the boat's state, in type data_t

extern double objectVals[2];//vector for detected objects

extern Servo PanServo;
extern Servo TiltServo;
/*------------------------------ Functions ------------------------------*/

/*Sensor setup*/
void initSensors(void);

void lightAllLEDs();

void lowAllLEDs();

/*Sets value of sensorData.windDir to current wind direction w.r.t North*/
void sRSensor(void);

/*Sets value of sensorData.lati, sensorData.longi and sensorData.dateTime
* to current lattitude, current longitude and current date/time respectively*/
void sGPS(void);

/*Sets value of sensorData.boatDir, sensorData.pitch and sensorData.roll
* to current boat direction w.r.t North, current boat pitch and current boat roll*/
void sIMU(void);

/* 
 */
void sLidar(void);

/*Returns servo command tail servo for inputted sail angle and tail angle
* Precondition: Sail Angle in 0.. 360, Tail Angle in -180.. 180, w.r.t BOAT*/
double tailMap(double sailAngle, double tailAngle);

/*Returns servo command for sail servo for inputted sail angle
* Precondition: Sail Angle in 0.. 360 w.r.t BOAT*/
double sailMap(double sailAngle);


/*Returns servo command for sail servo for inputted sail angle
* Precondition: Sail Angle in 0.. 360 w.r.t BOAT. To be used on the testbench*/
double sailMapBench( double sailAngle);

/*Returns servo command tail servo for inputted sail angle and tail angle
* Precondition: Sail Angle in 0.. 360, Tail Angle in -180.. 180, w.r.t BOAT. To be used on the testbench */
double tailMapBench( double sailAngle, double tailAngle);


void addObjects(void);
