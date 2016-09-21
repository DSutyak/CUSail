// -----------------------------------------------------------------
// Cornell Autonomous Sail Boat Team
// Author: Eric T. J. Jung, Brian Gross, Varun Shanbhag, David Brown
//
// Implements a library of functions for gathering data from sensors
// made changes
// -----------------------------------------------------------------
#ifndef __SENSORS_H__
#define __SENSORS_H__

#include <SPI.h>

/*------------------------------ Structures ------------------------------*/
typedef struct date {
  //unsigned char year;
  int year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned char seconds;
} date_t;

typedef struct dataStructure {
  float boatDir;
  float pitch;
  float roll;
  float windSpeed;
  float windDir;
  float longi;
  float lati;
  //float baroPress;
  date_t dateTime;
} data_t;

/*--------------------------- Predefined Variables ---------------------------*/
//#define potPin          A0
#define RS_CSN            10 // 53
#define IMU_CSN           4
//#define RS_SO             74 // 50
//#define RS_SI             75 // 51
//#define RS_CLK            76 // 52
#define SO                74
#define SI                75
#define CLK               76 
//#define anePin          49
#define debLimit        60
//#define anemoRep        3
//#define potenRep        5
//#define potenThreshold  30
//#define GPSRep          4


/*--------------------------- Global Variables ---------------------------*/
extern data_t sensorData;


/*------------------------------ Functions ------------------------------*/
// initializes sensors
void initSensors(void);

// uses potentiometer to update wind direction
void sRSensor(void);

// uses GPS to update current location
void sGPS(void);

// uses IMU to update the bearing of the boat
void sIMU(void);


#endif /* __SENSORS_H__ */
