/*
 * Nav tests: 
 */

#include <stdlib.h>
#include "sensors.h"
#include "servo.h"
#include "navigation_helper.h"
#include "coordinates.h"
#include "radio.h"
//#include "main.c"

void testAllOff() {
    
    initSensorData(); // initializing data as 0s, servos, nav algo
    initServos();
    navigationInit();
    
    sensorData->boat_direction = 30; 
    sensorData->wind_dir = 0;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;

    nav(); // run test case 1
    
    sensorData->boat_direction = 0; 
    sensorData->wind_dir = 270;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444254;
    sensorData->longi=-76.48266;

    nav(); // run test case 2
    
    sensorData->boat_direction = 90; 
    sensorData->wind_dir = 0;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;

    nav(); // run test case 3
    
    sensorData->boat_direction = 30; 
    sensorData->wind_dir = 90;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;

    nav(); // run test case 4
    
    sensorData->boat_direction = 60; 
    sensorData->wind_dir = 30;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;

    nav(); // run test case 5
}

void testIMUOn() {
    initSensorData(); // initializing data as 0s, servos, nav algo, gps
    initServos();
    navigationInit();
    //OpenI2C1( I2C_ON, 65);
    
    /*testing gps code*/
    sensorData->wind_dir = 30;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;
    
    while(1) {
        //PT_YIELD_TIME_msec(2000); // wont compile, use delay instead
        readIMU();
        nav();
    }    
}

void testGPSOn() {
    initSensorData(); // initializing data as 0s, servos, nav algo, gps
    initServos();
    navigationInit();
    
    /*testing gps code*/
    sensorData->boat_direction=60;
    sensorData->wind_speed=5;
    sensorData->wind_dir=30;
    
    while(1) {
        //PT_YIELD_TIME_msec(2000);
        nav();
    }
}

void testAnemometerOn() {
    
    initSensorData(); // initializing data as 0s, servos, nav algo, anemometer
    initServos();
    navigationInit();
    
    /*testing anemometer code*/
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=0; // test case 1
    nav();

    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=60; // test case 2
    nav();
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=120; // test case 3
    nav();
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=180; // test case 4
    nav();
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=240; // test case 5
    nav();
        
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=300; // test case 6
    nav();
    
    
    while(1) {
        //PT_YIELD_TIME_msec(2000);
        readAnemometer();
        nav();
    }
}

void testEverythingOn() {
    main();
}

void testLidarOn() {
    
} // obstacle detection unimplemented

void initSensorData () {
    sensorData = (data_t*) malloc(sizeof(data_t));
    sensorData->boat_direction = 0; //Boat direction w.r.t North
    sensorData->sailAngleBoat = 0; //Sail angle for use of finding wind wrt N
    sensorData->tailAngleBoat = 0; //Tail angle for use of finding wind wrt N
    sensorData->pitch = 0;
    sensorData->roll = 0;
    sensorData->wind_dir = 0; // Wind direction w.r.t North
    sensorData->wind_speed = 0;
    sensorData->x = 0; // X-coord of current global position;
    sensorData->y = 0; // Y-coord of current global position;
    sensorData->lat=0;
    sensorData->longi=0;
    sensorData->msec = 0;
    sensorData->sec = 0;
    sensorData->min = 0;
    sensorData->hour = 0;
}