/*
 * Nav tests: 
 */

#define _SUPPRESS_PLIB_WARNING // removes outdated plib warning
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <stdlib.h>
#include "sensors.h"
#include "servo.h"
#include "navigation_helper.h"
#include "coordinates.h"
#include "radio.h"

#include <plib.h> // peripheral library
#include "xc.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "delay.h"
//#include "main.c"

uint32_t clock = 40000000;

void testAllOff() {
    
    initSensors(); // initializing data as 0s, servos, nav algo
    IEC1bits.U1RXIE = 0;
    mINT0IntEnable(0);
    
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
    initSensors();
    IEC1bits.U1RXIE = 0;
    mINT0IntEnable(0);
    
    initServos();
    navigationInit();
    
    /*testing gps code*/
    sensorData->wind_dir = 30;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;
    
    while(1) {
        delay_ms(2000);
        readIMU();
        nav();
    }    
}

void testGPSOn() {
    initSensors(); // initializing data as 0s, servos, nav algo, gps
    mINT0IntEnable(0);
    initServos();
    navigationInit();
    
    /*testing gps code*/
    sensorData->boat_direction=60;
    sensorData->wind_speed=5;
    sensorData->wind_dir=30;
    
    while(1) {
        delay_ms(2000);
        checkSentence();
        nav();
    }
}

void testAnemometerOn() {
    
    initSensors(); // initializing data as 0s, servos, nav algo, anemometer
    IEC1bits.U1RXIE = 0;
    initServos();
    navigationInit();
    
    /*testing anemometer code*/
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=0; 
    nav();      // test case 1

    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=60; 
    nav();      // test case 2
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=120; 
    nav();      // test case 3
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=180; 
    nav();      // test case 4
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=240;
    nav();      // test case 5
        
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=300;
    nav();      // test case 6
    
    
    while(1) {
        delay_ms(2000);
        readAnemometer();
        nav();
    }
}

void testEverythingOn() {
    initSensors();
    initServos();
    navigationInit();
    
    while(1) {
        delay_ms(2000);
        nav();
    }
}

void testLidarOn() {
    
} // obstacle detection unimplemented