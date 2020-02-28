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

uint32_t clock = 40000000;

void testAllOff() {
    transmitString("Disabling Interrupts...\n");
    IEC1bits.U1RXIE = 0;
    mINT0IntEnable(0);
    
    transmitString("Initializing Navigation...\n");
    navigationInit();
    
    sensorData->boat_direction = 30; 
    sensorData->wind_dir = 0;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    convertLLtoXY();

    transmitString("Test Case 1: ");
    nav(); // run test case 1
    
    sensorData->boat_direction = 0; 
    sensorData->wind_dir = 270;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444254;
    sensorData->longi=-76.48266;
    convertLLtoXY();

    transmitString("Test Case 2: ");
    nav(); // run test case 2
    
    sensorData->boat_direction = 90; 
    sensorData->wind_dir = 0;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;
    convertLLtoXY();

    transmitString("Test Case 3: ");
    nav(); // run test case 3
    
    sensorData->boat_direction = 30; 
    sensorData->wind_dir = 90;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;
    convertLLtoXY();

    transmitString("Test Case 4: ");
    nav(); // run test case 4
    
    sensorData->boat_direction = 60; 
    sensorData->wind_dir = 30;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;
    convertLLtoXY();

    transmitString("Test Case 5: ");
    nav(); // run test case 5
    
    transmitString("Finished test cases...\n");
    while(1) {
        delay_ms(2000); // idle forever
    }
}

void testIMUOn() {
    transmitString("Disabling Interrupts...\n");
    IEC1bits.U1RXIE = 0;
    mINT0IntEnable(0);
    
    transmitString("Initializing Navigation...\n");
    navigationInit();
    
    /*testing imu code*/
    sensorData->wind_dir = 30;
    sensorData->wind_speed = 5;
    sensorData->lat=42.444612;
    sensorData->longi=-76.483492;
    convertLLtoXY();
    
    while(1) {
        delay_ms(2000);
        readIMU();
        
        char buffer[80];
        sprintf(buffer, "roll: %f, pitch: %f, yaw: %f\n", sensorData->roll, sensorData->pitch, sensorData->boat_direction);
        transmitString(buffer);
        
        nav();
    }    
}

void testGPSOn() {
    transmitString("Disabling Interrupts...\n");
    mINT0IntEnable(0);
    
    transmitString("Initializing Navigation...\n");
    navigationInit();
    
    /*testing gps code*/
    sensorData->boat_direction=60;
    sensorData->wind_speed=5;
    sensorData->wind_dir=30;
    
    while(1) {
        delay_ms(2000);
        
        char buffer[80];
        sprintf(buffer, "lat: %f, long: %f\n", sensorData->lat, sensorData->longi);
        transmitString(buffer);
        
        nav();
    }
}

void testAnemometerOn() {
    transmitString("Disabling Interrupts...\n");
    IEC1bits.U1RXIE = 0;
    mINT0IntEnable(0);
    
    transmitString("Initializing Navigation...\n");
    navigationInit();
    
    /*testing anemometer code*/
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=0; 
    convertLLtoXY();
    
    transmitString("Test Case 1: ");
    nav();      // test case 1

    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=60; 
    convertLLtoXY();
    
    transmitString("Test Case 2: ");
    nav();      // test case 2
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=120; 
    convertLLtoXY();
    
    transmitString("Test Case 3: ");
    nav();      // test case 3
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=180; 
    convertLLtoXY();
    
    transmitString("Test Case 4: ");
    nav();      // test case 4
    
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=240;
    convertLLtoXY();
    
    transmitString("Test Case 5: ");
    nav();      // test case 5
        
    sensorData->lat=42.444259;
    sensorData->longi=-76.484435;
    sensorData->boat_direction=0;
    sensorData->wind_speed=5;
    sensorData->wind_dir=300;
    convertLLtoXY();
    
    transmitString("Test Case 6: ");
    nav();      // test case 6
    
    transmitString("Enabling Speed Interrupt...\n");
    mINT0IntEnable(1);
    
    while(1) {
        delay_ms(2000);
        readAnemometer();
        
        char buffer[80];
        sprintf(buffer, "direction: %f, speed %f\n", sensorData->wind_dir, sensorData->wind_speed);
        transmitString(buffer);
        
        nav();
    }
}

void testEverythingOn() {
    transmitString("Initializing Navigation...\n");
    navigationInit();
    
    while(1) {
        delay_ms(2000);
        readIMU();
        readAnemometer();
        
        char buffer[200];
        sprintf(buffer, "pitch: %f, roll: %f, yaw: %f\nlat: %f, long: %f\ndir: %f, speed: %f\n", sensorData->pitch, sensorData->roll, sensorData->boat_direction, sensorData->lat, sensorData->longi, sensorData->wind_dir, sensorData->wind_speed);
        transmitString(buffer);
        
        nav();
    }
}

void testLidarOn() {
    while(1) {
        delay_ms(2000);
        
        float distance = readLIDAR();
        char buffer[60];
        sprintf(buffer, "distance: %f\n", distance);
        transmitString(buffer);
    }
} // obstacle detection unimplemented