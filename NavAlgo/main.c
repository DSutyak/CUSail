/*
 * Test cases for sensors
 */

// clock and protoThreads configure
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"

// sensor libraries
#include "sensors.h"
#include "delay.h"
#include "navigation_helper.c"
#include "coordinates.h"

// threads
static struct pt pt_sensor;

// sensor data
extern data_t* sensorData;

coord_xy waypoints[];

void delay_ms(unsigned int ms) {
    delay_us(ms * 1000);
}

void delay_us(unsigned int us) {
    us *= sys_clock / 1000000 / 2;
    _CP0_SET_COUNT(0); // Set Core Timer count to 0
    while (us > _CP0_GET_COUNT());
}

// thread to check sensor values every 2.5s
static PT_THREAD (protothread_timer(struct pt *pt)) {
    PT_BEGIN(pt);
    while(1) {
        readIMU();
        readAnemometer();
        PT_YIELD_TIME_msec(2500); // yield time for 2.5 seconds
    }
    PT_END(pt); // this will never execute
}


void main(void) { 
    initSensors();
  
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();

    // setup system wide interrupts
    INTEnableSystemMultiVectoredInt();

    // initialize the threads
    PT_INIT(&pt_sensor);
  
    // round-robin scheduler for threads
    while (1){
        PT_SCHEDULE(protothread_timer(&pt_sensor));
    }
  }

double calculateSailAngle() {
    coord_xy boatPosition = {sensorData->x, sensorData->y};
    coord_xy targetPosition = find_closest_waypoint(boatPosition, waypoints);
    double boat_heading = sensorData->boat_direction;
    double windDirection = sensorData->wind_dir; // should probably use true wind
    double intendedAngle = angleToTarget(boatPosition, targetPosition);
    double angleDifference = ((((windDirection - intendedAngle) % 360) + 360) % 360); // finds positive angle between wind and intended path
    double hysteresis = 0; // replace this
}

int shouldUpdateAngles () {
    return 1;
}