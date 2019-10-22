/*
 * Test cases for sensors
 */

// clock and protoThreads configure
#include "config.h"
// threading library
#include "pt_cornell_1_2_1.h"

// sensor libraries
#include "sensors.h"

// threads
static struct pt pt_sensor;

// sensor data
extern data_t* sensorData;

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