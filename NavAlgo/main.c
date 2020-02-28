/*
 * Main Navigation Algorithm
 * 
 * Initialize sensors and spawn threads to run algorithm
 */

#include "config.h" // clock and protoThreads configure
#include "pt_cornell_1_2_1.h" // threading library
#include <stdlib.h>

#include "sensors.h"
#include "servo.h"
#include "navigation_helper.h"
#include "coordinates.h"
#include "radio.h"
#include "navtest.h"
#include "delay.h"


// string buffer
char buffer[60];

// sensor data
extern data_t* sensorData;

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_sail;

// delay function implementation (needs config file)
void delay_ms(unsigned int ms) {
    delay_us(ms * 1000);
}

void delay_us(unsigned int us) {
    us *= sys_clock / 1000000 / 2;
    _CP0_SET_COUNT(0); // Set Core Timer count to 0
    while (us > _CP0_GET_COUNT());
}

// === Sail Thread =================================================
// read from IMU, Anemometer, then calculate servo angles
static PT_THREAD (protothread_sail(struct pt *pt))
{
    PT_BEGIN(pt);
    navigationInit();
    while(1) {
        // yield time 2 seconds - this is the quickest we can transmit uart
        PT_YIELD_TIME_msec(2000);
        
        // test transmit to "radio"
        //static char buffR[20];
        //sprintf(buffR, "uptime: %d\n", sensorData->sec);
        //transmitString(buffR);
        
        nav();
    } // END WHILE(1)
    PT_END(pt);
} // sail thread

// === Main  ======================================================
// Initialize sensors, servos, radio, schedule threads, then exit
void main(void) {
  ANSELA = 0; ANSELB = 0; 
    
  // === config threads ==========
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_sail);
  
  initSensors();
  initServos();
  initRadio();
 
  // Timer 1 interrupt for up time
  CloseTimer1();
  OpenTimer1( T1_ON | T1_PS_1_8 | T1_SOURCE_INT, 0x1387);
  ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_7);
  mT1SetIntPriority(7);
  mT1ClearIntFlag();
  mT1IntEnable(1);
  
  // test bench
  transmitString("Starting test...\n");
  testAllOff();
  //while(1);
  //testIMUOn();
  //testGPSOn();
  //testAnemometerOn();
  //testEverythingOn();
  //testLidarOn();
  
  // round-robin scheduler for threads
//  while (1){
//      PT_SCHEDULE(protothread_sail(&pt_sail));
//  }
} // main

// === end  ======================================================