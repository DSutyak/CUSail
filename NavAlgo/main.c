/*
 * Main Navigation Algorithm
 * 
 * Initialize sensors and spawn threads to run algorithm
 */

#include "config.h" // clock and protoThreads configure
#include "pt_cornell_1_2_1.h" // threading library

////////////////////////////////////
// graphics libraries (testing only)
// SPI channel 1 connections to TFT
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>
////////////////////////////////////

#include "sensors.h"
#include "servo.h"
#include "navigation_helper.h"
#include "coordinates.h"
#include "radio.h"

#include "test.h"


// string buffer
char buffer[60];

// sensor data
extern data_t* sensorData;
extern int numPulses;

////////////////////////////////////
// DAC ISR
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000
//== Timer 2 interrupt handler ===========================================
volatile unsigned int DAC_data ;// output value
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2; // 20 MHz max speed for this DAC

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) Timer2Handler(void)
{
    mT2ClearIntFlag();
    
    // generate  ramp
    // at 100 ksample/sec, 2^12 samples (4096) on one ramp up
    // yields 100000/4096 = 24.4 Hz.
     DAC_data = (DAC_data + 1) & 0xfff ; // for testing
    
    // CS low to start transaction
     mPORTBClearBits(BIT_4); // start transaction
    // test for ready
     while (TxBufFullSPI2());
     // write to spi2
     WriteSPI2(DAC_config_chan_A | DAC_data);
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
     mPORTBSetBits(BIT_4); // end transaction
}

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer;

// system 1 second interval tick
int sys_time_seconds ;

//TESTING
int testAngle = 0;

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);
     tft_setCursor(0, 0);
     tft_setTextColor(ILI9340_GREEN);  tft_setTextSize(1);
     //tft_writeString("Current Wind Angle:\n");
     //tft_writeString("Current Wind Speed:\n");
     //tft_writeString("Pulses so far:\n");
     tft_writeString("Testing Angle:\n");
     //tft_writeString("LiDAR Distance:\n");
     //tft_writeString("Pitch, Roll, Yaw:\n");
     // set up LED to blink
//     mPORTASetBits(BIT_0 );	//Clear bits to ensure light is off.
//     mPORTASetPinsDigitalOut(BIT_0 );    //Set port as output
     navigationInit();
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(2000);
        // draw sys_time
        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        //readAnemometer();
        //sprintf(buffer,"%d", (int) sensorData->wind_dir);
        //sprintf(buffer,"%f", sensorData->wind_speed);
        //sprintf(buffer,"%d", (int) numPulses);
        //sprintf(buffer,"%2d, %.2f, %.2f", sensorData->sec, sensorData->roll, sensorData->boat_direction);
        testServo(testAngle);
        sprintf(buffer,"%d", (int) testAngle);
        testAngle = (testAngle + 1) % 180;
//        float distance = readLIDAR();
//        sprintf(buffer,"%f\n", distance);
        tft_writeString(buffer);
        //nav();
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Main  ======================================================
void main(void) {
 //SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; 

  // set up DAC on big board
  // timer interrupt //////////////////////////
    // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
    // at 30 MHz PB clock 60 counts is two microsec
    // 400 is 100 ksamples/sec
    // 2000 is 20 ksamp/sec
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 400);

    // set up the timer interrupt with a priority of 2
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag

    // SCK2 is pin 26 
    // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
    PPSOutput(2, RPB5, SDO2);

    // control CS for DAC
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);

    // divide Fpb by 2, configure the I/O ports. Not using SS in this example
    // 16 bit transfer CKP=1 CKE=1
    // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
    // For any given peripherial, you will need to match these
    // clk divider set to 2 for 20 MHz
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 8);
  // end DAC setup
    
  // === config threads ==========
  // turns OFF UART support and debugger pin, unless defines are set
  PT_setup();

  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);

  // init the display
  // NOTE that this init assumes SPI channel 1 connections
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  // seed random color
  srand(1);
  
  initSensors();
  init_servos();
  
  // Timer 1 interrupt for up time
  CloseTimer1();
  OpenTimer1( T1_ON | T1_PS_1_8 | T1_SOURCE_INT, 0x1387);
  ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_7);
  mT1SetIntPriority(7);
  mT1ClearIntFlag();
  mT1IntEnable(1);
  
  // round-robin scheduler for threads
  while (1){
      PT_SCHEDULE(protothread_timer(&pt_timer));
      }
  } // main

// === end  ======================================================