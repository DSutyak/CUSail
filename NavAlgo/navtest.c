/*
 * Nav tests: 
 */

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
#include "tft_master.h"
//#include "main.c"

uint32_t clock = 40000000;

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
    OpenI2C1( I2C_ON, 0x0C2);
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
    
        //Initialize GPS
    // define setup Configuration 1 for OpenUARTx
		// Module Enable 
		// Work in IDLE mode 
		// Communication through usual pins 
		// Disable wake-up 
		// Loop back disabled 
		// Input to Capture module from ICx pin 
		// no parity 8 bit 
		// 1 stop bit 
		// IRDA encoder and decoder disabled 
		// CTS and RTS pins are disabled 
		// UxRX idle state is '1' 
		// 16x baud clock - normal speed
	#define config1 	UART_EN | UART_IDLE_CON | UART_RX_TX | UART_DIS_WAKE | UART_DIS_LOOPBACK | UART_DIS_ABAUD | UART_NO_PAR_8BIT | UART_1STOPBIT | UART_IRDA_DIS | UART_DIS_BCLK_CTS_RTS| UART_NORMAL_RX | UART_BRGH_SIXTEEN
	
	// define setup Configuration 2 for OpenUARTx
		// IrDA encoded UxTX idle state is '0'
		// Enable UxRX pin
		// Disable UxTX pin
		// Interrupt on transfer of every character to TSR 
		// Interrupt on every char received
		// Disable 9-bit address detect
		// Rx Buffer Over run status bit clear
	#define config2		UART_TX_PIN_LOW | UART_RX_ENABLE | UART_TX_DISABLE | UART_INT_TX | UART_INT_RX_CHAR | UART_ADR_DETECT_DIS | UART_RX_OVERRUN_CLEAR

	// Open UART1 with config1 and config2
	OpenUART1( config1, config2, clock/16/9600-1);
		
	// Configure UART1 RX Interrupt with priority 2
    IFS1bits.U1EIF = 0;
    IFS1bits.U1RXIF = 0;
    IFS1bits.U1TXIF = 0;
    IPC8bits.U1IP = 2;
    IPC8bits.U1IS = 2;
    IEC1bits.U1RXIE = 1;
    IEC1bits.U1TXIE = 0;
    
	//ConfigIntUART1(UART_INT_PR2 | UART_RX_INT_EN);
    PPSInput(3, U1RX, RPA4);
    
    initSensorData(); // initializing data as 0s, servos, nav algo, gps
    initServos();
    navigationInit();
    
    /*testing gps code*/
    sensorData->boat_direction=60;
    sensorData->wind_speed=5;
    sensorData->wind_dir=30;
    
    while(1) {
        //PT_YIELD_TIME_msec(2000);
        checkSentence();
        nav();
    }
}

void testAnemometerOn() {
    
    initSensorData(); // initializing data as 0s, servos, nav algo, anemometer
    initServos();
    navigationInit();
    ANSELA = 0; ANSELB = 0; // set A as input
    
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