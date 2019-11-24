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
#include "navigation_helper.h"
#include "coordinates.h"
#include "servo.h"

#include <math.h>

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

volatile int high = 0;
void toggleLED(void);

// thread to check sensor values every 2.5s
static PT_THREAD (protothread_timer(struct pt *pt)) {
    PT_BEGIN(pt);
    ANSELAbits.ANSA0 = 0; // enable RB3 (pin 7) as digital
    TRISAbits.TRISA0 = 0; // enable RB3 (pin 7) as output
    PORTAbits.RA0 = 0; // set low
    while(1) {
        toggleLED();
        PT_YIELD_TIME_msec(1000); // yield time for 2.5 seconds
    }
    PT_END(pt); // this will never execute
}

void toggleLED(void) {
    if (high == 1) {
        high = 0;
        PORTAbits.RA0 = 0;
    } else {
        high = 1;
        PORTAbits.RA0 = 1;
    }
}

void blinkLED(void) {
    ANSELAbits.ANSA0 = 0; // enable RB3 (pin 7) as digital
    TRISAbits.TRISA0 = 0; // enable RB3 (pin 7) as output
    PORTAbits.RA0 = 0; // set low
    int high = 0;
    while(1) {
        if (high == 1) {
            high = 0;
            PORTAbits.RA0 = 0;
            delay_ms(1000);
        } else {
            high = 1;
            PORTAbits.RA0 = 1;
            delay_ms(1000);
        }
    }
}

void main(void) { 
//    blinkLED();
//    initSensors();
//  
    // turns OFF UART support and debugger pin, unless defines are set
    PT_setup();

    // setup system wide interrupts
    INTEnableSystemMultiVectoredInt();
    
    // setup timer 3 to update time every ms
    // Fpb = SYS_FREQ = 40Mhz
    // Timer Prescale = 4
    // PR3 = 0x270F = 9,999
    // 1 ms = (PR3 + 1) * TMR Prescale / Fpb = (9999 + 1) * 4 / 40000000
    CloseTimer3();
    OpenTimer3( T3_ON | T3_PS_1_4 | T3_SOURCE_INT, 0x270F);
    ConfigIntTimer3(T3_INT_ON | T3_INT_PRIOR_7);
    mT3SetIntPriority(7);
    mT3ClearIntFlag();
    mT3IntEnable(1);

    // initialize the threads
    PT_INIT(&pt_sensor);
  
    // round-robin scheduler for threads
    while (1){
        PT_SCHEDULE(protothread_timer(&pt_sensor));
    }
  }

double calculateAngle() {
    // initializing variables for calculations
    coord_xy boatPosition = {sensorData->x, sensorData->y}; // initialize B
    coord_xy targetPosition = find_closest_waypoint(boatPosition, waypoints); // initialize T
    coord_xy boatTargetDifference = diff (boatPosition, targetPosition); // initialize t
    double t_mag = xyDist(boatPosition, targetPosition); // initialize magnitude of t
    double boat_heading = sensorData->boat_direction; // initialize phi(b))
    double beating_param = 100; // this can be adjusted
    double windDirection = sensorData->wind_dir; // should probably use true wind
    double intendedAngle = angleToTarget(boatPosition, targetPosition);
    double angleDifference = (double)(((((int)(windDirection - intendedAngle)) % 360) + 360) % 360); // finds positive angle between wind and intended path
    double hysteresis = 1 + (beating_param / t_mag); // initialize n
    double inverseWindAngle = angleDifference; // TODO: initialize phi(-w)?
    double alpha = 0.0; 
    double v_maxR = 0.0;
    double v_maxL = 0.0;
    double phi_bmaxR = inverseWindAngle;
    double phi_bmaxL = inverseWindAngle;
    double v_hyp;
    double v_tR;
    double v_tL;
    double phi_bnew;
    double delta_alpha = 1.0; // can change this
    while (alpha < 180) {
        v_hyp = fPolar (sensorData->wind_speed, (inverseWindAngle + alpha));
        v_tR = v_hyp * cos(inverseWindAngle + alpha); // Is this right
        if (v_tR > v_maxR) {
            v_maxR = v_tR;
            phi_bmaxR = inverseWindAngle + alpha;
        }
        alpha = alpha + delta_alpha;
    }
    alpha = 0;
    while (alpha < 180) {
        v_hyp = fPolar (sensorData->wind_speed, (inverseWindAngle - alpha));
        v_tL = v_hyp * cos(inverseWindAngle - alpha); // Is this right part 2
        if (v_tL > v_maxL) {
            v_maxL = v_tL;
            phi_bmaxL = inverseWindAngle - alpha;
        }
        alpha = alpha + delta_alpha;
    }
    if (abs((int)(phi_bmaxR - boat_heading)) < abs((int)(phi_bmaxL - boat_heading))) {
        if(v_maxR * hysteresis < v_maxL) {
            phi_bnew = phi_bmaxL;
        }
        else {
            phi_bnew = phi_bmaxR;
        }
    }
    else {
        if(v_maxL * hysteresis < v_maxR) {
            phi_bnew = phi_bmaxR;
        }
        else {
            phi_bnew = phi_bmaxL;
        }
    }
    return phi_bnew;
}

int shouldUpdateAngles () {
    return 1;
}

/* Every 1ms, update timer */
void __ISR( _TIMER_3_VECTOR, ipl7) T3Interrupt(void) {
    sensorData->msec += 1;
    if (sensorData->msec > 1000) {
        sensorData->msec -= 1000;
        sensorData->sec += 1;
    }
    if (sensorData->sec > 60) {
        sensorData->sec -= 60;
        sensorData->min += 1;
    }
    if (sensorData->min > 60) {
        sensorData->min -= 60;
        sensorData->hour += 1;
    }
    mT3ClearIntFlag();
}