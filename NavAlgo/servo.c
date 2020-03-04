/* ************************************************************************** */
/** Servo library
 *  Sail Servo - D954SW
 *  Tail Servo - Traxxas 2056
/* ************************************************************************** */
#define _SUPPRESS_PLIB_WARNING // removes outdated plib warning
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING

#include <plib.h>
#include <math.h>
#include "servo.h"
#include "sensors.h"
#include "delay.h"

#define MIN_SAIL_ANGLE 0
#define MAX_SAIL_ANGLE 148
#define MIN_SAIL_DUTY 2125 // 0.850 ms
#define MAX_SAIL_DUTY 5875 //2.35 ms

#define MIN_TAIL_ANGLE 0
#define MAX_TAIL_ANGLE 60
#define MIN_TAIL_DUTY 2145 // 0.858 ms
#define MAX_TAIL_DUTY 4175 // 1.67 ms

double p = 0.2; // proportional error for P control

void initServos(void) {
    OpenOC1(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Tail Servo
    OpenOC2(OC_ON | OC_TIMER3_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Sail Servo
    
    // Fpb = SYS_FREQ = 40Mhz
    // Timer Prescale = 16
    // 20 ms = (PR3 + 1) * TMR Prescale / Fpb = (49,999 + 1) * 16 / 40,000,000
    CloseTimer3();
    OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_16, 49999);
    
    // set PPS to configure pins (subject to change)
    PPSOutput(1, RPB4, OC1);    //OC1 is PPS Group 1, maps to RPB4
    PPSOutput(2, RPB5, OC2);    //OC2 is PPS Group 2, maps to RPB5
    
    // set servos to neutral (middle) angle
    SetDCOC1PWM((MIN_TAIL_DUTY + MAX_TAIL_DUTY)/2);
    SetDCOC2PWM((MIN_SAIL_DUTY + MAX_SAIL_DUTY)/2);
}

double map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return ((toHigh - toLow) * (value - fromLow)/(fromHigh-fromLow)) + toLow;
}

void testServo(int angle) {
    if (angle <= MAX_SAIL_ANGLE && angle >= MIN_SAIL_ANGLE) {
        SetDCOC2PWM((int) map(angle, MIN_SAIL_ANGLE, MAX_SAIL_ANGLE, MIN_SAIL_DUTY, MAX_SAIL_DUTY));
    }
    
    if (angle <= MAX_TAIL_ANGLE && angle >= MIN_TAIL_ANGLE) {
        SetDCOC1PWM((int) map(angle, MIN_TAIL_ANGLE, MAX_TAIL_ANGLE, MIN_TAIL_DUTY, MAX_TAIL_DUTY));
    }
}

/* Returns servo command tail servo for inputted sail angle and tail angle
 * Precondition: Sail Angle in [0, 74] or [286, 360] w.r.t boat, 
 * Tail Angle in -180.. 180 w.r.t boat
 * Only tail angles within [-30, 30] are actually attainable
 */
double setTailServoAngle(double sail_angle, double tail_angle) {
    double s_angle = sail_angle;
    if (s_angle > 180){ //convert sail angle to -180.. 180
        s_angle -= 360;
    }
    
    double newTailAngle = tail_angle - s_angle; //calculate position of tail with respect to sail
    
    //make sure tail angle is in range -180.. 180
    if (newTailAngle < -180) {
        newTailAngle += 360;
    } else if (newTailAngle > 180) {
        newTailAngle -= 360;
    }
    
    if (newTailAngle < -30.0) {
        newTailAngle = -30.0;
    } else if (newTailAngle > 30.0) {
        newTailAngle = 30.0;
    }
    
    int actualTailAngle = (int) map(newTailAngle, -30, 30, MAX_TAIL_ANGLE, MIN_TAIL_ANGLE);
    SetDCOC1PWM((int) map(actualTailAngle, MIN_TAIL_ANGLE, MAX_TAIL_ANGLE, MIN_TAIL_DUTY, MAX_TAIL_DUTY));

    return newTailAngle + s_angle; // return to wrt boat
}

/* 
 * Sets the PWM of the sail servo
 */
double setSailPWM(double angle) {
    if (angle >= 180 && angle < 286) {
        SetDCOC2PWM(MAX_SAIL_DUTY);
        return 286;
    } else if (angle < 180 && angle > 74) {
        SetDCOC2PWM(MIN_SAIL_DUTY);
    } else if (angle >= 0 && angle <= 74) {
        int newAngle = (int) map(angle, 0, 74, 74, 0); // wrt servo frame
        SetDCOC2PWM((int) map(newAngle, 0, 148, MIN_SAIL_DUTY, MAX_SAIL_DUTY));
        return angle;
    } else {
        int newAngle = (int) map(angle, 286, 360, 148, 74);
        SetDCOC2PWM((int) map(newAngle, 0, 148, MIN_SAIL_DUTY, MAX_SAIL_DUTY));
        return angle;
    }
}

/* 
 * Updates servo position for inputted sail angle
 * Precondition: Sail Angle in 0.. 360 w.r.t boat
 * Only angles within [0, 74] or [286, 360] are attainable
 * Returns the angle that was actually set
 */
double setSailServoAngle(double angle) {
    double setAngle = setSailPWM(angle);
    double realAngle = readEncoder();
    double error = setAngle - realAngle;
    int attempts = 10;
    
    // get within 2 degrees of where you want to be, or give up after 10 attempts
    while (attempts > 0 && fabs(error) > 2.0) {
        delay_ms(100); // wait for the servo to move
        double newAngle = setAngle + error;
        setAngle = setSailPWM(newAngle);
        realAngle = readEncoder();
        error = setAngle - realAngle;
        
        attempts--;
    }
    
    return setAngle;
}