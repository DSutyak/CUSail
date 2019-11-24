/* ************************************************************************** */
/** Servo library
/* ************************************************************************** */

#include <plib.h>
#include "servo.h"
#include "sensors.h"

#define MIN_SERVO_DUTY 10000 // 1 ms (TODO experiment with this)
#define MAX_SERVO_DUTY 20000 // 2 ms (TODO experiment with this)

void init_servos(void) {
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Tail Servo
    OpenOC2( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Sail Servo
    OpenOC3( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Pan Servo
    OpenOC4( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Tilt Servo
    
    // Fpb = SYS_FREQ = 40Mhz
    // Timer Prescale = 4
    // PR2 = 0x30D3F = 199,999
    // 20 ms = (PR2 + 1) * TMR Prescale / Fpb = (199,999 + 1) * 4 / 40,000,000
    CloseTimer2();
    OpenTimer2( T2_ON | T2_PS_1_4 | T2_SOURCE_INT, 0x30D3F);
    
    //TODO: set PPS to configure pins
    
    // init each servo to neutral
    SetDCOC1PWM((MAX_SERVO_DUTY - MIN_SERVO_DUTY) / 2);
    SetDCOC2PWM((MAX_SERVO_DUTY - MIN_SERVO_DUTY) / 2);
    SetDCOC3PWM((MAX_SERVO_DUTY - MIN_SERVO_DUTY) / 2);
    SetDCOC4PWM((MAX_SERVO_DUTY - MIN_SERVO_DUTY) / 2);
}

double map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return ((toHigh - toLow) * (value - fromLow)/(fromHigh-fromLow)) + toLow;
}

/* Returns servo command tail servo for inputted sail angle and tail angle
 * Precondition: Sail Angle in 0.. 360 w.r.t boat, Tail Angle in -180.. 180 w.r.t boat
 */
void setTailServoAngle(double sail_angle, double tail_angle) {
    double s_angle = sail_angle;
    if (s_angle > 180){ //convert sail angle to -180.. 180
        s_angle -= 360;
    }
    
    double newTailAngle = tail_angle - s_angle; //calculate position of tail with respect to sail
    
    //make sure tail angle is in range -180.. 180
    if(newTailAngle<-180){
        newTailAngle+=360;
    } else if (newTailAngle>180) {
        newTailAngle-=360;
    }
    
    //map to servo commands
    if (newTailAngle <= 0 ) {
        newTailAngle=map(newTailAngle,-30,0,160,100);
    } else if (newTailAngle > 0 ) {
        newTailAngle=map(newTailAngle,0,30,100,60);
    }
    
    SetDCOC1PWM((int) map(newTailAngle, 0, 180, MIN_SERVO_DUTY, MAX_SERVO_DUTY));
}

/* Updates servo position for inputted sail angle
 * Precondition: Sail Angle in 0.. 360 w.r.t boat
 */
void setSailServoAngle(double angle) {
    double new_angle;
    if (angle <= 90) {
        new_angle = map(angle, 0, 90, 142, 125);
    } else if (angle <= 180) {
        new_angle = map(angle, 90, 180, 125, 108);
    } else if (angle <= 270) {
        new_angle = map(angle, 180, 270, 108, 91);
    } else {
        new_angle = map(angle, 270, 360, 91, 74);
    }
    
    SetDCOC2PWM((int) map(new_angle, 0, 180, MIN_SERVO_DUTY, MAX_SERVO_DUTY));
}

/* Updates servo position for inputted pan servo angle
 * Precondition: Sail Angle in 0.. 180
 */
void setPanServoAngle(double angle) {
    SetDCOC3PWM((int) map(angle, 0, 180, MIN_SERVO_DUTY, MAX_SERVO_DUTY));
}

/* Updates servo position for inputted tilt servo angle
 * Precondition: Sail Angle in 0.. 180
 */
void setTiltServoAngle(double angle) {
    SetDCOC4PWM((int) map(angle, 0, 180, MIN_SERVO_DUTY, MAX_SERVO_DUTY));
}