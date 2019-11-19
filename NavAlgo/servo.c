/* ************************************************************************** */
/** Servo library - based on the Northwestern Servo Control Library
 ** http://hades.mech.northwestern.edu/index.php/PIC32MX:_Servo_Control
/* ************************************************************************** */

#include <plib.h>
#include "servo.h"

#define MIN_SERVO_DUTY 3000 // 0.3 ms (TODO experiment with this)
#define MAX_SERVO_DUTY 25000 // 2.5 ms (TODO experiment with this)

int dutyCycles[4];
int updateServo = 0;

void init_servos(void) {
    OpenOC1( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Tail Servo
    OpenOC2( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Sail Servo
    OpenOC3( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Pan Servo
    OpenOC4( OC_ON | OC_TIMER2_SRC | OC_PWM_FAULT_PIN_DISABLE, 0, 0); // Tilt Servo
    
    // init Timer2 mode and period (PR2)
    // Fpb = SYS_FREQ = 40Mhz
    // Timer Prescale = 4
    // PR2 = 0xC34F = 49,999
    // interrupts every 5 ms
    // 5 ms = (PR2 + 1) * TMR Prescale / Fpb = (49999 + 1) * 4 / 40000000
    CloseTimer2();
    OpenTimer2( T2_ON | T2_PS_1_4 | T2_SOURCE_INT, 0xC34F);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_7);
    mT2SetIntPriority(7);    // set Timer2 Interrupt Priority
    mT2ClearIntFlag();       // clear interrupt flag
    mT2IntEnable(1);      // enable timer2 interrupts
    
    int i;
    for (i = 0; i < 4; i++) {
        dutyCycles[i] = (MAX_SERVO_DUTY - MIN_SERVO_DUTY) / 2; // set neutral
    }
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
    
    dutyCycles[0] = map(newTailAngle, 0, 180, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
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
    dutyCycles[1] = map(new_angle, 0, 180, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
}

/* Updates servo position for inputted pan servo angle
 * Precondition: Sail Angle in 0.. 180
 */
void setPanServoAngle(double angle) {
    dutyCycles[2] = map(angle, 0, 180, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
}

/* Updates servo position for inputted tilt servo angle
 * Precondition: Sail Angle in 0.. 180
 */
void setTiltServoAngle(double angle) {
    dutyCycles[3] = map(angle, 0, 180, MIN_SERVO_DUTY, MAX_SERVO_DUTY);
}

void __ISR( _TIMER_2_VECTOR, ipl7) T2Interrupt( void) {
    if (++updateServo >= 4) updateServo = 0;    // 20mS cycle --> 4 interrupts
    
    // Set all PWM pins low
    SetDCOC1PWM(0);
    SetDCOC2PWM(0);
    SetDCOC3PWM(0);
    SetDCOC4PWM(0);
    
    // Determine selected servo and set PWM   
    switch(updateServo) {
        case 0:
            SetDCOC1PWM(dutyCycles[updateServo]);
            break;
        case 1:
            SetDCOC2PWM(dutyCycles[updateServo]);
            break;
        case 2:
            SetDCOC3PWM(dutyCycles[updateServo]);
            break;
        case 3:
            SetDCOC4PWM(dutyCycles[updateServo]);
            break;
    }
    
    // clear interrupt flag and exit
    mT2ClearIntFlag();
}