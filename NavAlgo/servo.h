/* ************************************************************************** */
/** Servo Library
/* ************************************************************************** */

#ifndef _SERVO_H    /* Guard against multiple inclusion */
#define _SERVO_H

void init_servos(void);

void setTailServoAngle(double sail_angle, double tail_angle);

void setSailServoAngle(double angle);

void setPanServoAngle(double angle);

void setTiltServoAngle(double angle);

void testServo(int angle);

#endif /* _SERVO_H */

/* *****************************************************************************
 End of File
 */
