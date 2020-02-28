/* ************************************************************************** */
/** Servo Library
/* ************************************************************************** */

#ifndef _SERVO_H    /* Guard against multiple inclusion */
#define _SERVO_H

void initServos(void);

double setTailServoAngle(double sail_angle, double tail_angle);

double setSailServoAngle(double angle);

void testServo(int angle);

#endif /* _SERVO_H */

/* *****************************************************************************
 End of File
 */
