#ifndef SERVO_H
#define SERVO_H

#define MAX_SERVO 2

void resetServo(void);
int servoInit(int pwmPin, int count_mode, int freq);
void moveServo(float angle,int servo_id);

#endif
