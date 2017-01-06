#include "servo.h"
#include "pwm_gen.h"

int servolibID[MAX_SERVO];
int id = -1;

void resetServo(void) {
	id = -1;
}

int servoInit(int pwmPin,int count_mode, int freq) {
	if(id < MAX_SERVO) {
		enable_pwm_clock(8);
		config_pwm(pwmPin,count_mode,freq);
		id++;
		servolibID[id] = pwmPin;
		return id;		
	} else {
		return -1;
	}
}

void moveServo(float angle,int servo_id) {
	float degree = ((0.00186/180)*angle+0.00054);
	set_pwm(servolibID[servo_id],1,0,degree);
}
