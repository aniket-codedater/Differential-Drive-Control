/*
 * shoot.c
 *
 *  Created on: Dec 15, 2016
 *      Author: Aniket
 */
#include "shoot.h"
#include <math.h>
//10150 maxpWM/30 0.1
int shootComplete = 1,triggered = 0;
long int des_throw_counter = STEP, FIRST_STAGE = STEP/THROW_REVOLUTION, SECOND_STAGE = STEP/THROW_REVOLUTION * 1;
volatile long int throw_counter = STEP;
int steady_state_counter = 0;
bool steady = false;

long int loadPoint(void) {
	int factor = round(throw_counter / TICK_PER_REV) ;
	return TICK_PER_REV*factor;
}

int8_t moveThrower(long int desiredCount) {
	float error = (desiredCount-throw_counter);
	float pwm = PID(throw_motor,error);
	if(absolute(error)<=SHOOT_TOLERANCE)
	{
		setPWM(0,throw_motor);
		if(steady == false) {
			steady_state_counter++;
		}
		if(steady_state_counter > STEADY_STATE_CONFIDENCE) {
			steady = true;
			return 1;
		}
	} else {
		steady = false;
		steady_state_counter = 0;
		setPWM(pwm,throw_motor);
	}
	return 0;
}

void shootDisc(void) {
	if(throw_counter <= FIRST_STAGE) {
		shootComplete = 0;
		setPWM(maxPWM_throw,throw_motor);
	}
	else if(throw_counter>FIRST_STAGE && throw_counter<=(FIRST_STAGE  + SECOND_STAGE)) {
		shootComplete = 0;
		setPWM(minPWM_throw,throw_motor);
	} else {
		shootComplete = moveThrower(des_throw_counter);
	}
}

void cmd_throw(void) {
	if((des_throw_counter + STEP) < 0)
	{
		throw_counter = throw_counter - des_throw_counter + STEP;
	}
	des_throw_counter += STEP;
	FIRST_STAGE += STEP;
	if(des_throw_counter < 0) {
		des_throw_counter = STEP;
		FIRST_STAGE = des_throw_counter/THROW_REVOLUTION;
	}
}
