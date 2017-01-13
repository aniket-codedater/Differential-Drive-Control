/*
 * shoot.h
 *
 *  Created on: Dec 15, 2016
 *      Author: Aniket
 */

#ifndef SHOOT_H_
#define SHOOT_H_
#include "userLib/common.h"
#include "userLib/pidController.h"
//6785 3
#define TICK_PER_REV 8145.0
#define THROW_REVOLUTION 4.0
#define STEP TICK_PER_REV*THROW_REVOLUTION
#define SHOOT_DISC 128
#define LOAD_DISC 64
#define SHOOT_TOLERANCE 15
#define STEADY_STATE_CONFIDENCE 50

extern int shootComplete,triggered;
extern volatile long int throw_counter;
extern long int des_throw_counter, FIRST_STAGE;
extern int steady_state_counter;
extern bool steady;

long int loadPoint(void);
int8_t moveThrower(long int desiredCount);
void shootDisc(void);
void cmd_throw(void);
void updateFirstStage(void);
void updateDesiredStage(void);
#endif /* SHOOT_H_ */
