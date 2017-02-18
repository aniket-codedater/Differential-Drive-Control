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
#define TICK_PER_REV                    49000.0
#define TICKS_PER_REVOLUTION_PER_DEGREE 136.11     //Green encoder ticks 4000 for 360degree
#define THROW_REVOLUTION                4.0
#define STEP                            TICK_PER_REV*THROW_REVOLUTION
#define SHOOT_DISC                      128
#define LOAD_DISC                       64
#define SHOOT_TOLERANCE                 50
#define STEADY_STATE_CONFIDENCE         50
#define ZCD_CONFIDENCE                  30

extern int shootComplete,triggered;
extern volatile long int throw_counter;
extern long int des_throw_counter, FIRST_STAGE;
extern bool steady;
extern float throw_angle;

long int loadPoint(void);
int8_t moveThrower(long int desiredCount);
void shootDisc(bool shootState);
void cmd_throw(void);
void updateFirstStage(void);
void updateDesiredStage(void);
float convertTicksToThrowAngle (long int count);
long int convertThrowAngleToTicks (float angle);

#endif /* SHOOT_H_ */
