/*
 * automation.h
 *
 *  Created on: 16-Jan-2017
 *      Author: HP
 */

#ifndef AUTOMATION_H_
#define AUTOMATION_H_

#define TICKS_PER_REVOLUTION 8145
#define TICKS_PER_REVOLUTION_PER_DEGREE 22.625

#define POLE1_des_counter   52.5
#define POLE2_des_counter   52.5
#define POLE3_des_counter   52.5
#define POLE4_des_counter   52.5
#define POLE5_des_counter   52.5
#define POLE6_des_counter   52.5
#define POLE7_des_counter   52.5

#define POLE1_ANGLE         9
#define POLE2_ANGLE         8
#define POLE3_ANGLE         7
#define POLE4_ANGLE         6
#define POLE5_ANGLE         6
#define POLE6_ANGLE         8
#define POLE7_ANGLE         9

#define POLE1_SHOOT_PER     0.05
#define POLE2_SHOOT_PER     0.05
#define POLE3_SHOOT_PER     0.05
#define POLE4_SHOOT_PER     0.05
#define POLE5_SHOOT_PER     0.05
#define POLE6_SHOOT_PER     0.05
#define POLE7_SHOOT_PER     0.05


struct set_parameters {
    float SHOOT_PERCENT;
    int PLANE_ANGLE;
};

typedef struct set_parameters SET_VALUE;
SET_VALUE SET_PARAMETERS(int POLE);
long int converAngleToTicks(int angle);

#endif /* AUTOMATION_H_ */
