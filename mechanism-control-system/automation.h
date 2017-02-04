/*
 * automation.h
 *
 *  Created on: 16-Jan-2017
 *      Author: HP
 */

#ifndef AUTOMATION_H_
#define AUTOMATION_H_

#include "shoot.h"
#include "angle.h"
#include "load.h"
#include "userLib/common.h"

#define POLE1_des_counter   52.5
#define POLE2_des_counter   52.5
#define POLE3_des_counter   52.5
#define POLE4_des_counter   90
#define POLE5_des_counter   52.5
#define POLE6_des_counter   52.5
#define POLE7_des_counter   82.5

#define POLE1_ANGLE         9
#define POLE2_ANGLE         8
#define POLE3_ANGLE         7
#define POLE4_ANGLE         11
#define POLE5_ANGLE         6
#define POLE6_ANGLE         8
#define POLE7_ANGLE         1

#define POLE1_SHOOT_PER     0.45
#define POLE2_SHOOT_PER     0.85
#define POLE3_SHOOT_PER     0.65
#define POLE4_SHOOT_PER     0.825
#define POLE5_SHOOT_PER     0.7
#define POLE6_SHOOT_PER     0.5
#define POLE7_SHOOT_PER     1.0


struct set_parameters {
    float SHOOT_PERCENT;
    int PLANE_ANGLE;
    float THROW_ANGLE;
};

typedef struct set_parameters SET_VALUE;
SET_VALUE SET_PARAMETERS(int POLE);

#endif /* AUTOMATION_H_ */
