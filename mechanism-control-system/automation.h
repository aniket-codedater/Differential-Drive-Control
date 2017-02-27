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

#define POLE1_des_counter   288.146
#define POLE2_des_counter   293.13
#define POLE3_des_counter   303.423
#define POLE4_des_counter   290.941
#define POLE5_des_counter   303.423
#define POLE6_des_counter   293.13
#define POLE7_des_counter   299.8

#define POLE1_ANGLE         6
#define POLE2_ANGLE         12
#define POLE3_ANGLE         18
#define POLE4_ANGLE         20    //17-18
#define POLE5_ANGLE         18
#define POLE6_ANGLE         12
#define POLE7_ANGLE         17

#define POLE1_SHOOT_PER     0.325
#define POLE2_SHOOT_PER     0.48
#define POLE3_SHOOT_PER     0.5175
#define POLE4_SHOOT_PER     0.4725      //58-63
#define POLE5_SHOOT_PER     0.5175
#define POLE6_SHOOT_PER     0.48
#define POLE7_SHOOT_PER     0.7974

struct set_parameters {
    float SHOOT_PERCENT;
    int PLANE_ANGLE;
    float THROW_ANGLE;
};

typedef struct set_parameters SET_VALUE;
SET_VALUE SET_PARAMETERS(int POLE);

#endif /* AUTOMATION_H_ */
