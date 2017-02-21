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
                                    //Height
#define POLE1_des_counter   202.8   //h69
#define POLE2_des_counter   291.124 //h12
#define POLE3_des_counter   294.0   //h41
#define POLE4_des_counter   279.4   //279-290 h69      //16/2/17 2:16
#define POLE5_des_counter   294.0   //h41
#define POLE6_des_counter   291.124 //h12
#define POLE7_des_counter   299.8   //h69

#define POLE1_ANGLE         6
#define POLE2_ANGLE         16
#define POLE3_ANGLE         15
#define POLE4_ANGLE         17    //17-18
#define POLE5_ANGLE         15
#define POLE6_ANGLE         16
#define POLE7_ANGLE         17

#define POLE1_SHOOT_PER     0.325
#define POLE2_SHOOT_PER     0.5075
#define POLE3_SHOOT_PER     0.5175
#define POLE4_SHOOT_PER     0.5225      //58-63
#define POLE5_SHOOT_PER     0.5175
#define POLE6_SHOOT_PER     0.5075
#define POLE7_SHOOT_PER     0.7974

struct set_parameters {
    float SHOOT_PERCENT;
    int PLANE_ANGLE;
    float THROW_ANGLE;
};

typedef struct set_parameters SET_VALUE;
SET_VALUE SET_PARAMETERS(int POLE);

#endif /* AUTOMATION_H_ */
