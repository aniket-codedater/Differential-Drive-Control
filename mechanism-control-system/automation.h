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
#define POLE1_des_counter   52.5
#define POLE2_des_counter   282.0
#define POLE3_des_counter   289.2   //69
#define POLE4_des_counter   295.9   //71        //16/2/17 2:16
#define POLE5_des_counter   289.2   //69
#define POLE6_des_counter   282.0   //27
#define POLE7_des_counter   82.5

#define POLE1_ANGLE         0
#define POLE2_ANGLE         16
#define POLE3_ANGLE         12
#define POLE4_ANGLE         19    //-16
#define POLE5_ANGLE         12
#define POLE6_ANGLE         16
#define POLE7_ANGLE         0

#define POLE1_SHOOT_PER     0.45
#define POLE2_SHOOT_PER     0.47
#define POLE3_SHOOT_PER     0.4875
#define POLE4_SHOOT_PER     0.56      //58-63
#define POLE5_SHOOT_PER     0.4620
#define POLE6_SHOOT_PER     0.47
#define POLE7_SHOOT_PER     1.0

struct set_parameters {
    float SHOOT_PERCENT;
    int PLANE_ANGLE;
    float THROW_ANGLE;
};

typedef struct set_parameters SET_VALUE;
SET_VALUE SET_PARAMETERS(int POLE);

#endif /* AUTOMATION_H_ */
