/*
 * automation.c
 *
 *  Created on: 16-Jan-2017
 *      Author: HP
 */
#include "automation.h"
#include "angle.h"
#include "shoot.h"

int plane_angle[7]={POLE1_ANGLE,POLE2_ANGLE,POLE3_ANGLE,POLE4_ANGLE,POLE5_ANGLE,POLE6_ANGLE,POLE7_ANGLE};
int des_AngleCounterVal[7]={POLE1_des_counter,POLE2_des_counter,POLE3_des_counter,POLE4_des_counter,POLE5_des_counter,POLE6_des_counter,POLE7_des_counter};
float shoot_percent[7]={POLE1_SHOOT_PER,POLE2_SHOOT_PER,POLE3_SHOOT_PER,POLE4_SHOOT_PER,POLE5_SHOOT_PER,POLE6_SHOOT_PER,POLE7_SHOOT_PER};

long int converAngleToTicks(int angle){
    long int count ;
    count = angle * TICKS_PER_REVOLUTION_PER_DEGREE;
    return count;
}

SET_VALUE SET_PARAMETERS(int POLE){
    SET_VALUE AUTO;

    AUTO.SHOOT_PERCENT = setShootPercent(shoot_percent[POLE]);
    AUTO.PLANE_ANGLE = setPlaneAngle(plane_angle[POLE]);
    setThrowerPosition(convertAngleToTicks(des_AngleCounterVal[POLE]));
    return AUTO;
}
