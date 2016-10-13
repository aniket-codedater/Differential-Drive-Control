#include "controlmath.h"
#include <math.h>
#include "pidController.h"

float sigmoid(float x) {
	return ((1/(1+exp(-x))) - 0.5) * 2;
}

float degreeToRadian(float degree) {
    return degree * PI / 180;
}

float radianToDegree(float radian) {
    return radian * 180 / PI;
}

float normalizeAngle(float degree) {
    return radianToDegree(atan2(sin(degreeToRadian(degree)), cos(degreeToRadian(degree))));
}

struct point rotationalTransform(struct point point_, float theta) {
	return rotationalTransform(point_.x, point_.y, theta);
}

struct point rotationalTransform(float x, float y, float theta){
    struct point pos;
    theta = degreeToRadian(theta);
    pos.x = x * cos(theta) + y * sin(theta);
    pos.y = y * cos(theta) - x * sin(theta);
    return pos;
}

struct differentialState transformUniToDiff(struct unicycleState uniState) {
	struct differentialState diffState;
    float v;
   	if(uniState.vy>0) {
		v = sqrt(unistate.vx * unistate.vx + unistate.vy * unistate.vy);
	} else if(uniState.vy<0) {
		v = - sqrt(unistate.vx * unistate.vx + unistate.vy * unistate.vy);
	}
	//using the kinematics equations
	float vleft = (2*uniState.v -L*uniState.w) / (2 * r);
	float vright = (2*uniState.v + L*uniState.w)/(2 * r);
	diffState.rightRPM = vright / circumference * 60;
	diffState.leftRPM = vleft / circumference * 60;
	return diffState;
}




