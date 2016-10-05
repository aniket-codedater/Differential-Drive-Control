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



struct differentialState transformUniToDiff(struct unicycleState uniState) {
	struct differentialState diffState;

	//using the kinematics equations
	float vleft = (2*uniState.v -L*uniState.w) / (2 * r);
	float vright = (2*uniState.v + L*uniState.w)/(2 * r);
	diffState.rightRPM = vright / circumference * 60;
	diffState.leftRPM = vleft / circumference * 60;
	return diffState;
}




