#include "controlmath.h"
#include <math.h>
#include "pidController.h"

bool append = false;

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

struct differentialState rpmLimiter(struct differentialState state) {
	float max,factor = 1,dividend = 1;
	(fabs(state.leftRPM) >= fabs(state.rightRPM)) ? max = fabs(state.leftRPM) : max = fabs(state.rightRPM);
	dividend = max;
	if(max > maxRPM) {
		while(dividend > maxRPM) {
			dividend = max / factor;
			factor += 0.5;
		}
		factor -= 0.5;
		state.leftRPM /= factor;	
		state.rightRPM /= factor;	
		append = true;
		return state;
	}
	append = false;
	return state;
}

struct unicycleState velocityLimiter(struct unicycleState state) {
	float max,factor = 1,dividend = 1;
	(fabs(state.vx) >= fabs(state.vy)) ? max = fabs(state.vx) : max = fabs(state.vy);
	dividend = max;
	if(max > maxVelocity) {
		while(dividend > maxVelocity) {
			dividend = max / factor;
			factor += 0.5;
		}
		factor -= 0.5;
		state.vx /= factor;	
		state.vy /= factor;	
	}
	return state;
}

struct differentialState transformUniToDiff(struct unicycleState uniState) {
	struct differentialState diffState;
    float v = sqrt(uniState.vx * uniState.vx + uniState.vy * uniState.vy);
	uniState = velocityLimiter(uniState);
	float w = fabs((L*uniState.w * 60.0) / (2 * r * circumference));	
	if(uniState.vy<0) {
		v = - v;
		w = -w;
	}
	//using the kinematics equations
	float vleft = (2*v - L*uniState.w) / (2 * r);
	float vright = (2*v + L*uniState.w)/(2 * r);
	diffState.rightRPM = vright / circumference * 60;
	diffState.leftRPM = vleft / circumference * 60;
	diffState = rpmLimiter(diffState);
	//If the RPM hits maximum limit, the heading correction rpm should be applied explicitly because if heading correction done before is scaled to almost neglibile
	if(append) {
		(vleft > vright) ? diffState.rightRPM = diffState.rightRPM - 2*w : diffState.leftRPM = diffState.leftRPM - 2*w;  
	}
	return (diffState);
}




