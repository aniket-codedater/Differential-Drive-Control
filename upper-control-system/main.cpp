// Compile with -lpthread -lwiringPi
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include "driveConfig.h"
#include "userlib_interrupt.h"
#include "minimu9.h"
#include "ps2USB.h"
#include "timerInterrupt.h"
#include "controlmath.h"
#include "pidController.h"
#include "LSA08.h"

//Interrupt function definitions
bool ps2Ready = false;
bool imuReady = false;
bool powerOffPressed = false;
void slowTimerHandler();
void ps2Activated();
void ps2Deactivated();
void imuActivated();
void imuDeactivated();

struct position curBotPosition;
struct position desiredBotPosition;
struct differentialState curDiffState;
struct differentialState desiredDiffState;
struct differentialState stopState;
struct lineSensor ls1,ls2;

float desiredHeading = 0.0;
float headingCorrection = 0;
float headingError = 0;
float prev_desiredPhi = 0;
float prev_vy = 0;

bool forward = true, reverse = false;
float desiredJunction = 4;
volatile float lastJunction = 0;

float ls1_error = 0, ls1_prev_error = 0;
float ls2_error = 0, ls2_prev_error = 0;
float ticks[2];

int rpiPort;

void reset() {
	printf("RESET\n");
	resetRefHeading();
	resetPIDvar(headingControl);
	resetPIDvar(lineControl_fw);
	resetPIDvar(lineControl_bw);
	ls1_error = 0;
	ls1_prev_error = 0;
	ls2_error = 0;
	ls2_prev_error = 0;
	headingCorrection = 0;
	prev_desiredPhi = 0;
	prev_vy = 0;
	lastJunction = 0;
	desiredJunction = 0;
	forward = true;
	reverse = false;
}

char encodeByte(int rpm) {
    if(rpm > maxRPM) {
        printf("Warning: Trying to send Velocity greater than 180. Limit : 180. Sending 180.\r\n");
        rpm = 252; //must be even (maxRPM)
    } else if(rpm < -(maxRPM + 1)) {
        printf("Warning: Trying to send Velocity lesser than -181. Limit : -181. Sending -181.\r\n");
        rpm = -251; // must be odd
    }
    if(rpm < 0) {
        rpm = -rpm;
        return (rpm | 0x01);
    } else {
        rpm = (rpm & 0xFE);
        return rpm == 0x0A ? 0x0C : rpm;
    }
}

void transmitDiffState(struct differentialState desiredDiffState) {
    serialPutchar(rpiPort, 0x0A);
    serialPutchar(rpiPort, encodeByte(desiredDiffState.leftRPM));
    serialPutchar(rpiPort, encodeByte(desiredDiffState.rightRPM));
}

/**************************************************************************************************************************/
/*Functions for automation of Differential robot																		  */
/**************************************************************************************************************************/
/*void calculateDiffState() {
	int x;
	int sampledTicks[] = {ticks[left], ticks[right]};
	ticks[0] = 0;
	ticks[1] = 0;
	float lrpm = sampledTicks[0] * 60 / ticksPerRotation / (timeInterval);
	float rrpm = sampledTicks[1] * 60 / ticksPerRotation / (timeInterval);
	curDiffState.leftRPM = lrpm;
	curDiffState.rightRPM = rrpm;
}

void calculatePos() {
	curBotPosition.phi = normalizeAngle(getHeading());
	float leftDist = curDiffState.leftRPM * timeInterval / 60.0 * circumference;
	float rightDist = curDiffState.rightRPM * timeInterval / 60.0 * circumference;
	float dist = (leftDist + rightDist) / 2;

	curBotPosition.x += dist * cos(degreeToRadian(curBotPosition.phi));
	curBotPosition.y += dist * sin(degreeToRadian(curBotPosition.phi));
}*/

//Function to read linesensors and preprocess it.
void lineFeedback(void) {
    ls1_error = readLineSensor(ls1);
    if(ls1_error == 255) {
    	if(ls1_prev_error < 0) {
			ls1_error = -50;
		} else if(ls1_prev_error > 0) {
			ls1_error = 50;
		} else {
			ls1_error = 0;
		}
    }
    ls1_prev_error = ls1_error;
    usleep(1000);	//readings skew if this is removed. Need further study : Aniket,20 October,2016
    ls2_error = readLineSensor(ls2);
    if(ls2_error == 255) {
    	if(ls2_prev_error < 0) {
			ls2_error = -50;
		} else if(ls2_prev_error > 0) {
			ls2_error = 50;
		} else {
			ls2_error = 0;
		}
    }
    ls2_prev_error = ls2_error;
}

//Map velocity according to percent path
float velocityMap(void) {
	float velocity = 0;
	int percentPath = desiredJunction - lastJunction;
	switch(abs(percentPath)) {
		case 0:
			velocity = 0;
			break;
		case 1:
			velocity = maxVelocity * 0.3;
			break;
		case 2:
			velocity = maxVelocity * 0.6;
			break;
		case 3:
			velocity = maxVelocity * 0.9;
			break;
		case 4:
			velocity = maxVelocity;
			break;
		case 5:
			velocity = maxVelocity;
			break;
	}
	if (percentPath > 0) {
		forward = true;
		reverse = false;
	} else if(percentPath < 0) {
		forward = false;
		reverse = true;
		velocity = -velocity;
	} else {
		forward = false;
		reverse = false;
	}
	return velocity;
}

/**************************************************************************************************************************/
/*Functions for manual driving of Differential robot with heading control												  */
/**************************************************************************************************************************/
/*
struct unicycleState getDesiredUnicycleState(struct position curBotPosition,struct position desiredBotPosition) {
	struct unicycleState desiredState;
	float vy;
	float vx;
	float v;
	//desiredBotPosition.x = (128 - ps2_getX());
	//desiredBotPosition.y = -(128 - ps2_getY());
	vy = (128 - ps2_getY()) * vmax / 128;
	vx = -(128 - ps2_getX()) * vmax / 128;
	if(vy>0) {
		v=sqrt((vx*vx)+(vy*vy));
	} else {
		v=-sqrt((vx*vx)+(vy*vy));
	}
	float desiredPhi = 0;
	if(vx == 0 && vy == 0) {
		desiredPhi = 0;
	} else if(vx == 0) {
		desiredPhi = 0;
	} else if(vy==0) {
		if(vx>0) {
			desiredPhi = 90;
		} else {
			desiredPhi = -90;
		}
	} else {
		if(vy>0 && vx>0) {
			desiredPhi = radianToDegree(atan(vy/vx));
		} else if(vy>0 && vx<0) {
			desiredPhi = -radianToDegree(atan((-1)*vy/vx));
		} else if(vy<0 && vx>0) {
			desiredPhi = 180 - radianToDegree(atan((-1)*vy/vx));
		} else {
			desiredPhi = radianToDegree(atan(vy/vx)) - 180;
		}
		//desiredPhi = radianToDegree(atan(vy/vx));

		//desiredPhi = radianToDegree(atan((desiredBotPosition.y - curBotPosition.y) / (desiredBotPosition.x - curBotPosition.x)));
	}
	printf(" %f ",desiredPhi);
	desiredState.v = vmax * sigmoid(v);
	//desiredState.w = normalizeAngle(desiredPhi - getHeading());
	desiredState.w = PID(normalizeAngle(desiredPhi - getHeading()), angularVel);
	//printf("%f  ",desiredState.v);
	//printf("W = %f ;\n",desiredState.w);
	return desiredState;
}
*/

//Function for manual driving with heading control
struct unicycleState getDesiredUnicycleState_manual(void) {
	struct unicycleState desiredState;
	float vy, vx;
	desiredState.vy = (128 - ps2_getY()) * maxVelocity / 128;
	desiredState.vx = -(128 - ps2_getX()) * maxVelocity / 128;
	float desiredPhi;
	if(desiredState.vy == 0 && desiredState.vx == 0) {
		desiredPhi = prev_desiredPhi;
	} else {
		desiredPhi = radianToDegree((PI/2) - atan2(abs(desiredState.vy), desiredState.vx)); //Absolute of vy so that for negative v the angle does not go -ve and the robot does not turn. We want it to drive backwards
	}
	prev_desiredPhi = desiredPhi;
	desiredState.w = PID(desiredPhi - getHeading(), headingControl);
//CHECK2 printf("%f %f %f;",desiredState.vx, desiredState.vy, desiredState.w);
	return desiredState;
}

//Function for semiautonomous driving with heading control and linesensors
struct unicycleState getDesiredUnicycleState_line(void) {
	struct unicycleState desiredState;
	float vy, vx;
	lineFeedback();
//CHECK1 printf("%f %f \n",ls1_error,ls2_error);
	desiredState.vx = 0;
	desiredState.vy = (128 - ps2_getY()) * maxVelocity;
	
	if(desiredState.vy > 0) {
		desiredState.w = PID(ls1_error, lineControl_fw);
		prev_vy = desiredState.vy;
	} else if(desiredState.vy < 0) {
		desiredState.w = PID(ls2_error, lineControl_bw);
		prev_vy = desiredState.vy;
	} else {
		if(prev_vy >= 0) {
			desiredState.w = PID(ls1_error, lineControl_fw);
		} else {
			desiredState.w = PID(ls2_error, lineControl_bw);			
		}
	}
//CHECK2 printf("%f %f %f\n",desiredState.vx, desiredState.vy, desiredState.w);
	return desiredState;
}

//Function for autonomous driving with heading control and linesensors
struct unicycleState getDesiredUnicycleState_auto(void) {
	struct unicycleState desiredState;
	float vy, vx;
	lineFeedback();
//CHECK1 printf("%f\n",ls1_error);
	desiredState.vx = 0;
	desiredState.vy = velocityMap();
	if(desiredState.vy > 0) {
		desiredState.w = PID(ls1_error, lineControl_fw);
		prev_vy = desiredState.vy;
	} else if(desiredState.vy < 0) {
		desiredState.w = PID(ls2_error, lineControl_bw);
		prev_vy = desiredState.vy;
	} else {
		if(prev_vy >= 0) {
			desiredState.w = PID(ls1_error, lineControl_fw);
		} else {
			desiredState.w = PID(ls2_error, lineControl_bw);			
		}
	}
//CHECK2	printf("%f %f %f\n",desiredState.vx, desiredState.vy, desiredState.w);
	return desiredState;
}

void timerHandler() {
	if(!ps2Ready /*|| !imuReady*/) {
		transmitDiffState(stopState);
	} else {
/*	AUTOMATION based on Positioning */
//		calculateDiffState();
//		calculatePos();
//		desiredDiffState = transformUniToDiff(getDesiredUnicycleState(curBotPosition, desiredBotPosition));

/*	semiAUTOMATION based on LineSensors */
//		desiredDiffState = transformUniToDiff(getDesiredUnicycleState_line());

/*	AUTOMATION based on LineSensors */
		desiredDiffState = transformUniToDiff(getDesiredUnicycleState_auto());

/* MANUAL with heading control */
//		desiredDiffState = transformUniToDiff(getDesiredUnicycleState_manual());

		transmitDiffState(desiredDiffState);
		digitalWrite(miscLED, !digitalRead(miscLED));
//CHECK4 
printf("%d %d \n",desiredDiffState.leftRPM,desiredDiffState.rightRPM);
	}
}

void init() {
   	stopState.leftRPM = 0;
	stopState.rightRPM = 0;
	if(wiringPiSetup() < 0) {
		printf("Error setting up while setting wiringPi\n");
	}
/*
* Button and LED configurations
*/
	pinMode(powerOffButton, INPUT);
	pinMode(headingRefButton, INPUT);
	pinMode(ps2InputLED, OUTPUT);
	pinMode(headingLED, OUTPUT);
	pinMode(slowLoopLED, OUTPUT);
	pinMode(miscLED, OUTPUT);
	if(wiringPiISR(powerOffButton, INT_EDGE_RISING, &powerOff) < 0) {
		printf("Power Off Button interrupt setup error \n");
	}
	if(wiringPiISR(headingRefButton, INT_EDGE_RISING, &reset)) {
		printf("Reset Heading Button interrupt setup error\n");
	}
	digitalWrite(ps2InputLED, LOW);
	digitalWrite(headingLED, LOW);
	digitalWrite(slowLoopLED, LOW);
	digitalWrite(miscLED, LOW);
/*
* PS2 and IMU configuration
*/
	enablePS2StatusInterrupt(&ps2Activated, &ps2Deactivated);
	enableIMUStatusInterrupt(&imuActivated, &imuDeactivated);
	enableSlowFuncInterrupt(&slowTimerHandler);
	initPS2();
	//initIMU();
	//taking initial point as origin
	curBotPosition.x = 0.0;
	curBotPosition.y = 0.0;
	curBotPosition.phi = 0;
}

//Interrupt on Junction occurence
void junctionInterrupt(void) {
	if (forward) {
		lastJunction++;
	} else if (reverse) {
		lastJunction--;
	}
}

void dummy() {}

int main() {
	rpiPort = serialOpen("/dev/ttyS0",38400);			/*Serial communication port established*/
	initPIDController(0.25,0.0,3.0,headingControl);		/*PID controller for angular velocity in manual mode*/
	initPIDController(0.03,0.0,1.2,lineControl_fw);		/*PID controller for angular velocity in linefollow_fw mode*/
	initPIDController(0.03,0.0,1.2,lineControl_bw);		/*PID controller for angular velocity in linefollow_bw mode*/
	init();

	ls1.address = 1;
	ls1.uartPort = rpiPort;
	ls1.UARTPin = 6;
	ls1.junctionPin = 5;

	ls2.address = 2;
	ls2.uartPort = rpiPort;
	ls2.UARTPin = 12;
	ls2.junctionPin = 13;

	desiredBotPosition.x = 0.0;
	desiredBotPosition.y = 0.0;
	desiredBotPosition.phi = 0;

	initLineSensor(ls1, &junctionInterrupt);
	initLineSensor(ls2, &dummy);
	initTimer(1000000/PIDfrequency, &timerHandler);
	while(1) {
//		printf("%f\n",lastJunction);
        	sleep(1);
	}
}
