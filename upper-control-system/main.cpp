
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <stdlib.h>
#include "driveConfig.h"
#include "minimu9.h"
#include "ps2USB.h"
#include "timerInterrupt.h"
#include "controlmath.h"
#include "pidController.h"
#include "LSA08.h"

long ticks[2] = {0, 0};
struct position curBotPosition;
struct position desiredBotPosition;
struct differentialState curDiffState;
struct differentialState desiredDiffState;

struct differentialState stopState;

float desiredHeading = 0.0;
float headingCorrection = 0;
float headingError = 0;
float prev_desiredPhi = 0;

bool forward = true, reverse = false;
float desiredJunction = 0;
float lastJunction = 0;
float ls1_error = 0, ls1_prev_error = 0;

bool ps2Ready = false;
bool imuReady = false;
bool powerOffPressed = false;

struct lineSensor ls1;

int rpiPort;

void slowTimerHandler() {
//	printf("slowLoop\n");
	digitalWrite(slowLoopLED, !digitalRead(slowLoopLED));
}

void powerOff() {
	if(powerOffPressed) return;
	powerOffPressed = true;
	printf("powerOff\n");
	stopIMU();
	stopPS2();
	stopTimer();
	for(int i = 1; i <=3; i++) {
		digitalWrite(ps2InputLED, HIGH);
		digitalWrite(headingLED, HIGH);
		digitalWrite(slowLoopLED, HIGH);
		digitalWrite(miscLED, HIGH);
		sleep(1);
		digitalWrite(ps2InputLED, LOW);
		digitalWrite(headingLED, LOW);
		digitalWrite(slowLoopLED, LOW);
		digitalWrite(miscLED, LOW);
		sleep(1);
	}
	system("shutdown -h now");
}

void reset() {
	resetRefHeading();
	resetPIDvar(headingControl);
	ls1_error = 0;
	ls1_prev_error = 0;
	headingCorrection = 0;
	prev_desiredPhi = 0;
}

void ps2Activated() {
	printf("ps2 Activated...\n");
	ps2Ready = true;
	digitalWrite(ps2InputLED, HIGH);
}

void ps2Deactivated() {
	printf("PS2 Deactivated...\n");
	ps2Ready = false;
	digitalWrite(ps2InputLED, LOW);
}

void imuActivated() {
	printf("IMU Activated...\n");
	imuReady = true;
	digitalWrite(headingLED, HIGH);
}

void imuDeactivated() {
	printf("IMU Deactivated...\n");
	imuReady = false;
	digitalWrite(headingLED, LOW);
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
    printf("%d %d \r\n", desiredDiffState.leftRPM, desiredDiffState.rightRPM);

}

/**************************************************************************************************************************/
/*Functions for automation of Differential robot																		  */
/**************************************************************************************************************************/
void calculateDiffState() {
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
}

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
	desiredState.vy = (128 - ps2_getY()) * vmax / 128;
	desiredState.vx = -(128 - ps2_getX()) * vmax / 128;
	float desiredPhi;
	if(desiredState.vy == 0 && desiredState.vx == 0) {
		desiredPhi = prev_desiredPhi;
	} else {
		desiredPhi = radianToDegree((PI/2) - atan2(abs(desiredState.vy), desiredState.vx)); //Absolute of vy so that for negative v the angle does not go -ve and the robot does not turn. We want it to drive backwards
	}
	prev_desiredPhi = desiredPhi;
	desiredState.w = PID(desiredPhi - getHeading(), headingControl); //No need of normalize angle because getHeading() gives normalized angle (I think)
//CHECK2    printf("%f %f %f\n",desiredState.vx, desiredState.vy, desiredState.w);
	return desiredState;
}

//Function for autonomous driving with heading control and linesensors
struct unicycleState getDesiredUnicycleState_line(void) {
	struct unicycleState desiredState;
	float vy, vx;
	lineFeedback();
//CHECK1	printf("%f\n",ls1_error);
	desiredState.vy = (128 - ps2_getY()) * vmax / 128;
	desiredState.vx = -(128 - ps2_getX()) * vmax / 128;
	float desiredPhi = radianToDegree((PI/2) - atan2(abs(desiredState.vy), desiredState.vx)); //Absolute of vy so that for negative v the angle does not go -ve and the robot does not turn. We want it to drive backwards
	desiredState.vx += PID(ls1_error,lineControl);
	desiredState.w = PID(desiredPhi - getHeading(), headingControl); //No need of normalize angle because getHeading() gives normalized angle (I think)
//CHECK2	printf("%f %f %f\n",desiredState.vx, desiredState.vy, desiredState.w);
	return desiredState;
}

void timerHandler() {
	if(!ps2Ready || !imuReady) {
		transmitDiffState(stopState);
	} else {
//For automation
//	AUTOMATION based on Positioning
//		calculateDiffState();
//		calculatePos();
//		desiredDiffState = transformUniToDiff(getDesiredUnicycleState(curBotPosition, desiredBotPosition));

//	AUTOMATION based on LineSensors
//		desiredDiffState = transformUniToDiff(getDesiredUnicycleState_line());

//For manual with heading control
		desiredDiffState = transformUniToDiff(getDesiredUnicycleState_manual());

		transmitDiffState(desiredDiffState);
		digitalWrite(miscLED, !digitalRead(miscLED));
//CHECK3		printf("%d %d\n",desiredDiffState.leftRPM,desiredDiffState.rightRPM);
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
	initIMU();
	//taking initial point as origin
	curBotPosition.x = 0.0;
	curBotPosition.y = 0.0;
	curBotPosition.phi = 0;
}

//Interrupt on Junction occurence
void junctionInterrupt() {
	if (forward) {
			lastJunction++;
	} else if (reverse) {
		lastJunction--;
	}
}

int main() {
	rpiPort = serialOpen("/dev/ttyS0",38400);			/*Serial communication port established*/
	initPIDController(0.05,0.0,0.0,headingControl);		/*PID controller initialization*/
	initPIDController(0.0,0.0,0.0,lineControl);
	init();

	ls1.address = 1;
	ls1.uartPort = rpiPort;
	ls1.UARTPin = 6;
	ls1.junctionPin = 13;

	desiredBotPosition.x = 0.0;
	desiredBotPosition.y = 0.0;
	desiredBotPosition.phi = 0;

	initLineSensor(ls1,junctionInterrupt);
	initTimer(1000000/PIDfrequency, &timerHandler);
	while(1) {
        sleep(1);
	}
}





