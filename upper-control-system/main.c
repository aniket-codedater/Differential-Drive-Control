
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


long ticks[2] = {0, 0};
struct position curBotPosition;
struct position desiredBotPosition;
struct differentialState curDiffState;
struct differentialState desiredDiffState;

struct differentialState stopState;


float phi_ref = 0;


float desiredHeading = 0.0;
float headingCorrection = 0;
float headingError = 0;


bool ps2Ready = false;
bool imuReady = false;
bool powerOffPressed = false;



//float kp[3] = {0.1, 0.1, 0.1}, ki[3] = {0, 0, 0}, kd[3] = {0.00, 0.00, 0}, E[3] = {0, 0, 0}, e_old[3] = {0, 0, 0};
/**
float PID(float error,int x) {
	float pid = 0;
	pid = (kp[x]*error) + (ki[x]*E[x]) + (kd[x]*(error - e_old[x]));
	E[x]+=error;
	e_old[x] = error;
	return pid;
}

**/
/**

inline void Graph_Plot()
{
	USART_Transmitchar(0xAB);
	USART_Transmitchar(0xCD);
	USART_Transmitchar(0x08);
	USART_Transmitchar(0x00);
	USART_Transmitchar(curDiffState.leftRPM & 0x00FF);
	USART_Transmitchar(((curDiffState.leftRPM & 0xFF00) >> 8));
	USART_Transmitchar(curDiffState.rightRPM & 0x00FF);
	USART_Transmitchar(((curDiffState.rightRPM & 0xFF00) >> 8));
	USART_Transmitchar(0x00);
	USART_Transmitchar(0x00);
	USART_Transmitchar(0x00);
	USART_Transmitchar(0x00);
}
**/

int rpiPort;

void slowTimerHandler() {
	printf("slowLoop\n");
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
	headingCorrection = 0;
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
        rpm = 200; //must be even
    } else if(rpm < -(maxRPM + 1)) {
        printf("Warning: Trying to send Velocity lesser than -181. Limit : -181. Sending -181.\r\n");
        rpm = -201; // must be odd
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
    printf("%d %d \r\n", desiredDiffState.leftRPM, desiredDiffState.rightRPM);
    serialPutchar(rpiPort, 0x0A);
    printf("%c\t",encodeByte(desiredDiffState.leftRPM));
    printf("%c\t;",encodeByte(desiredDiffState.rightRPM));
    serialPutchar(rpiPort, encodeByte(desiredDiffState.leftRPM));
    serialPutchar(rpiPort, encodeByte(desiredDiffState.rightRPM));
}


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
	curBotPosition.phi = normalizeAngle(phi_ref - getHeading());

	float leftDist = curDiffState.leftRPM * timeInterval / 60.0 * circumference;
	float rightDist = curDiffState.rightRPM * timeInterval / 60.0 * circumference;
	float dist = (leftDist + rightDist) / 2;

	curBotPosition.x += dist * cos(degreeToRadian(curBotPosition.phi));
	curBotPosition.y += dist * sin(degreeToRadian(curBotPosition.phi));
}

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
	printf("%f  ",desiredState.v);
	printf("%f  ;",desiredState.w);

	return desiredState;
}



void timerHandler() {
	if(!ps2Ready || !imuReady) {
		transmitDiffState(stopState);
	} else {
        calculateDiffState();
        calculatePos();
        desiredDiffState = transformUniToDiff(getDesiredUnicycleState(curBotPosition, desiredBotPosition));
        transmitDiffState(desiredDiffState);
	digitalWrite(miscLED, !digitalRead(miscLED));
	}
}
/**

void print(int x) {
	if(x < 0) {
		x *= -1;
		//USART_Transmitchar('-');
	}
	//USART_TransmitNumber(x);
}
**/


void init() {
   	rpiPort = serialOpen("/dev/ttyS0",38400);
   	stopState.leftRPM = 0;
	stopState.rightRPM = 0;
		if(wiringPiSetup() < 0) {
		printf("Error setting up while setting wiringPi\n");
	}

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

	enablePS2StatusInterrupt(&ps2Activated, &ps2Deactivated);
	enableIMUStatusInterrupt(&imuActivated, &imuDeactivated);
	enableSlowFuncInterrupt(&slowTimerHandler);
	initPS2();
	initIMU();

	//USART_Init(12);
	phi_ref = getHeading();					//initialising the first ever angle taken by the compass sensor as reference
	//init_movingArray(rpmMovArrayLength, lRPM);
	//init_movingArray(rpmMovArrayLength, rRPM);
	initPIDController(1.0,0.0,0.0,headingControl);
	//taking initial point as origin
	curBotPosition.x = 0.0;
	curBotPosition.y = 0.0;
	curBotPosition.phi = 0;

}

int main() {
	init();
	desiredBotPosition.x = 0.0;
	desiredBotPosition.y = 0.0;
	desiredBotPosition.phi = 0;
	initTimer(1000000/PIDfrequency, &timerHandler);
	while(1) {
        sleep(1);
	}
}





