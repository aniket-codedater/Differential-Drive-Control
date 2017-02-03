// Compile with -lpthread -lwiringPi
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>
#include <stdlib.h>
#include <fstream>
#include <istream>
#include "driveConfig.h"
#include "userlib_interrupt.h"
#include "minimu9.h"
#include "ps2USB.h"
#include "ps2_2k17.h"
#include "timerInterrupt.h"
#include "controlmath.h"
#include "pidController.h"
#include "LSA08.h"
#include "mechanismConfig.h"
#include "encoderGrey.h"
#include "odometry.h"
#include "automation.h"

//Interrupt function definitions
bool ps2Ready = false;
bool imuReady = false;
bool powerOffPressed = false;
void slowTimerHandler();
void ps2Activated();
void ps2Deactivated();
void imuActivated();
void imuDeactivated();

/* Driving variables */
struct unicycleState stopUniCycleState;
struct differentialState stopState;
struct differentialState curDiffState;
struct differentialState desiredDiffState;
float desiredHeading = 0.0;
float headingCorrection = 0;
float headingError = 0;
float prev_desiredPhi = 0;
float prev_vy = 0;

float arenaJunction[5] = {350.0,550.0,750.0,950.0,1150.0};

/* PS2 variables */
bool rotateWasPressed = false;

/* Tiva packet variables */
extern int shoot,load,planeAngle;
//extern float rpmpercent1;
int prev_right_analog_stick = 128, right_analog_stick = 128;
int desiredHeight = 0;
int maxPWM = 0, minPWM = 0;
struct encoder *encoderheight;

/* Line Sensor variables */
struct lineSensor lsF,lsB;
float lsF_error = 0, lsF_prev_error = 0;
float lsB_error = 0, lsB_prev_error = 0;

/* Odometry variables */
struct encoder *encoder1;
struct encoder *encoder2;
float distTravelled=0.0;
int rpiPort;

void reset() {
//	resetDriveConfig();
//	resetAutomation();
//	resetEncoder(encoderheight);
//	resetOdometry(encoder1,encoder2);
//	resetPS2_2k17();
//	resetPIDvar(headingControl);
//	resetPIDvar(lineControl_fw);
//	resetPIDvar(lineControl_bw);
//	resetPIDvar(heightControl);
//	resetPIDvar(odometryControl);

/* Driving variables reset */	
//	desiredHeading = 0.0;
//	headingCorrection = 0;
//	headingError = 0;
//	prev_desiredPhi = 0;
//	prev_vy = 0;
/* PS2 variables  reset */
//	rotateWasPressed = false;
/* Tiva packet variables reset */
//	prev_right_analog_stick = 128;
//	right_analog_stick = 128;
//	desiredHeight = 0;
/* Line Sensor variables reset */
//	lsF_error = 0;
//	lsF_prev_error = 0;
//	lsB_error = 0;
//	lsB_prev_error = 0;
/* Odometry variables reset */
//	distTravelled=0.0;
}

char encodeByte(int rpm) {
    if(rpm > maxRPM) {
        printf("Warning: Trying to send Velocity greater than 180. Limit : 180. Sending 180.\r\n");
        rpm = 100; //must be even (maxRPM)
    } else if(rpm < -(maxRPM + 1)) {
        printf("Warning: Trying to send Velocity lesser than -181. Limit : -181. Sending -181.\r\n");
        rpm = -99; // must be odd
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


//Function to read linesensors and preprocess it.
void lineFeedback(void) {
	if(digitalRead(lsF.NANDoutPin)) {
		lsF_error = -readLineSensor(lsF); //- because the sensors are flipped
	}
	if(lsF_error == -255) {
		if(lsF_prev_error < 0) {
			lsF_error = -50;
		} else if(lsF_prev_error > 0) {
			lsF_error = 50;
		} else {
			lsF_error = 0;
		}
	}
	lsF_prev_error = lsF_error;
	usleep(1000);	//readings skew if this is removed. Need further study : Aniket,20 October,2016
	if(digitalRead(lsB.NANDoutPin)) {
		lsB_error = -readLineSensor(lsB);
	}
	if(lsB_error == -255) {
		if(lsB_prev_error < 0) {
			lsB_error = -50;
		} else if(lsB_prev_error > 0) {
			lsB_error = 50;
		} else {
			lsB_error = 0;
		}
	}
	lsB_prev_error = lsB_error;
}

void initHeightControl() {
//	encoderheight = setupencoder(2,3);	
	pinMode(heightMotorPin1,OUTPUT);
	pinMode(heightMotorPin2,OUTPUT);
	softPwmCreate(heightMotorPWM,0,255);
	pinMode(relayButton,OUTPUT);
	digitalWrite(relayButton,1);
	digitalWrite(heightMotorPin1,0);
	digitalWrite(heightMotorPin2,0);
	maxPWM = 128;
	minPWM = 5;
}

//Function for manual driving with heading control
struct unicycleState getDesiredUnicycleState_manual(void) {
	struct unicycleState desiredState;
	rotateCheck();
	if(rotatePressed) {
	        desiredState = rotateBot();
			rotateWasPressed = true;
	} else {
		float vy, vx;
		desiredState.vy = (128 - ps2_getY()) * maxVelocity / 128;
		dir_log(desiredState.vy);
		desiredState.vx = 0;
		if(rotateWasPressed == true) {
			if(desiredState.vy == 0) {
				return stopUniCycleState;
			} else {
				headingOffset = getHeading();
				rotateWasPressed = false;
			}
		}
		float desiredPhi;
		if(desiredState.vy == 0 && desiredState.vx == 0) {
			desiredPhi = prev_desiredPhi;
		} else {
			desiredPhi = radianToDegree((PI/2) - atan2(abs(desiredState.vy), desiredState.vx)); //Absolute of vy so that for negative v the angle does not go -ve and the robot does not turn. We want it to drive backwards
		}
		prev_desiredPhi = desiredPhi;
		float error = desiredPhi - getHeading() + headingOffset;
		error = majorAngleFilter(error);
		(fabs(error) > HEADING_TOLERANCE) ? desiredState.w = PID(error, headingControl) : desiredState.w = 0;
	}
	return desiredState;
}

//Function for semiautonomous driving with heading control and linesensors
struct unicycleState getDesiredUnicycleState_line(void) {
	struct unicycleState desiredState;
	float vy, vx;
	lineFeedback();
	printf("%f %f ;\n",lsF_error,lsB_error);
	desiredState.vx = 0;
	desiredState.vy = ((128 - ps2_getY())/128.0) * maxVelocity;
	dir_log(desiredState.vy);
	if(desiredState.vy > 0) {
		desiredState.w = PID(lsF_error, lineControl_fw);
		prev_vy = desiredState.vy;
	} else if(desiredState.vy < 0) {
		desiredState.w = PID(lsB_error, lineControl_bw);
		prev_vy = desiredState.vy;
	} else {
		if(prev_vy >= 0) {
			desiredState.w = PID(lsF_error, lineControl_fw);
		} else {
			desiredState.w = PID(lsB_error, lineControl_bw);
		}
	}
	return desiredState;
}

//Function for autonomous driving with heading control and linesensors
struct unicycleState getDesiredUnicycleState_auto(void) {
	struct unicycleState desiredState;
	float vy, vx;
	lineFeedback();
	printf("%f %f ;",lsF_error,lsB_error);
	desiredState.vx = 0;
	desiredState.vy = velocityMap();
	dir_log(desiredState.vy);
	if(desiredState.vy > 0) {
		desiredState.w = PID(lsF_error, lineControl_fw);
		prev_vy = desiredState.vy;
	} else if(desiredState.vy < 0) {
		desiredState.w = PID(lsB_error, lineControl_bw);
		prev_vy = desiredState.vy;
	} else {
		if(prev_vy >= 0) {
			desiredState.w = PID(lsF_error, lineControl_fw);
		} else {
			desiredState.w = PID(lsB_error, lineControl_bw);
		}
	}
	printf(" %f %f; Last Junction = %d Desired Junction = %d \n",desiredState.vy,desiredState.vx,lastJunction,desiredJunction);
	return desiredState;
}

/**********************************************************************************************/
/**  Function for mode enabled driving  **/
/**********************************************************************************************/

struct unicycleState getDesiredUnicycleState_mode() {
    int botMode = getMode();
    modeChange();
    switch(botMode) {
            case 0: 
                    return getDesiredUnicycleState_manual();
                    break;
            case 1: 
		    return getDesiredUnicycleState_line();
                    break;
            case 2: 
		    return getDesiredUnicycleState_auto();
                    break;
    }
}

void setPWM(float pwm, int i) {
	if (pwm > maxPWM) {
		pwm = maxPWM;
	} 
	if (pwm < -maxPWM) {
		pwm = -maxPWM;
	}
	printf("%f is pwm;",pwm);
	if(i == heightControl) {
		if(fabs(pwm) < HEIGHT_TOLERANCE) {
			digitalWrite(heightMotorPin1,1);
			digitalWrite(heightMotorPin2,1);
			softPwmWrite(heightMotorPWM,minPWM);
		} else if(pwm > 0) {
			if(pwm < minPWM) {
				pwm = minPWM;
			}
			softPwmWrite(heightMotorPWM,0);			//Low PWM because for going downwards, gravity accelerates it
			digitalWrite(heightMotorPin1,1);
			digitalWrite(heightMotorPin2,0);		
		} else if (pwm < 0) {
			pwm = -pwm;
			if(pwm < minPWM) {
				pwm = minPWM;
			}
			softPwmWrite(heightMotorPWM,pwm);
			digitalWrite(heightMotorPin1,0);
			digitalWrite(heightMotorPin2,1);
		}
	}
}

void timerHandler() {
	if(!ps2Ready /*|| !imuReady*/) {
		transmitDiffState(stopState);
	} else {
/*  Mode control Driving */
		desiredDiffState = transformUniToDiff(getDesiredUnicycleState_mode());
		transmitDiffState(desiredDiffState);
	 	printf("%d %d %f\n",desiredDiffState.leftRPM,desiredDiffState.rightRPM,getHeading());

/*  Height control */
		right_analog_stick = ps2_getRY();
printf("%d is ps2;",right_analog_stick);
//		if(right_analog_stick == 128 && prev_right_analog_stick != 128) {
//			desiredHeight = encoderheight->value;
//		}
		if(right_analog_stick == 128) {
			digitalWrite(relayButton,1);
//			float heightError = (encoderheight->value)- desiredHeight ;
//			float out = PID(heightError,heightControl);
			setPWM(0,heightControl);
//			printf("0 :: \n");						
		} else {
			digitalWrite(relayButton,0);
			setPWM((ps2_getRY()-128)*0.75,heightControl);
//			printf("1 :: \n");						
		}
		prev_right_analog_stick = right_analog_stick;

/*Odometry*/
//		distTravelled=getDistTravelled();
//		printf("%d  %d\n",encoder1->value,encoder2->value);
//		printf("x=%f,y=%f,phi=%f \n",getX(distTravelled),getY(distTravelled),180.0*getPhi()/pi);
/* Saving data and status */
//		std::ofstream data_file;
//		data_file.open("data.log",std::ios::app);
//		data_file << desiredDiffState.leftRPM << ";" << desiredDiffState.rightRPM << ";" << getHeading() << ";\n";
//		data_file.close();
	}
	transmitMechanismControl();
	if(mechanismState() == false) {
		printf("Drive enable :: ");
	} else {
		printf("Mechanism enable :: ");
	}
//	printf(	"%d %d %d :: %d ::rmp:%f\n",printer_shoot,printer_load,printer_planeAngle,printer_dataFrame,rpmpercent1);
}

void init() {
   	stopState.leftRPM = 0;
	stopState.rightRPM = 0;
	stopUniCycleState.vx = 0;
	stopUniCycleState.vy = 0;
	stopUniCycleState.w = 0;

	if(wiringPiSetup() < 0) {
		printf("Error setting up while setting wiringPi\n");
	}

/* Button and LED configurations */
	if(wiringPiISR(headingRefButton, INT_EDGE_RISING, &reset)) {
		printf("Reset Heading Button interrupt setup error\n");
	}
/* PS2 and IMU configuration */
	enablePS2StatusInterrupt(&ps2Activated, &ps2Deactivated);
//	enableIMUStatusInterrupt(&imuActivated, &imuDeactivated);
	enableSlowFuncInterrupt(&slowTimerHandler);
	
/* Odometry parameters and pin init */
//	encoder1 = setupencoder(21,22);	
//	encoder2 = setupencoder(23,24);
//	initOdometry(encoder1,encoder2);

//Height control pin init
	initHeightControl();

	initPS2();
	initPS2_2k17();		//Robocon 2k17 PS2 configuration
//	initIMU();

/* Initialize mechanism control system	*/
	initMechanism();
}

//Interrupt on Junction occurence
void junctionInterruptF(void) {
/*	if(intF_flag){
		if(forward) {
			lastJunction++;
			writeOdometry(arenaJunction[lastJunction-1] - distFromLS,Y);
		}
	}
	intF_flag = true;
*/
//	printf("Junction interruptF fired\n");
}

void junctionInterruptB(void) {
/*	if(intB_flag){
		if(reverse) {
			lastJunction--;
			writeOdometry(arenaJunction[lastJunction-1] - distFromLS,Y);
		}
	}
	intB_flag = true;
*/
//	printf("Junction interruptB fired\n");
}

int main() {
	init();
	rpiPort = serialOpen("/dev/ttyS0",38400);			/*Serial communication port established*/
	initPIDController(0.05,0.0,0.02,headingControl);		/*PID controller for angular velocity in manual mode*/  //0.05,0.02
	initPIDController(0.01,0.0,0.22,lineControl_fw);	//0.03,0.0,1.2	/*PID controller for angular velocity in linefollow_fw mode*/
	initPIDController(0.01,0.0,0.22,lineControl_bw);	//0.03,0.0,1.2	/*PID controller for angular velocity in linefollow_bw mode*/
	initPIDController(0.16,0.0,0.0,heightControl);
	initPIDController(1.0,0.0,0.0,odometryControl);

	lsF.address = 2;
	lsF.uartPort = rpiPort;
	lsF.UARTPin = 6;
	lsF.junctionPin = 13;
	lsF.NANDoutPin = 1;

	lsB.address = 1;
	lsB.uartPort = rpiPort;
	lsB.UARTPin = 12;
	lsB.junctionPin = 5;
	lsB.NANDoutPin = 0;

	initLineSensor(lsF, &junctionInterruptF);
	initLineSensor(lsB, &junctionInterruptB);

	initTimer(1000000/PIDfrequency, &timerHandler);
	while(1) {
		sleep(1);
	}
}





