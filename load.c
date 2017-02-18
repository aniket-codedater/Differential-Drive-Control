#include "load.h"

volatile int8_t start_color,transition_color;
volatile uint8_t reload_in_progress = 0;
volatile uint8_t no_of_discs_loaded = 0;
volatile uint8_t system_going_0_from_top = 0;

bool limitFlag[2][2] = {{false,false},{false,false}};
int8_t servoID[2];

void resetLoad(void) {
	reload_in_progress = 0;
	no_of_discs_loaded = 0;
	system_going_0_from_top = 0;
}

void checkForSafetyTrigger(uint8_t mech_no,uint8_t dir) {
	uint32_t limitPort;
	uint32_t limitPinUp,limitPinDown;
	if(mech_no == loader1) {
		limitPort = LIMIT1_SWITCH_PORT;
		limitPinUp = LIMIT1_SWITCH_PIN2;
		limitPinDown = LIMIT1_SWITCH_PIN1;
	} else if(mech_no == loader2) {
		limitPort = LIMIT2_SWITCH_PORT;
		limitPinUp = LIMIT2_SWITCH_PIN2;
		limitPinDown = LIMIT2_SWITCH_PIN1;
	}
	if(dir == up) {
		if(GPIOPinRead(limitPort,limitPinUp) == 0) {
			limitFlag[mech_no][dir] = false;
		} else {
			limitFlag[mech_no][dir] = true;
		}
	} else if(dir == down) {
		if(GPIOPinRead(limitPort,limitPinDown) == 0) {
			limitFlag[mech_no][dir] = false;
		} else {
			limitFlag[mech_no][dir] = true;
		}
	}
}

void loadInit(void) {
	//gpio
	SYSCTLPERIPH_IR1;
	SYSCTLPERIPH_IR2;
	SYSCTLPERIPH_LOAD_MOTOR1;
	SYSCTLPERIPH_LOAD_MOTOR2;
	GPIOPinTypeGPIOInput(IR1_PORT,IR1_PIN);
	GPIOPinTypeGPIOInput(IR2_PORT,IR2_PIN);
	GPIOPinTypeGPIOOutput(LOAD_MOTOR1_PORT,LM11|LM12);
	GPIOPinTypeGPIOOutput(LOAD_MOTOR2_PORT,LM21|LM22);
	//gpio for solenoid valve
	SYSCTLPERIPH_SOLENOID;
	GPIOPinTypeGPIOOutput(SOLENOID_PORT,SOLENOID_PIN);
	GPIOPinWrite(SOLENOID_PORT,SOLENOID_PIN,(1<<SOLENOID_PIN_MASK));
	//limit switch
	SYSCTLPERIPH_LIMIT1_SWITCH;//limit switch for mech 1
	GPIOPinTypeGPIOInput(LIMIT1_SWITCH_PORT, LIMIT1_SWITCH_PIN1|LIMIT1_SWITCH_PIN2);
	SYSCTLPERIPH_LIMIT2_SWITCH;//limit switch for mech 2
	GPIOPinTypeGPIOInput(LIMIT2_SWITCH_PORT, LIMIT2_SWITCH_PIN1|LIMIT2_SWITCH_PIN2);

	//servo init
	servoID[loader1] =	servoInit(SERVO1_PIN,COUNT_DOWN,50);
	moveServo(SERVO_ANGLE2,servoID[loader1]);
	//servoID[loader2] = servoInit(SERVO2_PIN,COUNT_DOWN,50);
	//moveServo(SERVO_ANGLE2,servoID[loader2]);
}

int8_t moveLoader(uint8_t i,uint8_t startColor,uint8_t dir) {
	uint32_t motorPort;
	uint32_t motorPin1,motorPin2;
	uint8_t motorPin1Mask,motorPin2Mask;
	uint32_t stopColor;
	if(startColor == (WHITE1 || WHITE2)) {
		stopColor = BLACK;
	} else if(startColor == BLACK) {
		if(i == loader1) {
			stopColor = WHITE1;
		} else if(i == loader2) {
			stopColor = WHITE2;
		}
	}
	if(i == loader1) {
		motorPort = LOAD_MOTOR1_PORT;
		motorPin1 = LM11;
		motorPin2 = LM12;
		motorPin1Mask = LM11_MASK;
		motorPin2Mask = LM12_MASK;
	} else if(i == loader2) {
		motorPort = LOAD_MOTOR2_PORT;
		motorPin1 = LM21;
		motorPin2 = LM22;
		motorPin1Mask = LM21_MASK;
		motorPin2Mask = LM22_MASK;
	}
	transition_color = -1;
	while(transition_color == -1) {
		transition_color = IRstateConfidenceCheck(i);
	}
	while((transition_color != stopColor)) {
		transition_color = -1;
		while(transition_color == -1) {
			transition_color = IRstateConfidenceCheck(i);
		}
#if	LIMIT_ENABLE == 1
		checkForSafetyTrigger(i,dir);
		if(limitFlag[i][dir] == true) {
			if(dir == up) {
				bring_system_to_0_from_top(i);
			} else {
				bring_system_to_0_from_bottom(i);
			}
			return -1;
		}
#endif
		if(dir == up) {
			GPIOPinWrite(motorPort,motorPin1|motorPin2,(1<<motorPin2Mask));//move up
		} else if(dir == down) {
			GPIOPinWrite(motorPort,motorPin1|motorPin2,(1<<motorPin1Mask));//move down
		}
	}
	GPIOPinWrite(motorPort,motorPin1|motorPin2,(1<<motorPin1Mask)|(1<<motorPin2Mask));//stop
	return 1;
}

int IRstateConfidenceCheck(uint8_t mech_no)
{
	long int black_state_confidence = 0;
	long int white_state_confidence = 0;
	long int black_confidence_level = 100;
	long int white_confidence_level = 100;

	if(system_going_0_from_top == 1)
	{
		black_confidence_level = 45000;
		white_confidence_level = 45000;
	}

	uint32_t irPort;
	uint32_t irPin;
	uint8_t startColor;
	if(mech_no == loader1)
	{
		irPort = IR1_PORT;
		irPin = IR1_PIN;
		startColor = WHITE1;
	} else if(mech_no == loader2) {
		irPort = IR2_PORT;
		irPin = IR2_PIN;
		startColor = WHITE2;
	}
	while(GPIOPinRead(irPort,irPin) == startColor)
	{
		white_state_confidence++;
		black_state_confidence = 0;
		if(white_state_confidence >= white_confidence_level) {
			black_state_confidence = 0;
			white_state_confidence = 0;
			return startColor;
		}
	}
	while(GPIOPinRead(irPort,irPin) == BLACK) {
		black_state_confidence++;
		white_state_confidence = 0;
		if(black_state_confidence >= black_confidence_level) {
			black_state_confidence = 0;
			white_state_confidence = 0;
			return BLACK;
		}
	}
	return -1;
}

void reload_manual(uint8_t loader_,uint8_t dir) {
	if(loader_ == loader1) {
		if(dir == up) {
			GPIOPinWrite(LOAD_MOTOR1_PORT,LM11| LM12,(1<<LM12_MASK));//move up
			SysCtlDelay(3000000);
			GPIOPinWrite(LOAD_MOTOR1_PORT,LM11| LM12 ,(1<<LM11_MASK)|(1<<LM12_MASK));//stop
		} else if(dir == down) {
			GPIOPinWrite(LOAD_MOTOR1_PORT,LM11| LM12,(1<<LM11_MASK));//move down
			SysCtlDelay(3000000);
			GPIOPinWrite(LOAD_MOTOR1_PORT,LM11| LM12 ,(1<<LM11_MASK)|(1<<LM12_MASK));//stop
		}
	} else if(loader_ == loader2) {
		if(dir == up) {
			GPIOPinWrite(LOAD_MOTOR2_PORT,LM21| LM22,(1<<LM22_MASK));//move up
			SysCtlDelay(3000000);
			GPIOPinWrite(LOAD_MOTOR2_PORT,LM21| LM22 ,(1<<LM21_MASK)|(1<<LM22_MASK));//stop
		} else if(dir == down) {
			GPIOPinWrite(LOAD_MOTOR2_PORT,LM21| LM22,(1<<LM21_MASK));//move down
			SysCtlDelay(3000000);
			GPIOPinWrite(LOAD_MOTOR2_PORT,LM21| LM22 ,(1<<LM21_MASK)|(1<<LM22_MASK));//stop
		}
	}
}

int8_t reload(void)
{
	uint8_t loaderID;
	reload_in_progress = 1;
	GPIOPinWrite(SOLENOID_PORT,SOLENOID_PIN,0); 						//Piston up

//	if(no_of_discs_loaded < MAX_LOAD_DISK ) {							//Select loader
		loaderID = loader1;
//	} else if(no_of_discs_loaded >= MAX_LOAD_DISK ) {
//		loaderID = loader2;
//	}
	start_color = -1;
	while(start_color == -1) {
		start_color = IRstateConfidenceCheck(loaderID);							//Start color detection
	}
	if(moveLoader(loaderID,start_color,up) == -1) { 		//Move loader
		return -1;
	}
	no_of_discs_loaded++;
	moveServo(SERVO_ANGLE2,servoID[loaderID]);									//Servo hit
	SysCtlDelay(SERVO_DELAY);
	moveServo(SERVO_ANGLE1,servoID[loaderID]);
	SysCtlDelay(SERVO_DELAY);
	moveServo(SERVO_ANGLE2,servoID[loaderID]);
	SysCtlDelay(SERVO_DELAY);

	GPIOPinWrite(SOLENOID_PORT,SOLENOID_PIN,(1<<SOLENOID_PIN_MASK));	//Piston down
/*
	if(no_of_discs_loaded >= (2*MAX_LOAD_DISK)) {
		bring_system_to_0_from_top(loader2);
		no_of_discs_loaded = 0;
	} else if(no_of_discs_loaded >= MAX_LOAD_DISK) {
		bring_system_to_0_from_top(loader1);
	}
*/
	reload_in_progress = 0;
	return 1;
}


void bring_system_to_0_from_top(uint8_t mech_no)
{
	int count = 0;
	system_going_0_from_top = 1;
	while(count < MAX_LOAD_DISK) {
		start_color = -1;
		while(start_color == -1) {
			start_color = IRstateConfidenceCheck(mech_no);				//Start color detection
		}
		moveLoader(mech_no,start_color,down);
		SysCtlDelay(10000000);
		count++;
	}
	GPIOPinWrite(LOAD_MOTOR1_PORT,LM11| LM12,(1<<LM11_MASK));//move down
	SysCtlDelay(1500000);
	GPIOPinWrite(LOAD_MOTOR1_PORT,LM11| LM12 ,(1<<LM11_MASK)|(1<<LM12_MASK));//stop
	system_going_0_from_top = 0;
}

void bring_system_to_0_from_bottom(uint8_t mech_no) {
	int count = 0;
	while(count < 1) {
		start_color = -1;
		while(start_color == -1) {
			start_color = IRstateConfidenceCheck(mech_no);				//Start color detection
		}
		moveLoader(mech_no,start_color,up);
		SysCtlDelay(10000000);
		count++;
	}
	GPIOPinWrite(LOAD_MOTOR1_PORT,LM11| LM12,(1<<LM11_MASK));//move down
	SysCtlDelay(1500000);
	GPIOPinWrite(LOAD_MOTOR1_PORT,LM11| LM12 ,(1<<LM11_MASK)|(1<<LM12_MASK));//stop
}

