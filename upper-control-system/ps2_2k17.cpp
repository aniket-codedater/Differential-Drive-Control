#include "ps2USB.h"
#include "ps2_2k17.h"
#include "driveConfig.h"
#include "controlmath.h"
#include "minimu9.h"
#include <wiringPi.h>
#include <stdio.h>
#include "mechanismConfig.h"
#define NUMBER_OF_FAST_BUTTON 4
#define FAST_SEND_COUNTER_CONFIDENCE 50
float headingOffset = 0.0;
int shoot = 0, load = 0, planeAngle = 0;
int fast_send_counter[NUMBER_OF_FAST_BUTTON]={0};
enum {square_enum,circle_enum,r1_enum,r2_enum,triangle_enum,cross_enum};
enum {manual_enum,semiauto_enum,auto_enum};
bool buttonState[3][6] = {false};
bool decremental = false;
float rpmpercent1=1.0;
bool mechanismEnable = false;
int mechanismMode = manual_enum; 

void resetPS2_2k17() {
	mode = 0;
	mechanismMode = manual_enum;
	rotatePressed = 0;
	rotateDirection = clk;
	desiredJunction = 0;
	lastJunction = 0;
	curMode = 0;
	preMode = 0;
	headingOffset = 0.0;
	shoot = 0;
	load = 0;	
	planeAngle = 0.0;
	for(int i = 0; i < NUMBER_OF_FAST_BUTTON; i++) {
		fast_send_counter[i] = 0;
	}
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 6; j++) {
			buttonState[i][j] = false;
		}
	}
	decremental = false;
	mechanismEnable = false;
}

bool mechanismState() {
	return mechanismEnable;
}

int getMechanismMode(void) {
	if(mechanismState() == true) {
		return mechanismMode;
	} else {
		return -1;
	}
}

void writeDesiredJunction(int junc) {
	if(junc != -1) {
		desiredJunction = junc;
	}
}

int ps2Manager(int junction,int buttonEnum ,bool buttonPressState, int pole) {
	switch(getMechanismMode()) {
		//Driving mode
		case -1:		
			switch(mode) {
				case auto_enum:
					if(buttonPressState == true) {
				        writeDesiredJunction(junction);
					}
		        	break;
			}
			break;
		//Mechanism mode
		case manual_enum:
			if(buttonPressState) {
				buttonState[manual_enum][buttonEnum] = true;
				return 0;
			} else {
				buttonState[manual_enum][buttonEnum] = false;
				transmitMechanismPacket(0,0,noChangeAngle,noChangeRPM,noChangePos);
			}
			break;
		case auto_enum:
			if(buttonPressState == true) {
		        transmitMechanismPacket(pole);
			}
			break;
	}
	return 1;
}

/* Mode selector */
void (L2_Pressed)() {
	if(getMechanismMode() == -1) {
		mode = auto_enum;
	} else {
		mode = manual_enum;
		mechanismMode = auto_enum;
	}
}

void (L2_Released)() {
	if(getMechanismMode() == -1) {
		mode = manual_enum;
	} else {
		mode = manual_enum;
		mechanismMode = manual_enum;
	}
}

void (L1_Pressed)() {
	switch(getMechanismMode()) {
		case -1:
			mode = semiauto_enum;
			break;
		case manual_enum:
			break;
		case auto_enum:
			transmitMechanismPacket(6);
			break;
	}
}

void (L1_Released)() {
	switch(getMechanismMode()) {
		case -1:
			mode = manual_enum;
			break;
		case manual_enum:
			break;
		case auto_enum:
			break;
	}
}

/* Square functions :
* Mode 0 : Increase/Decrease thrower position *
*/

void (square_Pressed)() {
	ps2Manager(1,square_enum,true,0);
}

void (square_Released)() {
	ps2Manager(1,square_enum,false,0);
}

/* Triangle functions :
* Mode 0 : Shoot enable *
*/
void (triangle_Pressed)() {
	if(ps2Manager(2,triangle_enum,true,1) == 0) {
	    transmitMechanismPacket(1,0,noChangeAngle,noChangeRPM,noChangePos);
	}
}

void (triangle_Released)() {
	ps2Manager(2,triangle_enum,false,1);
}


/* Circle functions :
* Mode 0 : Increase/Decrease plane angle *
*/

void (circle_Pressed)() {
	ps2Manager(3,circle_enum,true,2);
}

void (circle_Released)() {
	ps2Manager(3,circle_enum,false,2);
}

/* Cross functions :
* Mode 0 : Load enable*
*/
void (cross_Pressed)() {
	if(ps2Manager(4,cross_enum,true,3) == 0) {
		transmitMechanismPacket(0,1,noChangeAngle,noChangeRPM,noChangePos);	
	}
}

void (cross_Released)() {
	ps2Manager(4,cross_enum,false,3);
}

/* R1 functions :
* Mode 0 : Incremental/Decremental selector *
*/
void (R1_Pressed)() {
	ps2Manager(5,r1_enum,true,4);
	if(buttonState[manual_enum][r1_enum] == true) {
		decremental = true;
	}
}

void (R1_Released)() {
	ps2Manager(5,r1_enum,false,4);
	if(buttonState[manual_enum][r1_enum] == false) {
		decremental = false;
	}
}

void (R2_Pressed)() {
	ps2Manager(-1,r2_enum,true,5);
}

void (R2_Released)() {
	ps2Manager(-1,r2_enum,false,5);
}

void (R3_Pressed)() {
}

void (R3_Released)() {
}

void (L3_Pressed)() {
}

void (L3_Released)() {
}

void (start_Pressed)() {
}

void (start_Released)() {
}

void (select_Pressed)() {
	mechanismEnable = !mechanismEnable;
}

void (select_Released)() {
}


void initPS2_2k17() {
	enableL1Button(&L1_Pressed,&L1_Released);
	enableL2Button(&L2_Pressed,&L2_Released);
	enableCircleButton(&circle_Pressed, &circle_Released);
	enableSquareButton(&square_Pressed, &square_Released);
   	enableCrossButton(&cross_Pressed, &cross_Released);
	enableTriangleButton(&triangle_Pressed, &triangle_Released);
   	enableR1Button(&R1_Pressed, &R1_Released);
   	enableR2Button(&R2_Pressed, &R2_Released);
	enableR3Button(&R3_Pressed, &R3_Released);
	enableL3Button(&L3_Pressed, &L3_Released);
	enableStartButton(&start_Pressed, &start_Released);
	enableSelectButton(&select_Pressed, &select_Released);
}


void modeChange() {
    curMode = getMode();
    if(curMode!=preMode) {
//	headingOffset = getHeading();
        switch(curMode) {
            case 0:
            case 1: 
            	break;
            case 2: 
            	desiredJunction = lastJunction;
                break;
        }
    }
    preMode = curMode;
}

void rotateCheck() {
	if(ps2_getY() - 128 == 0) {
		switch(ps2_getX() - 128) {
			case 0:
				rotatePressed = 0;
				break;
			case -128:
				rotatePressed = 1;
				rotateDirection = clk;
				break;
			case 127:
				rotatePressed = 1;
				rotateDirection = antiClk;
				break;
		}
	} else {
		rotatePressed = 0;
	}
}

struct unicycleState rotateBot() {
    if(rotateDirection==clk) {
        return rotateClk();
    } else if(rotateDirection==antiClk){
        return rotateAnticlk();
    }
}

struct unicycleState rotateClk() {
	struct unicycleState clkUniState;
	clkUniState.vx = 0.0;
	clkUniState.vy = 0.0;
	clkUniState.w = wRotate;
	return clkUniState;
}

struct unicycleState rotateAnticlk() {
	struct unicycleState anticlkUniState;
	anticlkUniState.vx = 0.0;
	anticlkUniState.vy = 0.0;
	anticlkUniState.w = -wRotate;
	return anticlkUniState;
}

int getMode() {
    return mode;
}

bool buttonFastSend(bool buttonState,int button)
{
	if(buttonState == 1) {
		fast_send_counter[button]++;
		if(fast_send_counter[button] > FAST_SEND_COUNTER_CONFIDENCE || fast_send_counter[button] == 1) {
			return true;
		}
		return false;
	} else {
		fast_send_counter[button] = 0;	
		return false;
	}	
}

void transmitMechanismControl(){
     if((buttonState[manual_enum][triangle_enum] == false) && (buttonState[manual_enum][cross_enum] == false)) {
        if(buttonFastSend(buttonState[manual_enum][circle_enum],circle_enum)) {					//Circle
        	if(decremental == false) {
				transmitMechanismPacket(0,0,increaseAngle,noChangeRPM,noChangePos);
			} else {
		        	transmitMechanismPacket(0,0,decreaseAngle,noChangeRPM,noChangePos);
			}		
        } else if(buttonFastSend(buttonState[manual_enum][square_enum],square_enum)) {          //Square
        	if(decremental == false) {
				transmitMechanismPacket(0,0,noChangeAngle,noChangeRPM,increasePos);        		
			} else {
	        	transmitMechanismPacket(0,0,noChangeAngle,noChangeRPM,decreasePos);
			}		
        } else if(buttonFastSend(buttonState[manual_enum][r2_enum],r2_enum)) {                 //R2
            if(decremental == false) {
				if(rpmpercent1>1)
				rpmpercent1=1;
				else if(rpmpercent1<1)
				rpmpercent1+=0.05;
				transmitMechanismPacket(0,0,noChangeAngle,increaseRPM,noChangePos); 
				       		
			} else {
				if(rpmpercent1 > 0.01)	{
					rpmpercent1 -= 0.05;
				}
				else {
					rpmpercent1 = 0.1;
				}				
	        	transmitMechanismPacket(0,0,noChangeAngle,decreaseRPM,noChangePos);
			}		
        }
     }
}






