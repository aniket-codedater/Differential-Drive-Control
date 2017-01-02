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
enum {manual,semiauto,auto_};
bool buttonState[3][6] = {false};
bool decremental = false;

void resetPS2_2k17() {
	mode = 0;
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
}

void (L1_Pressed)() {
	mode = 1;
}

void (L1_Released)() {
	mode = 0;
}


void (L2_Pressed)() {
	mode = 2;
}

void (L2_Released)() {
	mode = 0;
}

/* Circle functions :
* Mode 0 : Increase/Decrease plane angle *
*/

void (circle_Pressed)() {
	switch (mode) {
		case 0:
			buttonState[manual][circle_enum] = true;
			break;
		case 2:
	        desiredJunction = 1.0;
			break;
	}
}

void (circle_Released)() {
     	buttonState[manual][circle_enum] = false;
	transmitMechanismControl(0,0,noChangeAngle,noChangeRPM,noChangePos);
}

/* Square functions :
* Mode 0 : Increase/Decrease thrower position *
*/

void (square_Pressed)() {
        switch (mode) {
		case 0:
		        buttonState[manual][square_enum] = true;
			break;
		case 2:
	        desiredJunction = 3.0;
		break;
	}
}

void (square_Released)() {
     	buttonState[manual][square_enum] = false;
	transmitMechanismControl(0,0,noChangeAngle,noChangeRPM,noChangePos);
}

/* Cross functions :
* Mode 0 : Load enable*
*/
void (cross_Pressed)() {
	switch (mode) {
		case 0 :
            buttonState[manual][cross_enum] = true;
            transmitMechanismControl(0,1,noChangeAngle,noChangeRPM,noChangePos);
			break;
		case 2:
	        desiredJunction = 4.0;
		break;
	}
}

void (cross_Released)() {
     buttonState[manual][cross_enum] = false;
     transmitMechanismControl(0,0,noChangeAngle,noChangeRPM,noChangePos);
}

/* Triangle functions :
* Mode 0 : Shoot enable *
*/
void (triangle_Pressed)() {
	switch (mode) {
		case 0:
            buttonState[manual][triangle_enum] = true;
            transmitMechanismControl(1,0,noChangeAngle,noChangeRPM,noChangePos);
		case 2:
	        desiredJunction = 2.0;
		break;
	}
}

void (triangle_Released)() {
     	buttonState[manual][triangle_enum] = false;
	transmitMechanismControl(0,0,noChangeAngle,noChangeRPM,noChangePos);
}

/* R1 functions :
* Mode 0 : Incremental/Decremental selector *
*/
void (R1_Pressed)() {
	switch (mode) {
		case 0:
			decremental = true;
			break;
		case 2:
	        desiredJunction = 5.0;
			break;
	}
}

void (R1_Released)() {
	decremental = false;
}

void (R2_Pressed)() {
	buttonState[manual][r2_enum] = true;
}

void (R2_Released)() {
	buttonState[manual][r2_enum] = false;
	transmitMechanismControl(0,0,noChangeAngle,noChangeRPM,noChangePos);
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
            case 1: break;
            case 2: desiredJunction = lastJunction;
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

void transmitMechanismControl_Packet(){
     if((buttonState[manual][triangle_enum] == false) && (buttonState[manual][cross_enum] == false)) {
        if(buttonFastSend(buttonState[manual][circle_enum],circle_enum)) {					//Circle
        	if(decremental == false) {
				transmitMechanismControl(0,0,increaseAngle,noChangeRPM,noChangePos);
			} else {
	        		transmitMechanismControl(0,0,decreaseAngle,noChangeRPM,noChangePos);
			}		
        } else if(buttonFastSend(buttonState[manual][square_enum],square_enum)) {          //Square
        	if(decremental == false) {
				transmitMechanismControl(0,0,noChangeAngle,noChangeRPM,increasePos);        		
			} else {
	        		transmitMechanismControl(0,0,noChangeAngle,noChangeRPM,decreasePos);
			}		
        } else if(buttonFastSend(buttonState[manual][r2_enum],r2_enum)) {                 //R2
            if(decremental == false) {
				transmitMechanismControl(0,0,noChangeAngle,increaseRPM,noChangePos);        		
			} else {
	        		transmitMechanismControl(0,0,noChangeAngle,decreaseRPM,noChangePos);
			}		
        }
     }
}