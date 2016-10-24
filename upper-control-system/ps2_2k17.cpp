#include "ps2USB.h"
#include "ps2_2k17.h"
#include "driveConfig.h"
#include "controlmath.h"


void initPS2_2k17() {
	enableL1Button(L1Pressed,L1Released);
	enableL2Button(L2Pressed,L2Released);
	enableCircleButton(circlePressed, circleReleased);
	enableSquareButton(squarePressed, squareReleased);
    	enableCrossButton(crossPressed, crossReleased);
	enableTriangleButton(trianglePressed, triangleReleased);
    	enableR1Button(R1Pressed, R1Released);
}


void modeChange() {
    curMode = getMode();
    if(curMode!=preMode) {
        switch(curMode) {
            case 0:
            case 1: break;
            case 2: desiredJunction = lastJunction;
                    break;
        }
    }
    preMode = curMode;
}

void rotateBot() {
    if(rotateDirection==clk) {
        rotateClk();
    } else if(rotateDirection==antiClk){
        rotateAnticlk();
    }
}

void rotateClk() {
	struct unicycleState clkUniState;
	clkUniState.vx = 0.0;
	clkUniState.vy = 0.0;
	clkUniState.w = wRotate;
	transmitDiffState(transformUniToDiff(clkUniState));
}

void rotateAnticlk() {
	struct unicycleState anticlkUniState;
	anticlkUniState.vx = 0.0;
	anticlkUniState.vy = 0.0;
	anticlkUniState.w = -wRotate;
	transmitDiffState(transformUniToDiff(anticlkUniState));
}

int getMode() {
    return mode;
}

/*
void mechAngleSet() {
}

void mechElevationSet() {
}

void shootDisc() {
}

void discLoad() {
}
*/













