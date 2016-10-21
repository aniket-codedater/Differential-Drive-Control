#ifndef userlib_interrupt
#define userlib_interrupt
#include "driveConfig.h"
#include <wiringPi.h>
#include <unistd.h>
#include <stdio.h>
#include "ps2USB.h"
#include "minimu9.h"
#include "timerInterrupt.h"

extern void slowTimerHandler();
extern void ps2Activated();
extern void ps2Deactivated();
extern void imuActivated();
extern void imuDeactivated();

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

#endif
