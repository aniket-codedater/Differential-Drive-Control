#ifndef driveConfig
#define driveConfig

/* Tolerance */
#define HEADING_TOLERANCE 2
#define HEIGHT_TOLERANCE 5

/* GPIO pin def */
#define headingRefButton 7
#define heightMotorPin1 27
#define heightMotorPin2 29
#define heightMotorPWM 	28
#define relayButton 26

extern bool ps2Ready;
extern bool imuReady;
extern bool powerOffPressed;

extern bool rotatePressed;
extern bool rotateDirection;

extern int mode;
extern int curMode;
extern int preMode;

extern int desiredJunction;
extern int lastJunction;
extern float arenaJunction[];

enum {clk,antiClk};			//Rotate direction enum
enum {left, right}; 			//Wheel enum
enum {headingControl,lineControl_fw,lineControl_bw,heightControl,odometryControl};	//PID controller enum

struct differentialState {int leftRPM; int rightRPM;};

//Bot specifications
#define wheelRadius 				5.0
#define wheelCircumference 			(2 * PI * wheelRadius)
#define L					96.0
#define wRotate 				1.0
#define distFromLS				80.0

//Motion constraints specifications
#define maxRPM			252	//in rpm
#define maxVelocity		180 //in cm/s

//Frequency specifications
#define PIDfrequency	40

void resetDriveConfig();

#endif
