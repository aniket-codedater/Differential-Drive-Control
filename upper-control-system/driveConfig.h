#ifndef driveConfig
#define driveConfig

/*Buttons config*/
#define powerOffButton 			4
#define headingRefButton 		0
#define ps2InputLED 			25
#define headingLED 				29
#define slowLoopLED 			27
#define miscLED 				28
#define HEADING_TOL 			2
#define wRotate 				10.0

extern bool ps2Ready;
extern bool imuReady;
extern bool powerOffPressed;
extern bool rotatePressed;
extern bool rotateDirection;
extern int mode;
extern int curMode;
extern int preMode;
extern float desiredJunction;
extern float lastJunction;

enum{clk,antiClk};
enum {left, right}; //wheel
enum {headingControl,lineControl_fw,lineControl_bw};

struct differentialState {int leftRPM; int rightRPM;};

//Bot specifications
#define wheelRadius 10.0
#define wheelCircumference (2 * PI * wheelRadius)

#define botRadius  1

//Motion constraints specifications
#define maxRPM			252	//in rpm
#define maxVelocity		260 //in cm/s

//Frequency specifications
#define PIDfrequency	40

#endif
