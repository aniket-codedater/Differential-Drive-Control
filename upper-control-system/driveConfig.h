#ifndef driveConfig
#define driveConfig

/*Buttons config*/
#define powerOffButton 4
#define headingRefButton 0
#define ps2InputLED 25
#define headingLED 29
#define slowLoopLED 27
#define miscLED 28

extern bool ps2Ready;
extern bool imuReady;
extern bool powerOffPressed;

#define HEADING_TOL 2

enum {left, right}; //wheel
enum {headingControl,lineControl_fw,lineControl_bw};

struct differentialState {int leftRPM; int rightRPM;};

//Bot specifications
#define wheelRadius 		4.854
#define wheelCircumference 	(2 * PI * wheelRadius)
#define L			49.0
#define botRadius 		1.0

//Motion constraints specifications
#define maxRPM			252.0	//in rpm
#define maxVelocity		120.0 //in cm/s

//Frequency specifications
#define PIDfrequency	40

#endif
