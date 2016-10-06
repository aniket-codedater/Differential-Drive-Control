#ifndef driveConfig
#define driveConfig


// software var
#define pi					3.1416
#define	timeInterval				0.03264				// 1024 * 255 / F_CPU // in sec


// hardware var
#define ticksPerRotation			720
#define r					5.0
#define L					47.5
#define circumference				31.4				//2 * pi * r
#define vmax					300
#define powerOffButton 4
#define headingRefButton 0
#define ps2InputLED 25
#define headingLED 29
#define slowLoopLED 27
#define miscLED 28

#define HEADING_TOL 2

enum {left, right}; //wheel
enum {lRPM, rRPM, angularVel}; // movingArray, PID   //Why PID on lRPM and rRPM?
enum {headingControl};

struct differentialState {int leftRPM; int rightRPM;};

//Bot specifications
#define wheelRadius 5.0
#define wheelCircumference (2 * PI * wheelRadius)

#define botRadius  1500

//Motion constraints specifications
#define maxRPM			252	//in rpm
#define maxVelocity		300 //in cm/s

//Frequency specifications
#define PIDfrequency	40

#endif







