#ifndef controlmath
#define controlmath

#define PI 3.1415926
#include "driveConfig.h"

struct point { float x; float y; };
struct unicycleState {float v; float w;};
struct position {float x; float y; int phi;};


float sigmoid(float x);
float degreeToRadian(float degree);
float radianToDegree(float radian);
float normalizeAngle(float degree);
struct differentialState transformUniToDiff(struct unicycleState unistate); //alpha is the angular offset given to the robot
struct unicycleState getDesiredUnicycleState( struct position curBotPosition, struct position desiredBotPosition) ;

#endif
