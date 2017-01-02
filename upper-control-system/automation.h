#ifndef AUTOMATION_H
#define AUTOMATION_H

#include "driveConfig.h"
#include "odometry.h"
#include <cmath>
#include "pidController.h"

extern bool odometryEnable;
extern bool intF_flag, intB_flag;
extern int desiredJunction, lastJunction;
extern bool prevForward;
extern bool forward,reverse;

float velocityMap(void);
void dir_log(float velocity);
void resetAutomation(void);

#endif
