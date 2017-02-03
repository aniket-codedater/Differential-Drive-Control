#ifndef LOAD_H
#define LOAD_H

#include "userLib/common.h"
#include "userLib/servo.h"

#define LIMIT_ENABLE	0
#define SERVO_ANGLE1	45.0
#define SERVO_ANGLE2	135.0
#define SERVO_DELAY		15000000
enum {up, down};

void resetLoad(void);
void loadInit(void);
int8_t reload(void);
void reload_manual(uint8_t mech_no, uint8_t dir);
int8_t moveLoader(uint8_t i,uint8_t startColor,uint8_t dir);
void checkForSafetyTrigger(uint8_t mech_no,uint8_t dir);
int IRstateConfidenceCheck(uint8_t mech_no);
void bring_system_to_0_from_top(uint8_t mech_no);
void bring_system_to_0_from_bottom(uint8_t mech_no);

#endif
