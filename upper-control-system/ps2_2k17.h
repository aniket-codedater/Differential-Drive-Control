#ifndef ps2_2k17
#define ps2_2k17

#include "driveConfig.h"

extern void transmitDiffState(struct differentialState desiredDiffState);

extern void (*circlePressed)(void);
extern void (*circleReleased)(void);
extern void (*squarePressed)(void);
extern void (*squareReleased)(void);
extern void (*crossPressed)(void);
extern void (*crossReleased)(void);
extern void (*trianglePressed)(void);
extern void (*triangleReleased)(void);
extern void (*L1Pressed)(void);
extern void (*L1Released)(void);
extern void (*L2Pressed)(void);
extern void (*L2Released)(void);
extern void (*L3Pressed)(void);
extern void (*L3Released)(void);
extern void (*R1Pressed)(void);
extern void (*R1Released)(void);
extern void (*R2Pressed)(void);
extern void (*R2Released)(void);
extern void (*R3Pressed)(void);
extern void (*R3Released)(void);
extern void (*startPressed)(void);
extern void (*startReleased)(void);
extern void (*selectPressed)(void);
extern void (*selectReleased)(void);

extern bool rotatePressed;
extern bool rotateDirection;
extern int mode;
extern float desiredJunction;
extern float lastJunction;
extern int curMode;
extern int preMode;

int getMode(void);
void modeChange(void);
void rotateBot(void);
void initPS2_2k17(void);
void rotateClk(void);
void rotateAnticlk(void);

#endif





