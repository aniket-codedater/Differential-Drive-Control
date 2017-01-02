#ifndef MECHANISM_CONFIG_H
#define MECHANISM_CONFIG_H

#define USBTTLPORT "/dev/ttyUSB0"

extern int printer_shoot,printer_load,printer_planeAngle,printer_dataFrame;

void initMechanism();
void transmitMechanismControl(int throw_, int load_, int planeAngle_, int rpmChange_, int posChange_);

#endif
