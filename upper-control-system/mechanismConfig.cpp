#include "mechanismConfig.h"
#include <stdio.h>
#include <wiringSerial.h>
#include <fstream>
#include <istream>

int atmegaPort;
int printer_shoot = 0,printer_load = 0,printer_planeAngle = 0,printer_dataFrame = 0;
void initMechanism() {
	atmegaPort = serialOpen(USBTTLPORT,38400);
}

void transmitMechanismControl(int shoot_, int load_, int planeAngle_, int rpmChange_, int posChange_) {
	unsigned char dataFrame = 0;
	dataFrame |= shoot_ << 7;
	dataFrame |= load_ << 6;
	dataFrame |= planeAngle_ << 4;
	dataFrame |= rpmChange_ << 2;
	dataFrame |= posChange_;
	if((dataFrame&0xC0) == 0xC0) {
		dataFrame = 0x80|planeAngle_;	
	}
	serialPutchar(atmegaPort, dataFrame);
	printer_shoot = shoot_;
	printer_load = load_;
	printer_planeAngle = planeAngle_;
	printer_dataFrame = dataFrame;
//	printf("%d, %d, %d, ::%d ",shoot_,load_,planeAngle_,dataFrame);
//	std::ofstream data_file;
//	data_file.open("data.log",std::ios::app);
//	data_file << "Mechanism status:" << dataFrame << ";\n" ;
//	data_file.close();
}
