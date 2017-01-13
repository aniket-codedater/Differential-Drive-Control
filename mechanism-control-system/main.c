#include "userLib/common.h"
#include "userLib/init.h"
#include "userLib/pidController.h"
#include "userLib/movingArray.h"
#include <stdbool.h>
#include "angle.h"
#include "shoot.h"
#include "load.h"
int32_t maxPWM_throw = 10,maxPWM = 0;							//Random maxPWM value
int32_t maxPWM_angle = 10;							//Random maxPWM value
int32_t minPWM_throw = 0;						//Minimum PWM value for the throw motor to move
int32_t minPWM_angle = 0;						//Minimum PWM value for the angle motor to move

volatile float shootPercent = 1.0;
volatile int shoot = 0,load = 0,planeAngle = 0;			//UART packet value holders of the mechanism
volatile int8_t enablePositionChange = 0;
bool loadEnable = false;
//Interrupt routines prototype
void Timer0IntHandler(void);
void ISR_ANGLE(void);
void UARTIntHandler(void);

//Debugging variables
int printer;
long int printer_step,printer_first,printer_second;

int main() {
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	initPIDController(throw_motor,0.08,0.0,0.0); //0.16
	initPIDController(angle_motor,9.0,0.0,0.0);
	IntMasterEnable();
	uart0Init();
	UARTFIFODisable(UART0_BASE);
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX);
	UART_TransmitString("System started.\r\n",0);
	uart5Init();
	maxPWM = SysCtlClockGet()/(PWMfrequency*8);
	maxPWM_throw = maxPWM;
	maxPWM_angle = maxPWM;
	minPWM_throw = maxPWM/20.0;
	minPWM_angle = maxPWM/20.0;
	maxPWM_throw = maxPWM * shootPercent;	// 0.7 times for middle pole at 90 degree
	pwmInit();
	motorDirInit(angle_motor);
	motorDirInit(throw_motor);
	qeiInit();
	encoderInit(ISR_ANGLE,angle_motor);
	loadInit();
	timerInit();
	while(1) {
		UART_OutDec(angle_counter,0);
		UARTCharPut(UART0_BASE,';');
// 	 	UART_OutDec(shoot,0);
//		UARTCharPut(UART0_BASE,';');
		UART_OutDec(des_angle_counter,0);
		UARTCharPut(UART0_BASE,';');
//		UART_OutDec(planeAngle,0);
//		UARTCharPut(UART0_BASE,';');
		UART_OutDec(printer,0);
        UARTCharPut(UART0_BASE,';');
        UART_OutDec(des_throw_counter,0);
		UARTCharPut(UART0_BASE,';');
		UART_OutDec(throw_counter,0);
		UARTCharPut(UART0_BASE,';');
        UART_OutDec(printer,0);
        UARTCharPut(UART0_BASE,';');
        UART_OutDec(printer_step,0);
        UARTCharPut(UART0_BASE,';');
        UART_OutDec(printer_first,0);
        UARTCharPut(UART0_BASE,';');
        UART_OutDec(printer_second,0);
		UART_TransmitString("\r\n",0);
	}
}

void Timer0IntHandler(void) {
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	throw_counter = QEIPositionGet(QEI0_BASE);
	if(loadEnable == false) {
		if(enablePositionChange == 0) {
            shootDisc();
		} else {
			if(enablePositionChange == 1) {			//Clock
				setPWM(minPWM_throw,throw_motor);
			} else if(enablePositionChange == -1) {	//Anticlock
				setPWM(-minPWM_throw,throw_motor);
			}
			updateDesiredStage();
			updateFirstStage();
		}
		changeAngle();
	}
}

void UARTIntHandler(void) {
	uint32_t ui32Status;
	ui32Status = UARTIntStatus(UART5_BASE, true); //get interrupt status
	UARTIntClear(UART5_BASE, ui32Status); //clear the asserted interrupts
	char data = UARTCharGet(UART5_BASE);//HWREG(UART5_BASE + UART_O_DR);
	/*Data via bitmasking*/
	uint8_t tempPlaneAngle = 0,tempRpm = 0,tempPosChange = 0;
	shoot = (data & 0b10000000);
	load = (data & 0b01000000);
	if(load != LOAD_DISC) {
		/*Plane angle routine*/
		tempPlaneAngle = (data & 0b00110000)>>4;
		switch(tempPlaneAngle) {
		case 1:
			planeAngle++;
			if(planeAngle > 15) {
				planeAngle = 15;
			}
			break;
		case 2:
			planeAngle--;
			if(planeAngle < -15) {
				planeAngle = -15;
			}
			break;
		}
		cmd_angle(planeAngle);

		/*RPM change routine*/
		tempRpm = (data & 0b00001100)>>2;
		switch(tempRpm) {
		case 1:
			shootPercent += 0.05;
			if(shootPercent > 1.0) {
				shootPercent = 1.0;
			}
			break;
		case 2:
			shootPercent -= 0.05;
			if(shootPercent < 0.1) {
				shootPercent = 0.1;
			}
			break;
		}
		maxPWM_throw = maxPWM * shootPercent;

		/*Throw position change routine*/
		tempPosChange = (data & 0b00000011);
		switch(tempPosChange) {
		case 0:
			enablePositionChange = 0;
			break;
		case 1:
			enablePositionChange = 1;
			break;
		case 2:
			enablePositionChange = -1;
			break;
		default:
			enablePositionChange = 0;
			break;
		}

		/*Shoot Disc*/
		if (shoot == SHOOT_DISC) {
			if (shootComplete == 1 && triggered == 0)
			{
				cmd_throw();
				triggered = 1;
				shootComplete = 0;
				steady_state_counter = 0;
			}
		} else if (shoot == 0) {
			triggered=0;
		}
	} else {
		loadEnable = true;
		int confidenceCheck = 0;
		while(confidenceCheck < LOAD_POSITION_CONFIDENCE) {
			throw_counter = QEIPositionGet(QEI0_BASE);
			if(moveThrower(loadPoint()) == 1) {
				confidenceCheck++;
			} else {
				confidenceCheck = 0;
			}
		}
		reload();
		loadEnable = false;
	}
}

void ISR_ANGLE(void) {
	GPIOIntClear(ANGLE_ENCODER_PORT,ANGLE_ENCODER_CHANNELB);
	if(GPIOPinRead(ANGLE_ENCODER_PORT,ANGLE_ENCODER_CHANNELA)) {
		angle_counter++;
	} else {
		angle_counter--;
	}
}

void UART0Handler(void) {
	uint32_t ui32Status;
	ui32Status = UARTIntStatus(UART0_BASE, true); //get interrupt status
	UARTIntClear(UART0_BASE, ui32Status); //clear the asserted interrupts
	char char_data = UARTCharGet(UART0_BASE);
	if(char_data == 'u') {
		reload_manual(loader1,up);
	} else if(char_data == 'd') {
		reload_manual(loader1,down);
	} else if(char_data == 'r') {
		if(reload() == -1) {
			UART_TransmitString("Err :: Limit error\r\n",0);
		} else {
			UART_TransmitString("Loading :: \r\n",0);
		}
	} else if(char_data == 'b') {
		bring_system_to_0_from_bottom(loader1);
	}
}
