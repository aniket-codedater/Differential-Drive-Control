#include "userLib/common.h"
#include "userLib/init.h"
#include "userLib/pidController.h"
#include "userLib/movingArray.h"
#include <stdbool.h>
#include "angle.h"
#include "shoot.h"
#include "load.h"
#include "automation.h"
#include "LCD_16x2_595_lib.h"

int32_t maxPWM_throw = 10,maxPWM = 0;							//Random maxPWM value
int32_t maxPWM_angle = 10;							//Random maxPWM value
int32_t minPWM_throw = 0;						//Minimum PWM value for the throw motor to move
int32_t minPWM_angle = 0;						//Minimum PWM value for the angle motor to move

volatile float shootPercent = 1.0;
volatile int shoot = 0,load = 0,planeAngle = 0;			//UART packet value holders of the mechanism
volatile int8_t enablePositionChange = 0;
bool loadEnable = false; 
bool autonomous_mode = false;
SET_VALUE GET_PARAM;
int POLE;

//Interrupt routines prototype
void Timer0IntHandler(void);
void ISR_ANGLE(void);
void UARTIntHandler(void);

//Parameter changer
float setShootPercent(float i) {
    shootPercent = i;
    if(shootPercent > 1.0) {
        shootPercent = 1.0;
    }
    if(shootPercent < 0.1) {
        shootPercent = 0.1;
    }
    maxPWM_throw = maxPWM * shootPercent;
    return shootPercent;
}

int setPlaneAngle(int i) {
    planeAngle = i;
    if(planeAngle > 15) {
        planeAngle = 15;
    }
    if(planeAngle < -15) {
        planeAngle = -15;
    }
    cmd_angle(planeAngle);
    return planeAngle;
}

void setThrowerPosition(long int i) {
    loadPoint();
    des_throw_counter = throw_counter + i;

}

long int getThrowerPosition(void) {
    return QEIPositionGet(QEI0_BASE);
}

//Debugging variables
int printer;
long int printer_step,printer_first,printer_second;

int main() {
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	Lcd_16x2_595_init();
	initPIDController(throw_motor,0.08,0.0,0.0); //0.04
	initPIDController(angle_motor,9.0,0.0,0.0);
	IntMasterEnable();
	uart0Init();
	UARTFIFODisable(UART0_BASE);
	IntEnable(INT_UART0);
	UARTIntEnable(UART0_BASE, UART_INT_RX);
	Lcd_Print("system started");
	UART_TransmitString("System started.\r\n",0);
	uart5Init();
	maxPWM = SysCtlClockGet()/(PWMfrequency*8);
	maxPWM_throw = maxPWM;
	maxPWM_angle = maxPWM;
	minPWM_throw = maxPWM/20.0;             //16.0
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
        SysCtlDelay(5000000);
        Lcd_clearScreen();
	    if(loadEnable == false){
	        if(shootComplete == 0){
	            Lcd_Print("SHOOT");
	        }
	        else{
	            Lcd_Print("PA %d ",planeAngle);
	            Lcd_Print("RPM %f ",shootPercent*100);
	            Lcd_newLine();
	            Lcd_Print("TA %f ",throw_angle);
	        }
	    }
	    else{
            Lcd_Print("RELOAD");
	    }

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
        UART_OutDec(maxPWM_throw,0);
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
   TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
//   UART_TransmitString("inside timer\n",0);
   throw_counter = getThrowerPosition() ;
   throw_angle = convertTicksToAngle(throw_counter);
    if(loadEnable == false) {
        if(enablePositionChange == 0) {
            shootDisc(!shootComplete);
        } else {
            if(enablePositionChange == 1) {         //Clock
                setPWM(minPWM_throw,throw_motor);
            } else if(enablePositionChange == -1) { //Anticlock
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
	/* Mode selection */
	if((data & 0b11111000) == 0b11111000){
	    autonomous_mode = true;
	} else {
	    autonomous_mode = false;
	}
	/* Manual mode */
	if(autonomous_mode == false){
		uint8_t tempPlaneAngle = 0, tempRpm = 0, tempPosChange = 0;
		shoot = (data & 0b10000000);
		load = (data & 0b01000000);
		if(load != LOAD_DISC) {
			/*Plane angle routine*/
			tempPlaneAngle = (data & 0b00110000)>>4;
			switch(tempPlaneAngle) {
			case 1:
				planeAngle++;
				break;
			case 2:
				planeAngle--;
				break;
			}
			planeAngle = setPlaneAngle(planeAngle);
	
			/*RPM change routine*/
			tempRpm = (data & 0b00001100)>>2;
			switch(tempRpm) {
			case 1:
				shootPercent += 0.0125;
				break;
			case 2:
				shootPercent -= 0.0125;
				break;
			}
			shootPercent = setShootPercent(shootPercent);
	
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
				}
			} else if (shoot == 0) {
				triggered=0;
			}

		} else {
			loadEnable = true;
	        reload();
			int confidenceCheck = 0;
			while(confidenceCheck < LOAD_POSITION_CONFIDENCE) {
				throw_counter = QEIPositionGet(QEI0_BASE);
				if(moveThrower(des_throw_counter) == 1) {
					confidenceCheck++;
				} else {
					confidenceCheck = 0;
				}
			}
			loadEnable = false;
		}
	}
	else{
	    POLE = (data & 0b00000111);
	    GET_PARAM = SET_PARAMETERS(POLE);
	    planeAngle = GET_PARAM.PLANE_ANGLE;
	    shootPercent = GET_PARAM.SHOOT_PERCENT;
	    maxPWM = maxPWM * shootPercent;
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
	} else if(char_data == 's') {
        UART_TransmitString("Servo angle 1 :: \r\n",0);
	    moveServo(135,0);                                  //Servo hit
	} else if(char_data == 'a') {
        UART_TransmitString("Servo angle 2 :: \r\n",0);
        moveServo(45,0);                                  //Servo hit
    }
}
