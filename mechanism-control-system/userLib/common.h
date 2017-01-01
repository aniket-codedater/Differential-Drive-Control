#ifndef common
#define common

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"

enum {angle_motor,throw_motor,revolution};
extern int32_t maxPWM_angle,maxPWM_throw, minPWM_throw,minPWM_angle;

#define PIDfrequency 						100
#define PWMfrequency 						100

#define SYSCTLPERIPH_THROW_MOTOR		 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
#define THROW_MOTOR_PORT 					GPIO_PORTA_BASE
#define TM1 								GPIO_PIN_6
#define TM2 								GPIO_PIN_7
#define TM1_MASK 							0b01000000
#define TM2_MASK 							0b10000000
#define SYSCTLPERIPH_THROW_ENC				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
#define THROW_ENCODER_PORT					GPIO_PORTF_BASE
#define THROW_ENCODER_CHANNELA				GPIO_PIN_0
#define THROW_ENCODER_CHANNELB				GPIO_PIN_1
#define THROW_ENCODER_CHANNELA_PINCONFIG	GPIO_PF0_PHA0
#define THROW_ENCODER_CHANNELB_PINCONFIG	GPIO_PF1_PHB0
//PWM pin for throwing motor PE5

#define SYSCTLPERIPH_ANGLE_MOTOR 			SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
#define ANGLE_MOTOR_PORT 					GPIO_PORTE_BASE
#define AM1 								GPIO_PIN_2
#define AM2 								GPIO_PIN_3
#define AM1_MASK 							0b00000100
#define AM2_MASK 							0b00001000
#define SYSCTLPERIPH_ANGLE_ENC 				SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
#define ANGLE_ENCODER_PORT					GPIO_PORTF_BASE
#define ANGLE_ENCODER_CHANNELA				GPIO_PIN_2
#define ANGLE_ENCODER_CHANNELB				GPIO_PIN_3
//PWM pin for angle motor  PD0

void setPWM(int32_t pwm,int i);
unsigned absolute(float x);
void UART_TransmitString(char* string,int i);
void UART_OutDec(int x, int i);

#endif
