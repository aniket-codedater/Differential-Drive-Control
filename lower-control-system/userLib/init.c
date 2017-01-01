#include "init.h"

void motorDirInit(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIOPinTypeGPIOOutput(motorDirectionRegister, A1|A2|B1|B2);
}

void timerInit(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE,TIMER_CFG_A_PERIODIC);
	TimerLoadSet(TIMER0_BASE,TIMER_A,(uint32_t)(SysCtlClockGet()/PIDfrequency));
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE, TIMER_A);
}

void uart0Init(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 38400,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void uart1Init(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 38400,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void uart5Init(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinConfigure(GPIO_PE4_U5RX);
	GPIOPinConfigure(GPIO_PE5_U5TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	UARTConfigSetExpClk(UART5_BASE, SysCtlClockGet(), 38400,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	IntEnable(INT_UART5);
	UARTIntEnable(UART5_BASE, UART_INT_RX);
}

void pwmInit(void) {
	//PB5 for motor A
	//PB4 for motor B
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	GPIOPinConfigure(GPIO_PB5_M0PWM3);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, SysCtlClockGet()/(2*PWMfrequency));
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

void qeiInit(void) {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6);
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_7);
	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);
	QEIConfigure(QEI0_BASE,QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_RESET | QEI_CONFIG_NO_SWAP,3999);
	QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,SysCtlClockGet()/QEIfrequency);
	QEIVelocityEnable(QEI0_BASE);
	QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
	IntEnable(INT_QEI0);
	QEIEnable(QEI0_BASE);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);
	QEIConfigure(QEI1_BASE,QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_RESET | QEI_CONFIG_NO_SWAP,3999);
	QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1,SysCtlClockGet()/QEIfrequency);
	QEIVelocityEnable(QEI1_BASE);
	QEIIntEnable(QEI1_BASE, QEI_INTTIMER);
	IntEnable(INT_QEI1);
	QEIEnable(QEI1_BASE);

}
