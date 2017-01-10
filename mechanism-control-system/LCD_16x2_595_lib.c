//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//includes
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "LCD_16x2_595_lib.h"

//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//Variables
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//------------------------------------------------------------------------------
//Global
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//Local
//------------------------------------------------------------------------------
static unsigned char UpperNibble, LowerNibble, bitsHigh, bitsLow,temp;
int l = 0;
//'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//Local Functions declaration
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void PulseEnable ();
void shiftOut(unsigned char c);
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//Function descriptions
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//Global Functions
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void LCD_16x2_595_Periph_Init()
{

		SysCtlPeripheralEnable(lcd_SysCtlPeripheral);

		GPIOPinTypeGPIOOutput(lcd_port, Enable | DATA | CLOCK);

		GPIOPinWrite(lcd_port, (Enable | DATA | CLOCK), 0x00);

}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void LCD_16x2_595_Command_Init()
{
	SysCtlDelay(delay_12ms);
	shiftOut(0x03);
	PulseEnable ();
	SysCtlDelay(delay_2_5ms);

	shiftOut(0x03);
	PulseEnable ();
	SysCtlDelay(delay_2_5ms);

	shiftOut(0x03);
	PulseEnable ();
	SysCtlDelay(delay_2_5ms);

	shiftOut(0x02);
	PulseEnable ();
	SysCtlDelay(delay_2_5ms);

	LCD_595_Cmd(0x28);  //Function set - number of display 2 lines

	LCD_595_Cmd(0x08);  //Display off

	LCD_595_Cmd(0x01);  //Display clear

	LCD_595_Cmd(0x0F);  //Cursor - Display on/off control
}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
// to send command
void LCD_595_Cmd(unsigned char c)
{
    UpperNibble = c >> 4;
    LowerNibble = c & 0x0F;

    shiftOut(UpperNibble);
    PulseEnable ();

    shiftOut(LowerNibble);
    PulseEnable ();

    SysCtlDelay(delay_2_5ms);
}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//to print character
void LCD_595_Chr_Cp(unsigned char c)
{

    UpperNibble = c >> 4;
    bitsHigh = UpperNibble | 0x10;// 0x10 =>this is done to make RS pin high while sending data
    LowerNibble = c & 0x0F;
    bitsLow = LowerNibble | 0x10;


     shiftOut(bitsHigh);
     PulseEnable ();

     shiftOut(bitsLow);
     PulseEnable ();
     l++;

}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//to print a string
void sendAstring(char *string_of_character){
	while(*string_of_character>0){
		LCD_595_Chr_Cp(*string_of_character);
		*string_of_character++;
	}
}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void LCD_595_Chr(char row, char column, char out_char)
{
	Position_LCD(row,column);
	LCD_595_Chr_Cp(out_char);
	l=0;
}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void LCD_595_text(char row, char column, char *text)
{
	Position_LCD(row,column);
	l = 0;
	send_to_next_line(text);
}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//Local Functions
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void PulseEnable ()
{
	GPIOPinWrite(lcd_port, Enable, 0x01);//EN = 1;
	SysCtlDelay(delay_0_6us);
	GPIOPinWrite(lcd_port, Enable, 0x00);//EN = 0;
}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void shiftOut(unsigned char c)
{
   int i;
   for (i=7;i>=0;i--)
   {
	   GPIOPinWrite(lcd_port, CLOCK, 0x00);//CLOCK = 0
      if (c & (1<<i))
      {
       GPIOPinWrite(lcd_port, DATA, 0x02);//DATA = 1
      }
      else
      {
       GPIOPinWrite(lcd_port, DATA, 0x00);//DATA = 0
      }
       GPIOPinWrite(lcd_port, CLOCK, 0x20);//CLOCK = 1
   }

   SysCtlDelay(delay_12us);
   GPIOPinWrite(lcd_port, CLOCK, 0x00); //CLOCK = 0
   SysCtlDelay(delay_12us);
   GPIOPinWrite(lcd_port, CLOCK, 0x20); //CLOCK = 1
}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void Position_LCD(unsigned char x, unsigned char y)
{
	 temp = 127 + y;
	 if (x == 2) temp = temp + 64;
	 LCD_595_Cmd(temp);
	 l = 0;
}
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
//to print number
void send_number(unsigned long n) {
	unsigned long val = n;
	int arr[10];
	int len = 0;
	int j = 0,r = 0;
	while(val!=0) {
		 r = val%10;
		arr[len] = r;
		val = val/10;
		len++;
	}
	for( j=(len-1);j>=0;j--) {
		LCD_595_Chr_Cp(arr[j]+'0');
	}
}
//  ENF OF FILE
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
void send_to_next_line(char *ch)
{
    while(*ch!='\0'){
			if(l < 16){
				LCD_595_Chr_Cp(*ch);
			}
			else{
				if (l == 16)
				{
					LCD_595_Cmd(0xC0);
				}

				LCD_595_Chr_Cp(*ch);
			}
			ch++;
			l++;
		}
}

void sendFloat_number( double n){
	int len1 = 0,len2 = 0;
    int32_t temp = n;
    float difference = n - (float)temp;
    int32_t num = difference*1000.0;
    int32_t val1 = temp,val2=num;
    while(val1!=0){
    	val1 = val1/10;
    	len1++;
    }
    while(val2!=0){
    	val2 = val2/10;
    	len2++;
    }
    l = l + len1 + len2;
    if(l > 16){
    	l = l - 16;
    }
    if(l < 16){
    	send_number(temp);
    	LCD_595_Chr_Cp('.');
    	send_number(num);
    	l = l + 1;
    }
    else{
    	if(l >= 16){
    		LCD_595_Cmd(0xC0);
    		l = l - 16;
    	}
    	send_number(temp);
       	LCD_595_Chr_Cp('.');
       	send_number(num);
    }
}
void LCD_Print( char* format,...){
	char *traverse;
	int i;
	char ch;
	char *s;
	double fl;

	va_list arg;
	va_start(arg,format);

	for(traverse = format; *traverse != '\0'; traverse++)
	    {
	        while( *traverse != '%' && *traverse != '\0')
	        {
	            LCD_595_Chr_Cp(*traverse);
	            traverse++;
	            l++;
	        }
	        if(*traverse != '\0')
	        {
	            traverse++;
	        } else {
	            break;
	        }
			switch(*traverse){
			case 'd': i = va_arg(arg,int);
					  if(i<0)
					  {
						  i = -i;
						  LCD_595_Chr_Cp('-');
					  }
					  send_number(i);
					  break;

			case 'c': ch = va_arg(arg,char);
					  LCD_595_Chr_Cp(ch);
					  break;

			case 's': s = va_arg(arg,char*);
					  send_to_next_line(s);
					  break;

			case 'f': fl = va_arg(arg,double);
						if(fl<0.0)
					  {
						  fl = -fl;
						  LCD_595_Chr_Cp('-');
					  }
					  sendFloat_number(fl);
					  break;
			}
	    }
	va_end(arg);

}
