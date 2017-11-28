//
// GPIO_7seg_keypad : 3x3 keypad inpt and display on 7-segment LEDs
//
#include <stdio.h>
#include "NUC100Series.h"
#include "MCU_init.h"
#include "SYS_init.h"
#include "Seven_Segment.h"
#include "Scankey.h"

// display an integer on four 7-segment LEDs
void Display_7seg(uint16_t value)
{
  uint8_t digit;
	digit = value / 1000;
	CloseSevenSegment();
	if(digit==0){
		
	}else{
		ShowSevenSegment(3,digit);		
	}
	
	CLK_SysTickDelay(3000);
			
	value = value - digit * 1000;
	digit = value / 100;
	CloseSevenSegment();
		if(digit==0){
		
	}else{
		ShowSevenSegment(2,digit);		
	}
	CLK_SysTickDelay(3000);

	value = value - digit * 100;
	digit = value / 10;
	CloseSevenSegment();
		if(digit==0){
		
	}else{
		ShowSevenSegment(1,digit);		
	}
	CLK_SysTickDelay(3000);

	value = value - digit * 10;
	digit = value;
	CloseSevenSegment();
		if(digit==0){
		
	}else{
		ShowSevenSegment(0,digit);		
	}
	CLK_SysTickDelay(3000);
}

int main(void)
{
	
	while(1) 
		{
				uint16_t i;
				i=ScanKey();
				
					SYS_Init();

					OpenSevenSegment();
					OpenKeyPad();
			
					/*if(!i){
						Display_7seg(33);
								CloseSevenSegment();
								ShowSevenSegment(3,9);
								CLK_SysTickDelay(3000);
							  CloseSevenSegment();
								ShowSevenSegment(2,0);
								CLK_SysTickDelay(3000);
							 	CloseSevenSegment();
								ShowSevenSegment(1,3);
								CLK_SysTickDelay(3000);
							 	CloseSevenSegment();
								ShowSevenSegment(0,3);
								CLK_SysTickDelay(3000);
						printf("Hello World");

					
					}else{
							Display_7seg(i);
							//printf("Hello World");
					}*/


					
					   if (i==1)
						   {		
								 Display_7seg(i);
						   }
						 
						 else if(i==2)
							 {
								Display_7seg(i);

							 }

						 else if(i==3)
							 {
								Display_7seg(i);

							 }
						 
						 else if(i==4)
							 {
								Display_7seg(i);

							 }
						 else if(i==5)
							 {
								Display_7seg(i);

							 }						 
						 else if(i==6)
							 {
								Display_7seg(i);

							 }						 
						 else if(i==7)
							 {
								Display_7seg(i);

							 }						 
						 else if(i==8)
							 {
								Display_7seg(i);

							 }	
						 else if(i==9)
							 {
								Display_7seg(i);

							 }				 
						 
					 else
						 {
							 Display_7seg(33);
					 
						 }
			 
			 
	  }
}
  