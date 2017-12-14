
/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 1 $
 * $Date: 14/12/08 11:49a $
 * @brief    Implement timer counting in periodic mode.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"
#include <string.h>
#include "GPIO.h"
/***********Scankey.h********/
#ifndef SCANKEY_H
#define SCANKEY_H

#define GPIO_MODE_QUASI         0x3UL /*!< Quasi-bidirectional Mode */

/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

void delay(void)
{
	int j;
	for(j=0;j<1000;j++);
}

void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0; i < GPIO_PIN_MAX; i++)
    {
        if(u32PinMask & (1 << i))
        {
            port->PMD = (port->PMD & ~(0x3 << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

void OpenKeyPad(void)
{
  GPIO_SetMode(PA, BIT0, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT1, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT2, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT3, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT4, GPIO_MODE_QUASI);
  GPIO_SetMode(PA, BIT5, GPIO_MODE_QUASI);						
}

uint8_t ScanKey(void)
{
  PA0=1; PA1=1; PA2=0; PA3=1; PA4=1; PA5=1;
	if (PA3==0) return 1;
	if (PA4==0) return 4;
	if (PA5==0) return 7;
  PA0=1; PA1=0; PA2=1; PA3=1; PA4=1; PA5=1;
	if (PA3==0) return 2;
	if (PA4==0) return 5;
	if (PA5==0) return 8;
  PA0=0; PA1=1; PA2=1; PA3=1; PA4=1; PA5=1;
	if (PA3==0) return 3;
	if (PA4==0) return 6;
	if (PA5==0) return 9;
	return 0;
}



void OpenKeyPad(void);
void CloseKeyPad(void);
uint8_t ScanKey(void);

#endif
/************************/

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

#define RXBUFSIZE 1024


uint8_t g_u8RecData[RXBUFSIZE]  = {0};

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
//int32_t main(void);
//void UART_TEST_HANDLE(void);
//void UART_FunctionTest(void);



/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};
volatile uint32_t u32InitCount;

char stop0=0x11,stop1=0x11;

void TMR0_IRQHandler(void)
{
		uint8_t num;
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);
				if(stop0==0x55){

				
				}else{
					g_au32TMRINTCount[0]++;
					g_au32TMRINTCount[0]++;
				
				}

    }
}
void TMR1_IRQHandler(void)
{
		uint8_t num;
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
				if(stop1!=0x01){
					g_au32TMRINTCount[1]++;
					g_au32TMRINTCount[1]++;
					g_au32TMRINTCount[1]++;
				
				}
				
    }
}
void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
			
				/* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);
				if(stop0!=0x51){
					g_au32TMRINTCount[2]++;
					//g_au32TMRINTCount[2]++;
				
				}
//        /* Clear Timer2 time-out interrupt flag */
//        TIMER_ClearIntFlag(TIMER2);
//        g_au32TMRINTCount[2]++;
    }
}
void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);

        g_au32TMRINTCount[3]++;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IRC22M clock */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to Power-down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12 MHz XTAL, IRC10K */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC10K_STB_Msk));

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable peripheral clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk |
                  CLK_APBCLK_TMR0_EN_Msk | CLK_APBCLK_TMR1_EN_Msk | CLK_APBCLK_TMR2_EN_Msk | CLK_APBCLK_TMR3_EN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL |
                   CLK_CLKSEL1_TMR0_S_HXT | CLK_CLKSEL1_TMR1_S_HXT | CLK_CLKSEL1_TMR2_S_HIRC | CLK_CLKSEL1_TMR3_S_HXT;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void periodicTimer(void){
	    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER0->TCMPR = __HXT;
    TIMER0->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);

    /* Open Timer1 in periodic mode, enable interrupt and 2 interrupt ticks per second */
    TIMER1->TCMPR = __HXT;
    TIMER1->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);

    /* Open Timer2 in periodic mode, enable interrupt and 4 interrupt ticks per second */
//    TIMER2->TCMPR = __HXT;//((__HIRC / 1) / 4);
//    TIMER2->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
//    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);
		TIMER2->TCMPR = __HXT;
    TIMER2->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER2, 0);

    /* Open Timer3 in periodic mode, enable interrupt and 8 interrupt ticks per second */
    TIMER3->TCMPR = ((__HXT / 1) / 8);
    TIMER3->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER3, 0);

    /* Enable Timer0 ~ Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_EnableIRQ(TMR2_IRQn);
    NVIC_EnableIRQ(TMR3_IRQn);

    /* Clear Timer0 ~ Timer3 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = g_au32TMRINTCount[2] = g_au32TMRINTCount[3] = 0;
    u32InitCount = g_au32TMRINTCount[0];

}



uint32_t i =0;
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

		OpenKeyPad();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

		periodicTimer();	
		
    /* Start Timer0 ~ Timer3 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER1);
    TIMER_Start(TIMER2);
    TIMER_Start(TIMER3);

    /* Check Timer0 ~ Timer3 interrupt counts */
    printf("LAB4 timer start!!\n");
		while(1)
    {
				i=ScanKey();
				if(i==1){
					stop0=0x55;
				
				}
				if(i==2){
					stop1=0x01;
				
				}
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
						printf("\rTimer0:%3d    Timer1:%3d ",g_au32TMRINTCount[0], g_au32TMRINTCount[1]);
            //u32InitCount = g_au32TMRINTCount[0];
						
						

        }
    }

    printf("----------LAB4 terminate----------\n");

    while(1);
}


/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/





