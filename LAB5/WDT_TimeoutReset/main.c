/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 1 $
 * $Date: 14/12/08 11:49a $
 * @brief
 *           Show how to generate time-out reset system event while WDT time-out reset delay period expired.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000
//Define Clock source
#define MCU_CLOCK_SOURCE      
#define MCU_CLOCK_SOURCE_HIRC   // HXT, LXT, HIRC, LIRC 
#define MCU_CLOCK_FREQUENCY     50000000  //Hz

//Define MCU Interfaces
#define MCU_INTERFACE_UART0
#define UART_CLOCK_SOURCE_HIRC // HXT, PLL, HIRC
#define UART_CLOCK_DIVIDER     3
#define PIN_UART0_RX_PB0
#define PIN_UART0_TX_PB1
/**************************************/
//
// For NUC140
//
#ifndef __SYS_init_H__
#define __SYS_init_H__

   #ifdef MCU_CLOCK_SOURCE_HXT
     #define MCU_CLOCK_SOURCE_MASK_HXT  CLK_PWRCTL_HXT_EN_Msk
     #define CLK_CLKSTATUS_STB_MASK_HXT CLK_CLKSTATUS_HXT_STB_Msk
   #else
     #define MCU_CLOCK_SOURCE_MASK_HXT 0
     #define CLK_CLKSTATUS_STB_MASK_HXT 0
   #endif
   #ifdef MCU_CLOCK_SOURCE_LXT
     #define MCU_CLOCK_SOURCE_MASK_LXT  CLK_PWRCTL_LXT_EN_Msk
     #define CLK_CLKSTATUS_STB_MASK_LXT CLK_CLKSTATUS_LXT_STB_Msk
   #else
     #define MCU_CLOCK_SOURCE_MASK_LXT 0
     #define CLK_CLKSTATUS_STB_MASK_LXT 0
   #endif
   #ifdef MCU_CLOCK_SOURCE_HIRC
     #define MCU_CLOCK_SOURCE_MASK_HIRC  CLK_PWRCTL_HIRC_EN_Msk
     #define CLK_CLKSTATUS_STB_MASK_HIRC CLK_CLKSTATUS_HIRC_STB_Msk
   #else
     #define MCU_CLOCK_SOURCE_MASK_HIRC 0
     #define CLK_CLKSTATUS_STB_MASK_HIRC 0
   #endif
   #ifdef MCU_CLOCK_SOURCE_LIRC
     #define MCU_CLOCK_SOURCE_MASK_LIRC  CLK_PWRCTL_LIRC_EN_Msk
     #define CLK_CLKSTATUS_STB_MASK_LIRC CLK_CLKSTATUS_LIRC_STB_Msk
   #else
     #define MCU_CLOCK_SOURCE_MASK_LIRC 0
     #define CLK_CLKSTATUS_STB_MASK_LIRC 0
   #endif 
   #ifdef MCU_CLOCK_SOURCE
     #define MCU_CLOCK_SOURCE_MASK (MCU_CLOCK_SOURCE_MASK_HXT | MCU_CLOCK_SOURCE_MASK_LXT | MCU_CLOCK_SOURCE_MASK_HIRC | MCU_CLOCK_SOURCE_MASK_LIRC)
     #define MCU_CLOCK_STABLE_MASK (CLK_CLKSTATUS_STB_MASK_HXT | CLK_CLKSTATUS_STB_MASK_LXT | CLK_CLKSTATUS_STB_MASK_HIRC | CLK_CLKSTATUS_STB_MASK_LIRC)
   #endif

extern void SYS_Init(void);

#endif

#define GPIO_MODE_OUTPUT        0x1UL /*!< Output Mode */
/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
extern int IsDebugFifoEmpty(void);
volatile uint8_t g_u8IsWDTTimeoutINT = 0;

volatile uint32_t state=0;

void GPAB_IRQHandler(void)
{
    /* To check if PB.3 interrupt occurred */

    if(GPIO_GET_INT_FLAG(PA, BIT2))
    {
				state=state+1;
        GPIO_CLR_INT_FLAG(PA, BIT2);
					printf("Change!!!\n");

    }
    else
    {
        /* Un-expected interrupt. Just clear all PA, PB interrupts */
        PA->ISRC = PA->ISRC;
        PB->ISRC = PB->ISRC;
        printf("Un-expected interrupts.\n");
    }
}

/**
 * @brief       IRQ Handler for WDT and WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT and WWDT, declared in startup_NUC100Series.s.
 */
void WDT_IRQHandler(void)
{
    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();

        g_u8IsWDTTimeoutINT = 1;

        printf("Watch Dog Time-out Interrupt is taking place.\n");
				if(state%2==0)
					printf("reset the emergency alarm when the state is not safe!!\n");
				else if(state%2==1)
				{
						/* Use PA.0 to check time-out period time */
						PA->PMD = 0xFFFFFFFD;
						PA0 = 1;
						
						WDT_CLEAR_RESET_FLAG();
						printf("Alarm!!!~~~Reset!!!\n");
						printf("Start Lab5\n");
							//while(1);
				}
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

    /* Enable external 12 MHz XTAL, IRC 10 kHz */
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
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_WDT_EN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL | CLK_CLKSEL1_WDT_S_LIRC;

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
    /* Reset IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}


void Buzz(int number)
{
	int i;
	for (i=0; i<number; i++) {
          PB11=0; // PB11 = 0 to turn on Buzzer
	  CLK_SysTickDelay(100000);	 // Delay 
	  PB11=1; // PB11 = 1 to turn off Buzzer	
	  //CLK_SysTickDelay(100000);	 // Delay 
	}
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
		
	
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

		PA3=0;
    /* Configure PB.3 as Input mode and enable interrupt by rising edge trigger */
    PA->PMD = (PA->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD2_Pos);
    PA->IMD |= (GPIO_IMD_EDGE << 2);
    PA->IEN |= (BIT2 << GPIO_IEN_IR_EN_Pos);
    NVIC_EnableIRQ(GPAB_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO->DBNCECON = (GPIO_DBNCECON_ICLK_ON_Msk | GPIO_DBCLKSRC_LIRC | GPIO_DBCLKSEL_1024);
    PA->DBEN |= (BIT3 | BIT2);
		
    /* Use PA.0 to check time-out period time */
    PA->PMD = 0xFFFFFFFD;
    PA0 = 1;
    PA0 = 0;

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Enable WDT time-out reset function and select time-out interval to 2^14 * WDT clock then start WDT counting */
    g_u8IsWDTTimeoutINT = 0;
    WDT->WTCR = WDT_TIMEOUT_2POW14 | WDT_WTCR_WTIE_Msk | WDT_WTCR_WTRE_Msk | WDT_WTCR_WTE_Msk;
	
    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);
		//GPIO_SetMode(PB, BIT11, GPIO_MODE_OUTPUT);

    /* Waiting for interrupts */
    while(1)
		{
				if(state%2==0){
					printf("Safe!\n");				
				}else if(state%2==1){
					printf("Alarm!\n");
					Buzz(1); // Buzz 1 times
					
				}
				CLK_SysTickDelay(1000000);
		}
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
