/**************************************************************************//**
 * @file     main.c
 * @version  V3.0
 * $Revision: 3 $
 * $Date: 15/04/20 2:54p $
 * @brief
 *           Implement SPI Master loop back transfer.
 *           This sample code needs to connect SPI0_MISO0 pin and SPI0_MOSI0 pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>

#include "NUC100Series.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

#define TEST_COUNT             64
#define SPI_READ 0x80
#define SPI_WRITE 0x00


/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);
void SPI_Init(void);

void ADXL_Init();
void SPI_Write(uint8_t addr, uint8_t value);
int8_t SPI_Read(uint8_t addr);

float ADXL_Read_DataX();
float ADXL_Read_DataY();
float ADXL_Read_DataZ();

void SPI_Write(uint8_t addr, uint8_t value){

		SPI2->TX[0] =addr | SPI_WRITE;
		SPI2->SSR |= SPI_SSR_SSR_Msk;  	//ss start
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;               //wait until transfer done
    while((SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == 1);
    SPI2->TX[0] = value;                                 //tx
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;
    while((SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == 1);

    SPI2->SSR &= ~SPI_SSR_SSR_Msk;                      //set ss high
}

int8_t SPI_Read(uint8_t addr){

    int8_t temp;

		SPI2->TX[0] =addr | SPI_READ;
		SPI2->SSR |= SPI_SSR_SSR_Msk;  	//ss start
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk; 	//wait until transfer done
    while((SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == 1);
		//SPI2->TX[0] = 0xFF;
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;               //receive
    while((SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == 1);
    temp = SPI2->RX[0];
    SPI2->SSR &= ~SPI_SSR_SSR_Msk;                      //set ss high

    return temp;
}

void ADXL_Init(){
    SPI_Write(0x31,0x0B);
		SPI_Write(0x2D,0x08);
    SPI_Write(0x38,0x80);
}

float ADXL_Read_DataX(void){
		float xaxisData;
		int8_t xhigh;
		int8_t xlow;
		xlow=SPI_Read(0x32);
		xhigh=SPI_Read(0x33);

		xaxisData=((xhigh<<8)+ xlow);
		xaxisData=(xaxisData-7)/(256-7);
//		xaxisData=(xaxisData+150)/440-0.36;
		//xaxisData=xaxisData+149;
		//xaxisData=xaxisData/437;
		//printf("x= %.2f\n",xaxisData);
    return xaxisData;
}

float ADXL_Read_DataY(void){
		float yaxisData;
		int8_t yhigh;
		int8_t ylow;
		ylow=SPI_Read(0x34);
		yhigh=SPI_Read(0x35);

		yaxisData=((yhigh<<8)+ ylow);
		yaxisData=(yaxisData-3)/(256-3);
//		yaxisData=(yaxisData+170)/440+0.3;
//		//yaxisData=yaxisData+168;
		//yaxisData=yaxisData/432;
		//printf("y= %.2f\n",yaxisData);
    return yaxisData;
}

float ADXL_Read_DataZ(void){
		float zaxisData;
		int8_t zhigh;
		int8_t zlow;
		zlow=SPI_Read(0x36);
		zhigh=SPI_Read(0x37);

		zaxisData=((zhigh<<8)+ zlow);
		zaxisData=(1000*zaxisData-25*256)/(256000);
//		zaxisData=(zaxisData+1500)/(256+1500);
//		zaxisData=(zaxisData-755)/265+2.86;
		//printf("%.2f\n",zaxisData);
    return zaxisData;
}
int main(void)
{
		float xData;
		float yData;
		float zData;
		int d;

        SYS_UnlockReg();

        SYS_Init();

        SYS_LockReg();

        UART0_Init();

        SPI_Init();

		ADXL_Init();

		d=SPI_Read(0x00);
		printf("\n%x\n",d);//e5


        while(1){
        //Read 3 axis acceleration(calibration) then print out
            xData=ADXL_Read_DataX();
            yData=ADXL_Read_DataY();
            zData=ADXL_Read_DataZ();

			printf("x= %.2f ,y= %.2f ,z= %.2f\n",xData ,yData,zData);

            for(d=1;d<=2;d++)
                CLK_SysTickDelay(10000000);
		}
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184 MHz clock. Some peripherals select internal RC oscillator as default clock source. */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Enable external 12 MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Configure PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    /* Select PLL as the system clock source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;

    /* Select HXT as the clock source of UART; select HCLK as the clock source of SPI0. */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~(CLK_CLKSEL1_UART_S_Msk | CLK_CLKSEL1_SPI2_S_Msk))) | (CLK_CLKSEL1_UART_S_HXT | CLK_CLKSEL1_SPI2_S_HCLK);

    /* Enable UART and SPI2 clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_SPI2_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Setup SPI0 multi-function pins */
    SYS->GPD_MFP = SYS_GPD_MFP_PD0_SPI2_SS0 | SYS_GPD_MFP_PD1_SPI2_CLK | SYS_GPD_MFP_PD2_SPI2_MISO0 | SYS_GPD_MFP_PD3_SPI2_MOSI0;
    SYS->ALT_MFP = SYS_ALT_MFP_PD0_SPI2_SS0 | SYS_ALT_MFP_PD1_SPI2_CLK | SYS_ALT_MFP_PD2_SPI2_MISO0 | SYS_ALT_MFP_PD3_SPI2_MOSI0;
}

void UART0_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LCR = UART_LCR_WLS_Msk;
    /* Using mode 2 calculation: UART bit rate = UART peripheral clock rate / (BRD setting + 2) */
    /* UART peripheral clock rate 12MHz; UART bit rate 115200 bps. */
    /* 12000000 / 115200 bps ~= 104 */
    /* 104 - 2 = 0x66. */
    UART0->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0x66);
}



void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/

    SPI2->DIVIDER = (SPI2->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | 24;


    /* set SS at low level trigger */
    SPI2->SSR &= ~SPI_SSR_SS_LVL_Msk;
    /* disable the auto SS */
    SPI2->SSR &= ~SPI_SSR_AUTOSS_Msk;

    /* set SPI at master mode */
    SPI2->CNTRL &=~SPI_CNTRL_SLAVE_Msk;
    /* set clock idle state at  */
    SPI2->CNTRL |=SPI_CNTRL_CLKP_Msk;
    /* set transmitt data at falling edge, recieve data at rising edge */
		SPI2->CNTRL |= SPI_CNTRL_TX_NEG_Msk;
    SPI2->CNTRL	&=~SPI_CNTRL_RX_NEG_Msk;
    /* set bit length of word transfer */
		//SPI2->CNTRL |= ( (0x08 << SPI_CNTRL_TX_BIT_LEN_Pos));
		SPI2->CNTRL |= ((SPI2->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | (0x08 << SPI_CNTRL_TX_BIT_LEN_Pos));
		//SPI2->CNTRL = 0x0<<8;
		//SPI2->CNTRL = 0x0<<9;

}



/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

