/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 15/04/13 8:37p $
 * @brief
 *           Show a I2C Master how to access Slave.
 *           This sample code needs to work with I2C_Slave.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"
#include <stdint.h>

#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000

#define ADXL_W_ADR 0xA6
#define ADXL_R_ADR 0xA7

		float xData;
		float yData;
		float zData;
		int d;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
typedef void (*I2C_FUNC)(uint32_t u32Status);

//static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

void ADXL_Write(uint8_t addr,uint8_t value){
    //I2C Write operation
		I2C_Trigger(I2C0,1,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		I2C0->I2CDAT=ADXL_W_ADR;
		I2C_Trigger(I2C0,0,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		I2C0->I2CDAT=addr;
		I2C_Trigger(I2C0,0,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);
		I2C0->I2CDAT=value;
		I2C_Trigger(I2C0,0,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		I2C_Trigger(I2C0,0,1,1,0);
}

int8_t ADXL_Read(uint8_t addr){
    //I2C Read operation
		int8_t byteData;

		I2C_Trigger(I2C0,1,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		I2C0->I2CDAT=ADXL_W_ADR;
		I2C_Trigger(I2C0,0,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		I2C0->I2CDAT=addr;
		I2C_Trigger(I2C0,0,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		I2C_Trigger(I2C0,1,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		I2C0->I2CDAT=ADXL_R_ADR;
		I2C_Trigger(I2C0,0,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		I2C0->I2CDAT=0xFF;
		I2C_Trigger(I2C0,0,0,1,0);
		while(((I2C0->I2CON)& I2C_I2CON_SI_Msk)==0);

		byteData=I2C_GetData(I2C0);

		I2C_Trigger(I2C0,0,1,1,0);

    return byteData;
}

void ADXL_Init(){
    ADXL_Write(0x31,0x0B);
		ADXL_Write(0x2D,0x08);
    ADXL_Write(0x38,0x80);
}

float ADXL_Read_DataX(void){
		float xaxisData;
		int8_t xhigh;
		int8_t xlow;
		xlow=ADXL_Read(0x32);
		xhigh=ADXL_Read(0x33);

		xaxisData=((xhigh<<8)+ xlow);
		xaxisData=xaxisData/256;
		
    return xaxisData;
}

float ADXL_Read_DataY(void){
		float yaxisData;
		int8_t yhigh;
		int8_t ylow;
		ylow=ADXL_Read(0x34);
		yhigh=ADXL_Read(0x35);

		yaxisData=((yhigh<<8)+ ylow);
		yaxisData=yaxisData/256;
		
	
    return yaxisData;
}

float ADXL_Read_DataZ(void){
		float zaxisData;
		int8_t zhigh;
		int8_t zlow;
		zlow=ADXL_Read(0x36);
		zhigh=ADXL_Read(0x37);

		zaxisData=((zhigh<<8)+ zlow);
		zaxisData=zaxisData/256;
		
    return zaxisData;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART & I2C0 module clock */
    CLK->APBCLK |= (CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_I2C0_EN_Msk);

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set GPA multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFP = SYS_GPA_MFP_PA8_I2C0_SDA | SYS_GPA_MFP_PA9_I2C0_SCL;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void I2C0_Init(void)
{
    uint32_t u32BusClock;

    /* Reset I2C0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_I2C0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_I2C0_RST_Msk;

    /* Enable I2C0 Controller */
    I2C0->I2CON |= I2C_I2CON_ENS1_Msk;

    /* I2C0 bus clock 100K divider setting, I2CLK = PCLK/(100K*4)-1 */
    u32BusClock = 100000;
    I2C0->I2CLK = (uint32_t)(((SystemCoreClock * 10) / (u32BusClock * 4) + 5) / 10 - 1); /* Compute proper divider for I2C clock */

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", (SystemCoreClock / (((I2C0->I2CLK) + 1) << 2)));
/*
    // Set I2C0 4 Slave Addresses
    // Slave Address : 0x15
    I2C0->I2CADDR0 = (I2C0->I2CADDR0 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x15 << I2C_I2CADDR_I2CADDR_Pos);
    // Slave Address : 0x35
    I2C0->I2CADDR1 = (I2C0->I2CADDR1 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x35 << I2C_I2CADDR_I2CADDR_Pos);
    // Slave Address : 0x55
    I2C0->I2CADDR2 = (I2C0->I2CADDR2 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x55 << I2C_I2CADDR_I2CADDR_Pos);
    // Slave Address : 0x75
    I2C0->I2CADDR3 = (I2C0->I2CADDR3 & ~I2C_I2CADDR_I2CADDR_Msk) | (0x75 << I2C_I2CADDR_I2CADDR_Pos);

    // Enable I2C0 interrupt and set corresponding NVIC bit
    I2C0->I2CON |= I2C_I2CON_EI_Msk;
    NVIC_EnableIRQ(I2C0_IRQn);*/
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C0->I2CON &= ~I2C_I2CON_EI_Msk;
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C0->I2CON &= ~I2C_I2CON_ENS1_Msk;
    CLK->APBCLK &= ~CLK_APBCLK_I2C0_EN_Msk;

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

		I2C_Open(I2C0,100000);
    /* Init I2C0 */
    I2C0_Init();

    ADXL_Init();

		printf("LAB6 START\n");

		d=ADXL_Read(0x00);
//		printf("\n%x",d);
        while(1){
        //Read 3 axis acceleration(calibration) then print out
            xData=ADXL_Read_DataX();
            yData=ADXL_Read_DataY();
            zData=ADXL_Read_DataZ();
            printf("x= %.2f, y= %.2f, z= %.2f\n",xData,yData,zData);
            for(d=1;d<=2;d++)
                CLK_SysTickDelay(10000000);
		}
		I2C0_Close();
    while(1);
}
