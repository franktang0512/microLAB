#include <stdio.h> 
#include "NUC100Series.h"
#include <string.h> 
#include "GPIO.h"
#define PLLCON_SETTING CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK 50000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);

void SYS_Init(void) {
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

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

}

void UART0_Init() {
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void) {
  /* Unlock protected registers */
  SYS_UnlockReg();
  /* Init System, peripheral clock and multi-function I/O */
  SYS_Init();
  /* Lock protected registers */
  SYS_LockReg();
  /* Init UART0 for printf and testing */
  UART0_Init();
  printf("Lab2_UART with LED\n");
  PA7 = 0;
  /* UART sample function */
  UART_FunctionTest();

  while (1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void) {
  UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE() {
  char output[100];
  int i = 0;
  int ledon, ledoff;
  int redon, redoff, greenon, greenoff, blueon, blueoff;
  uint8_t u8InChar = 0xFF;
  uint32_t u32IntSts = UART0 -> ISR;

  if (u32IntSts & UART_ISR_RDA_INT_Msk) {
    printf("Input:");

    /* seize the data from keyboard and put it in ouput*/
    //while(UART0->ISR & UART_ISR_RDA_IF_Msk)
    do {
      while (UART_IS_RX_READY(UART0)) {

        /* Get the character from UART Buffer */
        u8InChar = UART0 -> RBR;
				//The seized data is going to store in output array step by step
        output[i] = u8InChar;
        i++;
      }
    } while (u8InChar != 0x0D);
  }

  if (u32IntSts & UART_ISR_RDA_INT_Msk) {
    printf("%s\n", output);
    GPIO_SetMode(PA, BIT12, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT13, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PA, BIT14, GPIO_PMD_OUTPUT);
    PA12 = 0;
    PA13 = 0;
    PA14 = 0;

    redon = strcmp(output, "red on");
    redoff = strcmp(output, "red off");
    greenon = strcmp(output, "green on");
    greenoff = strcmp(output, "green off");
    blueon = strcmp(output, "blue on");
    blueoff = strcmp(output, "blue off");

    if (redon == 1) {

      PA12 = 1;
      PA13 = 1;
      PA14 = 0;

    }
    if (redoff == 1) {

      PA12 = 1;
      PA13 = 1;
      PA14 = 1;

    }
    if (greenon == 1) {

      PA12 = 1;
      PA13 = 0;
      PA14 = 1;

    }
    if (greenoff == 1) {

      PA12 = 1;
      PA13 = 1;
      PA14 = 1;

    }
    if (blueon == 1) {

      PA12 = 0;
      PA13 = 1;
      PA14 = 1;

    }
    if (blueoff == 1) {

      PA12 = 1;
      PA13 = 1;
      PA14 = 1;

    }

    memset(output, '\0', 100);
  }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest() {
  UART0 -> IER |= UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_TOUT_IEN_Msk;
  NVIC_EnableIRQ(UART02_IRQn);
}

