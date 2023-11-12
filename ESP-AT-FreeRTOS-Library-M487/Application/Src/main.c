// Header: Main Source File
// File Name: main.c
// Author: Turgay Hopal
// Date: 11.08.2023


#include "main.h"

/* IRQ Handler Functions*/

/* Main Application*/

int main(void) {
	
	/* Hardware Setup*/
	
	setupSystemClock();
	setupGpio();
	setupUart();
	
	printf("\n\nCPU @ %dHz\n", SystemCoreClock);
	printf("M487JIDAE Running\n");
	

	/* FreeRTOS Setup */
	
	printf("Check Task is creating ...\n");
	xTaskCreate(vCheckTask, "Check Task",  mainCHECK_TASK_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL);
	
	printf("FreeRTOS is starting ...\n");
	
	/* Start the scheduler. */
  vTaskStartScheduler();

	for(;;);
	
	/* Application Setup */
	
	while (1){}
	
	return 0;
}


/* FreeRTOS Functions Implemantation*/

void vCheckTask(void *pvParameters) {
	
	/* Task Setup */
	
	printf("Check Task is started ...\n");
	
	/* Task Loop */
	
	for (;;) {
		
		printf("App running time : %d \n", xTaskGetTickCount());
		vTaskDelay(1000);
		
	}
	
}


/* Application Function Implemantation*/


/* Hardware Function Implemantation*/

void setupGpio(void) {
		
}


void setupSystemClock(void)
{
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init System Clock                                                                                       */
	/*---------------------------------------------------------------------------------------------------------*/

	/* Unlock protected registers */
	SYS_UnlockReg();
	
	/* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
   PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

  /* Enable HIRC48 Clock (Internal 48 Mhz)*/
  CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

  /* Wait for HIRC48 clock ready */
  CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);
	
	/* Switch HCLK clock source to HIRC */
  CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

  /* Set core clock as PLL_CLOCK from PLL */
  CLK_SetCoreClock(FREQ_192MHZ);
	
	/* Set both PCLK0 and PCLK1 as HCLK/2 */
  CLK->PCLKDIV = CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2;
	CLK->PCLKDIV = CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2;


	setupUartClock();
	
	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
	SystemCoreClockUpdate();
	
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init I/O Multi-function                                                                                 */
	/*---------------------------------------------------------------------------------------------------------*/

	setupUartMultifunction();
	
	/* Lock protected registers */
	SYS_LockReg();
	
}

void setupUartMultifunction(void) {
	
	/* Set GPC multi-function pins for UART0 RXD and TXD */
	SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
	SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
	
}

void setupUartClock(void) {
	/* Enable IP clock */
	CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk; // UART0 Clock Enable

	/* Enable UART module clock */
	CLK_EnableModuleClock(UART0_MODULE);
	/* Select UART module clock source */
	CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
	
}

void setupUart(void) {
	
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init UART                                                                                               */
	/*---------------------------------------------------------------------------------------------------------*/
	/* Reset UART module */
	SYS_ResetModule(UART0_RST);

	/* Configure UART0 and set UART0 baud rate */
	UART_Open(UART0, 115200);
	
}

void writePin(volatile unsigned int * pinMask, uint8_t value) {
	*pinMask = value;
}