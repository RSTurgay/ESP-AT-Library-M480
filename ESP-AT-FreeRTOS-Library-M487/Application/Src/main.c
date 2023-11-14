// Header: Main Source File
// File Name: main.c
// Author: Turgay Hopal
// Date: 11.08.2023

#include "main.h"

/* IRQ Handler Functions*/


/* Callback Functions */

static lwespr_t lwesp_callback_func(lwesp_evt_t *evt) {
	    
	switch (lwesp_evt_get_type(evt)) {
		case LWESP_EVT_AT_VERSION_NOT_SUPPORTED:
			printf("Current ESP8266 AT version is not supported by library!\r\n");
			break;

		case LWESP_EVT_INIT_FINISH:
			printf("Library initialized!\r\n");
			break;

		case LWESP_EVT_RESET:
			printf("Device reset sequence finished!\r\n");
			break;

		case LWESP_EVT_RESET_DETECTED:
			printf("Device reset detected!\r\n");
			break;

		case LWESP_EVT_WIFI_IP_ACQUIRED:
				break;
		default:
				break;
    }
}

/**
 * \brief           Lookup table for preferred SSIDs with password for auto connect feature
 */
typedef struct
{
    const char *ssid;
    const char *pass;
} ap_entry_t;


ap_entry_t ap_list[] =
{
    //{ "SSID name", "SSID password" },
		{"STRT", "STRT5481**tr!" },
};

/**
 * \brief           List of access points found by ESP device
 */
static lwesp_ap_t aps[100];

/**
 * \brief           Number of valid access points in \ref aps array
 */
static size_t apf;

lwespr_t connect_to_preferred_access_point(uint8_t unlimited) {
	lwespr_t eres;
	uint8_t tried;

	/*
	 * Scan for network access points
	 * In case we have access point,
	 * try to connect to known AP
	 */
	do
	{
			if (lwesp_sta_has_ip())
			{
					return lwespOK;
			}

			/* Scan for access points visible to ESP device */
			printf("Scanning access points...\r\n");

			if ((eres = lwesp_sta_list_ap(NULL, aps, LWESP_ARRAYSIZE(aps), &apf, NULL, NULL, 1)) == lwespOK)
			{
				tried = 0;

				/* Print all access points found by ESP */
				for (size_t i = 0; i < apf; i++)
				{
						printf("AP found: %s, CH: %d, RSSI: %d\r\n", aps[i].ssid, aps[i].ch, aps[i].rssi);
				}

			}
			else if (eres == lwespERRNODEVICE)
			{
					printf("Device is not present!\r\n");
					break;
			}
			else
			{
					printf("Error on WIFI scan procedure!\r\n");
			}

			if (!unlimited)
			{
					break;
			}
    } while (1);

    return lwespOK;
}



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
	
	printf("Wifi Task is creating ...\n");
	xTaskCreate(vWifiTask, "Wifi Task",  mainWIFI_TASK_STACK_SIZE, NULL, mainWIFI_TASK_PRIORITY, NULL);
	
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
		
	}
	
}

void vWifiTask(void *pvParameters) {
	
	/* Task Setup */
	
	printf("Wifi Task is started ...\n");
	
	printf("Initializing ESP-AT Lib\r\n");

	vTaskDelay(100);
	
	if (lwesp_init(lwesp_callback_func, 1) != lwespOK)
	{
			printf("Cannot initialize ESP-AT Lib!\r\n");
	}
	else
	{
			printf("ESP-AT Lib initialized!\r\n");
	}
	
	/* Task Loop */
	
	for (;;) {
		
		if (!lwesp_sta_is_joined())
		{
				/*
				 * Connect to access point.
				 *
				 * Try unlimited time until access point accepts up.
				 * Check for station_manager.c to define preferred access points ESP should connect to
				 */
				connect_to_preferred_access_point(1);
		}

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