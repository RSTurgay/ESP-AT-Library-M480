// Header: Main Header File
// File Name: main.h
// Author: Turgay Hopal
// Date: 11.08.2023


#ifndef INC_MAIN_H_
#define INC_MAIN_H_

/* Hardware Includes */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "M480.h"

/* Library Includes */

/* Kernel Includes */

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* LwESP Library Includes*/
#include "lwesp/lwesp.h"

/* Application Definitions */


/* FreeRTOS Definitions */
#define mainCHECK_TASK_PRIORITY				(tskIDLE_PRIORITY + 3UL)
#define mainWIFI_TASK_PRIORITY				(tskIDLE_PRIORITY + 3UL)

#define mainCHECK_TASK_STACK_SIZE			(configMINIMAL_STACK_SIZE)
#define mainWIFI_TASK_STACK_SIZE			(512)

/* Hardware Definitions */

/* FreeRTOS Functions*/

void vCheckTask(void *pvParameters);
void vWifiTask(void *pvParameters);

/* Application Functions*/

/* Hardware Functions */

void setupSystemClock(void);

void setupUartMultifunction(void);
void setupUartClock(void);

void setupUart(void);
void setupGpio(void);

void writePin(volatile unsigned int * pinMask, uint8_t value);

#endif /* INC_MAIN_H_ */