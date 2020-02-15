/*
 * line_sensor.c
 *
 *  Created on: 15 de jul de 2019
 *      Author: Matheus_Pinto
 */

#include "line_sensor.h"
#include <diff_robot/fault.h>
#include <diff_robot/measure.h>




// Ultrasonic task parameters
static TickType_t xLineTaskPeriod;

static void LineDetectTask(void *pvParameters);


void lineDetect_Init(void)
{

}

void lineDetect_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod)
{
	xLineTaskPeriod = xPeriod;

    if (xTaskCreate(LineDetectTask, "line", configMINIMAL_STACK_SIZE + 10, NULL, uxPriority, NULL) != pdPASS)
    {
           while(1);
    }
}


static void LineDetectTask(void *pvParameters)
{
	TickType_t xActualWakeTime, xLastWakeTime;
	lineSensorDir_t dir;

	communic_WaitInitSignal();
	xActualWakeTime = xLastWakeTime = communic_GetInitalTime(); // Get the first task activation
	for(;;)
    {
		// 0110 0000 1110 0000 0000 0000 0000 0000
		// 0x60E00000
		switch(GPIOE->PDIR & IR_PINS_MASK)
		{
		// 0010 0000 1110 0000 0000 0000 0000 0000
		case 0x20E00000:
		case 0x00E00000:
		case 0x00600000:
			dir = lineVeryRightDir;
			break;
		// 0100 0000 1110 0000 0000 0000 0000 0000
		case 0x40E00000:
			dir = lineRightDir;
			break;
		// 0100 0000 0110 0000 0000 0000 0000 0000
		case 0x40600000:
			dir = lineLowRightDir;
			break;
		// 0110 0000 0110 0000 0000 0000 0000 0000
		case 0x60600000:
			dir = lineFrontDir;
			break;
		// 0110 0000 0010 0000 0000 0000 0000 0000
		case 0x60200000:
			dir = lineLowLeftDir;
			break;
		// 0110 0000 1010 0000 0000 0000 0000 0000
		case 0x60A00000:
			dir = lineLeftDir;
			break;
		// 0110 0000 1100 0000 0000 0000 0000 0000
		case 0x60C00000:
		case 0x60800000:
		case 0x60000000:
			dir = lineVeryLeftDir;
			break;
		case 0x60E00000:
			dir = lineNowhere;
			break;
		default:
			dir = lineEverywhere;
		}

		communic_SendLineDir(&dir);

		measure_CodeEndCatch();
		/* Delays until next activation. */
        vTaskDelayUntil(&xActualWakeTime, xLineTaskPeriod);
        measure_CodeBeginCatch();
        fault_VerifyDeadline(xLastWakeTime, xActualWakeTime, xLineTaskPeriod);
        xLastWakeTime = xActualWakeTime;

    }

}
