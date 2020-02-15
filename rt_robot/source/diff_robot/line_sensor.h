/*
 * line_sensor.h
 *
 *  Created on: 2 de jul de 2019
 *      Author: Matheus_Pinto
 */


#include <diff_robot/communic.h>
#include "FreeRTOS.h"
#include "task.h"
#include "fsl_gpio.h"

#define IR_PINS_MASK 0x60E00000 // 0110 0000 1110 0000 0000 0000 0000 0000

void lineDetect_Init(void);

void lineDetect_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod);
