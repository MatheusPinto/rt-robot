/*
 * display.c
 *
 *  Created on: 17 de jul de 2019
 *      Author: Matheus_Pinto
 */
#include <diff_robot/communic.h>
#include "FreeRTOS.h"
#include "task.h"
#include "emb_util/emb_util.h"
#include "string.h"
#include "display/lcd.h"
#include "pin_mux.h"
#include "math.h"
#include <diff_robot/fault.h>
#include <diff_robot/measure.h>

// task parameters
static TickType_t xDisplayTaskPeriod;

static void DisplayTask(void *pvParameters);


static void DisplayTask(void *pvParameters)
{
	pose2D_t pose;
	char poseXStr[12], poseYStr[12], poseThetaStr[12];
	TickType_t xLastWakeTime, xActualWakeTime;
	communic_WaitInitSignal();
	xActualWakeTime = xLastWakeTime = communic_GetInitalTime(); // Get the first task activation
	for(;;)
    {
		communic_ReceivePose2D(&pose);
		lcd_Clear();

		emb_ftoa(pose.x, poseXStr, 2);
		emb_ftoa(pose.y, poseYStr, 2);
		emb_ftoa((pose.theta*(180/emb_PI)), poseThetaStr, 4);

		/*It was put a space before "x" because the first charactere is not cosumed by LCD.
		 * It is a timing issue: putting large or lower times between lcd_Clear and
		 * the first charactere write make appears random moving chars.*/
	    lcd_WriteString(" x=");
	    lcd_WriteString(poseXStr);
	    lcd_WriteString(" y=");
	    lcd_WriteString(poseYStr);
	    lcd_WriteLn();
	    lcd_WriteString("theta=");
	    lcd_WriteString(poseThetaStr);

	    measure_CodeEndCatch();
	    /* Delays until next activation. */
        vTaskDelayUntil(&xActualWakeTime, xDisplayTaskPeriod);
        measure_CodeBeginCatch();
        fault_VerifyDeadline(xLastWakeTime, xActualWakeTime, xDisplayTaskPeriod);
        xLastWakeTime = xActualWakeTime;
    }

}

void display_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod)
{
	xDisplayTaskPeriod = xPeriod;

    if (xTaskCreate(DisplayTask, "disp", configMINIMAL_STACK_SIZE + 10, NULL, uxPriority, NULL) != pdPASS)
    {
           while(1);
    }
}


void display_Init(void)
{
	lcdConfig_t config;

    config.lines = 2;
    config.cols = 16;
    config.charsize = LCD_5x8DOTS;
    config.bus.data[0].base = BOARD_INITPINS_LCD_D4_GPIO;
    config.bus.data[0].pin =  BOARD_INITPINS_LCD_D4_PIN;
    config.bus.data[1].base = BOARD_INITPINS_LCD_D5_GPIO;
    config.bus.data[1].pin =  BOARD_INITPINS_LCD_D5_PIN;
    config.bus.data[2].base = BOARD_INITPINS_LCD_D6_GPIO;
    config.bus.data[2].pin =  BOARD_INITPINS_LCD_D6_PIN;
    config.bus.data[3].base = BOARD_INITPINS_LCD_D7_GPIO;
    config.bus.data[3].pin =  BOARD_INITPINS_LCD_D7_PIN;
    config.bus.rs.base = BOARD_INITPINS_LCD_RS_GPIO;
    config.bus.rs.pin =  BOARD_INITPINS_LCD_RS_PIN;
    config.bus.en.base = BOARD_INITPINS_LCD_EN_GPIO;
    config.bus.en.pin =  BOARD_INITPINS_LCD_EN_PIN;

    lcd_Init(&config);
}

