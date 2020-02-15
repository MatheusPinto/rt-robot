/*
 * pose_handler.c
 *
 *  Created on: 17 de jul de 2019
 *      Author: Matheus_Pinto
 */

#include "diff_robot/communic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pose_handler.h"
#include "emb_util/emb_util.h"
#include <math.h>
#include <diff_robot/fault.h>
#include <diff_robot/measure.h>

// task parameters
static TickType_t xPoseTaskPeriod;

static void PoseTask(void *pvParameters);

static void PoseTask(void *pvParameters)
{
	pose2D_t pose;
	float actualDist = 0, actualAngle = 0;
	TickType_t xActualWakeTime, xLastWakeTime;

	pose.x = pose.y = pose.theta = 0;

	communic_WaitInitSignal();
	xActualWakeTime = xLastWakeTime = communic_GetInitalTime(); // Get the first task activation
	for(;;)
    {
		communic_ReceiveDelthaDist(&actualDist);
		communic_ReceiveDelthaAngle(&actualAngle);

		pose.theta += actualAngle;

		while(pose.theta < 0)
		{
			pose.theta += emb_2_PI;
		}

		while(pose.theta > emb_2_PI)
		{
			pose.theta -= emb_2_PI;
		}

		pose.x += (actualDist * cos(pose.theta));
		pose.y += (actualDist * sin(pose.theta));

		communic_SendPose2D(&pose);
		measure_CodeEndCatch();
	    /* Delays until next activation. */
        vTaskDelayUntil(&xActualWakeTime, xPoseTaskPeriod);
        measure_CodeBeginCatch();
        fault_VerifyDeadline(xLastWakeTime, xActualWakeTime, xPoseTaskPeriod);
         xLastWakeTime = xActualWakeTime;

    }

}

void poseHand_Init(void)
{

}

void poseHand_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod)
{
	xPoseTaskPeriod = xPeriod;

    if (xTaskCreate(PoseTask, "pose", configMINIMAL_STACK_SIZE + 10, NULL, uxPriority, NULL) != pdPASS)
    {
           while(1);
    }
}

