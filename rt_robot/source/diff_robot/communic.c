/*
 * diff_robot.c
 *
 *  Created on: 1 de jul de 2019
 *      Author: Matheus_Pinto
 */

#include <diff_robot/communic.h>
#include "task.h"
#include "semphr.h"

#define TASKS_NUMBER_CREATED 9

//QueueHandle_t cmdVelQueue, obstacleDistQueue, lineDirQueue;
SemaphoreHandle_t xCmdVelSem, xObstLeftSem, xObstFrontSem, xObstRightSem, xLineDirSem, xPose2DSem, xDelthaDistSem, xDelthaThetaSem;

static twist_t cmdVelMsg;
static uint16_t obstLeftMsg, obstFrontMsg, obstRightMsg;
static lineSensorDir_t lineDirMsg;
static pose2D_t pose2DMsg;
static float delthaDistMsg;
static float delthaThetaMsg;

static xSemaphoreHandle xSignalInitSem;

TickType_t initialTime;

void communic_Init(void)
{
	xCmdVelSem 	 	= xSemaphoreCreateMutex();
	xObstFrontSem 	= xSemaphoreCreateMutex();
	xObstLeftSem 	= xSemaphoreCreateMutex();
	xObstRightSem 	= xSemaphoreCreateMutex();
	xLineDirSem  	= xSemaphoreCreateMutex();
	xPose2DSem	 	= xSemaphoreCreateMutex();
	xDelthaDistSem 	= xSemaphoreCreateMutex();
	xDelthaThetaSem = xSemaphoreCreateMutex();

	xSignalInitSem = xSemaphoreCreateCounting(TASKS_NUMBER_CREATED, 0);

	//cmdVelQueue = xQueueCreate(10, sizeof(twist_t));
	//obstacleDistQueue = xQueueCreate(10, sizeof(uint16_t));
	//lineDirQueue = xQueueCreate(1, sizeof(lineSensorDir_t));
}

void communic_WaitInitSignal(void)
{
	xSemaphoreTake(xSignalInitSem, portMAX_DELAY);
}

void communic_NotifyInitSignal(void)
{
	for(int i = 0; i < TASKS_NUMBER_CREATED; ++i)
	{
		xSemaphoreGive(xSignalInitSem);
	}
	initialTime =  xTaskGetTickCount(); // Get the first task activation
}

TickType_t communic_GetInitalTime(void)
{
	return initialTime;
}

void communic_SendCmdVel(twist_t *cmdVel)
{
	xSemaphoreTake(xCmdVelSem, portMAX_DELAY);
	cmdVelMsg.linear.x 	= cmdVel->linear.x;
	cmdVelMsg.angular.z = cmdVel->angular.z;
	xSemaphoreGive(xCmdVelSem);
	//xQueueSend(cmdVelQueue, (void*)cmdVel, portMAX_DELAY);
}


void communic_ReceiveCmdVel(twist_t *cmdVel)
{
	xSemaphoreTake(xCmdVelSem, portMAX_DELAY);
	cmdVel->linear.x  = cmdVelMsg.linear.x;
	cmdVel->angular.z = cmdVelMsg.angular.z;
	xSemaphoreGive(xCmdVelSem);

	//xQueueReceive(cmdVelQueue, (void*)cmdVel, portMAX_DELAY);
}

void communic_SendObstRightDist(uint16_t *dist)
{
	xSemaphoreTake(xObstRightSem, portMAX_DELAY);
	obstRightMsg = *dist;
	xSemaphoreGive(xObstRightSem);
}

void communic_ReceiveObstRightDist(uint16_t *dist)
{
	xSemaphoreTake(xObstRightSem, portMAX_DELAY);
	*dist = obstRightMsg;
	xSemaphoreGive(xObstRightSem);
}

void communic_SendObstLeftDist(uint16_t *dist)
{
	xSemaphoreTake(xObstLeftSem, portMAX_DELAY);
	obstLeftMsg = *dist;
	xSemaphoreGive(xObstLeftSem);
}

void communic_ReceiveObstLeftDist(uint16_t *dist)
{
	xSemaphoreTake(xObstLeftSem, portMAX_DELAY);
	*dist = obstLeftMsg;
	xSemaphoreGive(xObstLeftSem);
}

void communic_SendObstFrontDist(uint16_t *dist)
{
	xSemaphoreTake(xObstFrontSem, portMAX_DELAY);
	obstFrontMsg = *dist;
	xSemaphoreGive(xObstFrontSem);
}

void communic_ReceiveObstFrontDist(uint16_t *dist)
{
	xSemaphoreTake(xObstFrontSem, portMAX_DELAY);
	*dist = obstFrontMsg;
	xSemaphoreGive(xObstFrontSem);
}


void communic_SendLineDir(lineSensorDir_t *dir)
{
	xSemaphoreTake(xLineDirSem, portMAX_DELAY);
	lineDirMsg = *dir;
	xSemaphoreGive(xLineDirSem);
	//xQueueSend(lineDirQueue, (void*)dir, portMAX_DELAY);
}

void communic_ReceiveLineDir(lineSensorDir_t *dir)
{
	xSemaphoreTake(xLineDirSem, portMAX_DELAY);
	*dir = lineDirMsg;
	xSemaphoreGive(xLineDirSem);

	//xQueueReceive(lineDirQueue, (void*)dir, portMAX_DELAY);
}

void communic_SendPose2D(pose2D_t *pose)
{
	xSemaphoreTake(xPose2DSem, portMAX_DELAY);
	pose2DMsg.x = pose->x;
	pose2DMsg.y = pose->y;
	pose2DMsg.theta = pose->theta;
	xSemaphoreGive(xPose2DSem);
}

void communic_ReceivePose2D(pose2D_t *pose)
{
	xSemaphoreTake(xPose2DSem, portMAX_DELAY);
	pose->x = pose2DMsg.x;
	pose->y = pose2DMsg.y;
	pose->theta = pose2DMsg.theta;
	xSemaphoreGive(xPose2DSem);
}

void communic_SendDelthaDist(float *dist)
{
	xSemaphoreTake(xDelthaDistSem, portMAX_DELAY);
	delthaDistMsg = *dist;
	xSemaphoreGive(xDelthaDistSem);
}

void communic_ReceiveDelthaDist(float *dist)
{
	xSemaphoreTake(xDelthaDistSem, portMAX_DELAY);
	*dist = delthaDistMsg;
	xSemaphoreGive(xDelthaDistSem);
}

void communic_SendDelthaAngle(float *theta)
{
	xSemaphoreTake(xDelthaThetaSem, portMAX_DELAY);
	delthaThetaMsg = *theta;
	xSemaphoreGive(xDelthaThetaSem);
}

void communic_ReceiveDelthaAngle(float *theta)
{
	xSemaphoreTake(xDelthaThetaSem, portMAX_DELAY);
	*theta = delthaThetaMsg;
	xSemaphoreGive(xDelthaThetaSem);
}
