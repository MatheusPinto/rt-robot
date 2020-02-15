/*
 * diff_robot.h
 *
 *  Created on: 1 de jul de 2019
 *      Author: Matheus_Pinto
 */

#ifndef DIFF_ROBOT_COMMUNIC_H_
#define DIFF_ROBOT_COMMUNIC_H_

#include "FreeRTOS.h"
#include "queue.h"

typedef struct{
	float x;
//	int16_t y;
//	int16_t z;
}linearVel_t;

typedef struct{
//	int16_t x;
//	int16_t y;
	float z;
}angularVel_t;


typedef struct{
	linearVel_t linear;
	angularVel_t angular;
	float maxSpeed;
}twist_t;

typedef struct{
	float x;
	float y;
	float theta;
}pose2D_t;

typedef struct
{
	uint16_t left;
	uint16_t front;
	uint16_t right;
}obst3Dist_t;

typedef enum
{
	lineNowhere 				= 0,
	lineFrontDir		= 1,
	lineVeryRightDir	= 2,
	lineRightDir		= 3,
	lineLowRightDir		= 4,
	lineLowLeftDir		= 5,
	lineLeftDir			= 6,
	lineVeryLeftDir		= 7,
	lineEverywhere		= 8
}lineSensorDir_t;

void communic_WaitInitSignal(void);

void communic_NotifyInitSignal(void);

TickType_t communic_GetInitalTime(void);

void communic_SendCmdVel(twist_t *cmdVel);
void communic_ReceiveCmdVel(twist_t *cmdVel);
void communic_SendObstRightDist(uint16_t *dists);
void communic_ReceiveObstRightDist(uint16_t *dists);
void communic_SendObstFrontDist(uint16_t *dists);
void communic_ReceiveObstFrontDist(uint16_t *dists);
void communic_SendObstLeftDist(uint16_t *dists);
void communic_ReceiveObstLeftDist(uint16_t *dists);
void communic_SendLineDir(lineSensorDir_t *dir);
void communic_ReceiveLineDir(lineSensorDir_t *dir);
void communic_SendPose2D(pose2D_t *pose);
void communic_ReceivePose2D(pose2D_t *pose);
void communic_SendDelthaDist(float *dist);
void communic_ReceiveDelthaDist(float *dist);
void communic_SendDelthaAngle(float *theta);
void communic_ReceiveDelthaAngle(float *theta);

void communic_Init(void);


#endif /* DIFF_ROBOT_COMMUNIC_H_ */
