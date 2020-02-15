/*
 * motor_ctrl.h
 *
 *  Created on: 1 de jul de 2019
 *      Author: Matheus_Pinto
 */

#ifndef DIFF_ROBOT_MOTOR_CTRL_H_
#define DIFF_ROBOT_MOTOR_CTRL_H_

#include "FreeRTOS.h"
#include "queue.h"

//#define pwmMAX_DUTY_CYCLE 65535
#define motorWHEEL_MIDDLE_AXLE 0.075 // 7.5 cm
#define motorWHELL_RADIUS 0.034 //34 cm
#define motorMAX_ROTATIONAL_SPEED 7.0 // 13.1 rad/s (aprox. 125 rpm)
//#define motorMAX_SPEED 0.4454
#define motorTEST_ENCODER_TASK

void motor_Init(void);

float motor_GetMaxSpeed(void);

void motor_CtrlTaskCreate(UBaseType_t uxPriority, TickType_t xPeriod);

void motor_OdomTaskCreate(UBaseType_t uxPriority, TickType_t xPeriod);

void motor_SetIncrementRightEncoder(uint32_t encoderCounter);
void motor_SetIncrementLeftEncoder(uint32_t encoderCounter);

#endif /* DIFF_ROBOT_MOTOR_CTRL_H_ */
