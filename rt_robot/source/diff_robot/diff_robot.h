/***************************************************************************************
 * Module      : diff_robot.h
 * Revision    : 1.0
 * Date        : 17/07/2019
 * Description : It contains the declarations of the differential robot controller
				 tasks and functions.
 * Comments    : Copyright (c) 2018, Matheus Leiztke Pinto
 * 				 Instituto Federal de Santa Catarina - IFSC
 * 				 Departamento Acadêmico de Eletrônica.
 * 		 		 All rights reserved (https://github.com/MatheusPinto/rt-robot).
 *
 * 		 		 Redistribution and use in source and binary forms, with or without modification,
 * 		 		 are permitted provided that the following conditions are met:
 * 		 		 	- Redistributions of source code must retain the above copyright notice, this list
 * 		 		 	  of conditions and the following disclaimer.
 *
 * 					- Redistributions in binary form must reproduce the above copyright notice, this
 * 					  list of conditions and the following disclaimer in the documentation and/or
 * 					  other materials provided with the distribution.
 * Author(s)   : Matheus Leitzke Pinto
 ***************************************************************************************/

#ifndef DIFF_ROBOT_DIFF_ROBOT_H_
#define DIFF_ROBOT_DIFF_ROBOT_H_

#include <diff_robot/communic.h>
#include <diff_robot/motor_ctrl.h>
#include <diff_robot/display.h>
#include <diff_robot/line_sensor.h>
#include <diff_robot/ultrasonic.h>
#include <diff_robot/pose_handler.h>
#include <diff_robot/fault.h>
#include <diff_robot/imu.h>
#include "delayer/delayer.h"
#include "FreeRTOS.h"
#include "task.h"

/* Tasks priorities. */
/*#define INIT_TASK_PRIORITY			(configMAX_PRIORITIES - 1)
#define ULTRASONIC_TASK_PRIORITY 	(configMAX_PRIORITIES - 2)
#define CONTROLLER_TASK_PRIORITY 	(configMAX_PRIORITIES - 2)
#define MOTOR_CTRL_TASK_PRIORITY 	(configMAX_PRIORITIES - 2)
#define MOTOR_ENCODER_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define MPU_TASK_PRIORITY 			(configMAX_PRIORITIES - 2)
#define POSE_TASK_PRIORITY		 	(configMAX_PRIORITIES - 2)
#define LINE_DETECT_TASK_PRIORITY 	(configMAX_PRIORITIES - 2)
#define DISPLAY_TASK_PRIORITY		(configMAX_PRIORITIES - 2)
*/
/* Tasks periods. */
/*#define ULTRASONIC_TASK_PERIOD 		(3 / portTICK_PERIOD_MS) //ms
#define CONTROLLER_TASK_PERIOD 		(3 / portTICK_PERIOD_MS) //ms
#define MOTOR_CTRL_TASK_PERIOD		(3 / portTICK_PERIOD_MS)  //ms
#define MOTOR_ENCODER_TASK_PERIOD 	(3 / portTICK_PERIOD_MS) //ms
#define MPU_TASK_PERIOD 			(3 / portTICK_PERIOD_MS) //ms
#define POSE_TASK_PERIOD		 	(3 / portTICK_PERIOD_MS) //ms
#define LINE_DETECT_TASK_PERIOD 	(3 / portTICK_PERIOD_MS) //ms
#define DISPLAY_TASK_PERIOD			(30 / portTICK_PERIOD_MS) //ms
*/


#define INIT_TASK_PRIORITY			(configMAX_PRIORITIES - 1)
#define MOTOR_ENCODER_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define MPU_TASK_PRIORITY 			(configMAX_PRIORITIES - 2)
#define POSE_TASK_PRIORITY		 	(configMAX_PRIORITIES - 3)
#define ULTRASONIC_TASK_PRIORITY 	(configMAX_PRIORITIES - 4)
#define CONTROLLER_TASK_PRIORITY 	(configMAX_PRIORITIES - 5)
#define MOTOR_CTRL_TASK_PRIORITY 	(configMAX_PRIORITIES - 6)
#define LINE_DETECT_TASK_PRIORITY 	(configMAX_PRIORITIES - 7)
#define DISPLAY_TASK_PRIORITY		(configMAX_PRIORITIES - 8)


#define IN_TEST_TASK_PRIORITY		(configMAX_PRIORITIES - 8)

/* Tasks periods. */

#define MOTOR_ENCODER_TASK_PERIOD 	(5 / portTICK_PERIOD_MS) //ms
#define MPU_TASK_PERIOD 			(5 / portTICK_PERIOD_MS) //ms
#define POSE_TASK_PERIOD		 	(6 / portTICK_PERIOD_MS) //ms
#define ULTRASONIC_TASK_PERIOD 		(10 / portTICK_PERIOD_MS) //ms
#define CONTROLLER_TASK_PERIOD 		(13 / portTICK_PERIOD_MS) //ms
#define MOTOR_CTRL_TASK_PERIOD		(15 / portTICK_PERIOD_MS)  //ms
#define LINE_DETECT_TASK_PERIOD 	(100 / portTICK_PERIOD_MS) //ms
#define DISPLAY_TASK_PERIOD			(150 / portTICK_PERIOD_MS) //ms

#define IN_TEST_TASK_PERIOD			(3 / portTICK_PERIOD_MS) //ms

#define INITIAL_WAIT_TIME			(3000 / portTICK_PERIOD_MS) //ms

void robot_Init(void);

void robot_CrtlTaskCreate(UBaseType_t uxPriority, TickType_t xPeriod);

void robot_InitTaskCreate(UBaseType_t uxPriority);

void robot_SetToGoForward(twist_t *cmdVel);

void robot_SetToGoBackward(twist_t *cmdVel);

void robot_SetToGoLowLeft(twist_t *cmdVel);

void robot_SetToGoLeft(twist_t *cmdVel);

void robot_SetToGoVeryLeft(twist_t *cmdVel);

void robot_SetToGoLowRight(twist_t *cmdVel);

void robot_SetToGoRight(twist_t *cmdVel);

void robot_SetToGoVeryRight(twist_t *cmdVel);

void robot_SetToStop(twist_t *cmdVel);

#endif /* DIFF_ROBOT_DIFF_ROBOT_H_ */

/***************************************************************************************
 * END: Module - diff_robot.h
 ***************************************************************************************/
