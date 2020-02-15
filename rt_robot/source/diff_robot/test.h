/*
 * test.h
 *
 *  Created on: 18 de jul de 2019
 *      Author: Matheus_Pinto
 */

#ifndef DIFF_ROBOT_TEST_H_
#define DIFF_ROBOT_TEST_H_

#include <diff_robot/communic.h>
#include <diff_robot/motor_ctrl.h>
#include <diff_robot/display.h>
#include <diff_robot/line_sensor.h>
#include <diff_robot/ultrasonic.h>
#include "FreeRTOS.h"
#include "task.h"

#define ROBOT_TEST_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define ROBOT_TEST_TASK_PERIOD	 (5000 / portTICK_PERIOD_MS) //ms

#define TEST_ROBOT
#ifdef TEST_ROBOT

#define TEST_CTRL
#ifndef TEST_CTRL


#define TEST_MOTOR
#ifndef TEST_MOTOR

#define TEST_DISPLAY
#ifndef TEST_DISPLAY

//#define TEST_POSE_HAND
#ifndef TEST_POSE_HAND

#define TEST_ENCODER
#ifndef TEST_ENCODER

#endif //TEST_ENCODER

#endif //TEST_POSE_HAND

#endif //TEST_DISPLAY

#endif //TEST_MOTOR

#endif //TEST_CTRL

void inTest_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod);

#endif // TEST_ROBOT

#endif /* DIFF_ROBOT_TEST_H_ */
