/***************************************************************************************
 * Module      : diff_robot.c
 * Revision    : 1.0
 * Date        : 17/07/2019
 * Description : It contains the implementations of the differential robot controller
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

#include <diff_robot/diff_robot.h>
#include <diff_robot/measure.h>
#include <diff_robot/test.h>
#include "fsl_gpio.h"
#include "pin_mux.h"

static obst3Dist_t obstDist;
static lineSensorDir_t lineDir;

static float _maxLinearSpeed, _maxAngularSpeed;

#define robotASSERT(x) if(x == 0){fault_Throw();}

// task parameters
static TickType_t xCtrlTaskPeriod;

static void RobotCtrlTask(void *pvParameters);


/**********************************************************************
* Function		:   robot_SetToGoForward
*
* Description	:   Put a twist value pattern that makes the robot to go
*					forward.
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToGoForward(twist_t *cmdVel)
{
	cmdVel->linear.x = _maxLinearSpeed/5;
	cmdVel->angular.z = 0;
}

/**********************************************************************
* Function		:   robot_SetToGoBackward
*
* Description	:   Put a twist value pattern that makes the robot to go
*					backward.
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToGoBackward(twist_t *cmdVel)
{
	cmdVel->linear.x = -_maxLinearSpeed/5;
	cmdVel->angular.z = 0;
}

/**********************************************************************
* Function		:   robot_SetToGoLowLeft
*
* Description	:   Put a twist value pattern that makes the robot to go
*					low left.
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToGoLowLeft(twist_t *cmdVel)
{
	cmdVel->linear.x = _maxLinearSpeed/5;
	// cmdVel->angular.z = _maxAngularSpeed/5; //motor do not spin
	cmdVel->angular.z = _maxAngularSpeed/2;
}

/**********************************************************************
* Function		:   robot_SetToGoLeft
*
* Description	:   Put a twist value pattern that makes the robot to go
*					left.
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToGoLeft(twist_t *cmdVel)
{
	cmdVel->linear.x = 0;
	// cmdVel->angular.z = _maxAngularSpeed/4; //motor do not spin
	cmdVel->angular.z = _maxAngularSpeed/2;
}

/**********************************************************************
* Function		:   robot_SetToGoVeryLeft
*
* Description	:   Put a twist value pattern that makes the robot to go
*					very left.
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToGoVeryLeft(twist_t *cmdVel)
{
	cmdVel->linear.x  = 0;
	//cmdVel->angular.z = _maxAngularSpeed/3; // motor do not spin.
	cmdVel->angular.z = _maxAngularSpeed/2;
}

/**********************************************************************
* Function		:   robot_SetToGoLowRight
*
* Description	:   Put a twist value pattern that makes the robot to go
*					low right.
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToGoLowRight(twist_t *cmdVel)
{
	cmdVel->linear.x  = _maxLinearSpeed/5;
	//cmdVel->angular.z = -_maxAngularSpeed/5; // motor do not spin
	cmdVel->angular.z = -_maxAngularSpeed/2;
}

/**********************************************************************
* Function		:   robot_SetToGoRight
*
* Description	:   Put a twist value pattern that makes the robot to go
*					right .
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToGoRight(twist_t *cmdVel)
{
	cmdVel->linear.x = 0;
	//cmdVel->angular.z = -_maxAngularSpeed/4; //motor do not spin
	cmdVel->angular.z = -_maxAngularSpeed/2;
}

/**********************************************************************
* Function		:   robot_SetToGoVeryRight
*
* Description	:   Put a twist value pattern that makes the robot to go
*					very right .
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToGoVeryRight(twist_t *cmdVel)
{
	cmdVel->linear.x = 0;
	//cmdVel->angular.z = -_maxAngularSpeed/3; //motor do not spin
	cmdVel->angular.z = -_maxAngularSpeed/2;
}

/**********************************************************************
* Function		:   robot_SetToStop
*
* Description	:   Put a twist value pattern that makes robot stop.
*
* Inputs		:   *cmdVel - the memory position that will store the value.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_SetToStop(twist_t *cmdVel)
{
	cmdVel->linear.x = 0;
	cmdVel->angular.z = 0;
}

/**********************************************************************
* Function		:   RobotInitTask
*
* Description	:   Contains the task for initialization implementation,
* 					called through robot_InitTaskCreate.
*
* Inputs		:   *pvParameters - user arguments.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
static void RobotInitTask(void *pvParameters)
{
	/*Tasks initialization.*/
	robot_CrtlTaskCreate(CONTROLLER_TASK_PRIORITY, CONTROLLER_TASK_PERIOD);
	ultrasonic_TaskCreate(ULTRASONIC_TASK_PRIORITY, ULTRASONIC_TASK_PERIOD);
	motor_CtrlTaskCreate(MOTOR_CTRL_TASK_PRIORITY, MOTOR_CTRL_TASK_PERIOD);
	motor_OdomTaskCreate(MOTOR_ENCODER_TASK_PRIORITY, MOTOR_ENCODER_TASK_PERIOD);
	lineDetect_TaskCreate(ULTRASONIC_TASK_PRIORITY, MOTOR_ENCODER_TASK_PERIOD);
	display_TaskCreate(DISPLAY_TASK_PRIORITY, DISPLAY_TASK_PERIOD);
	poseHand_TaskCreate(POSE_TASK_PRIORITY, POSE_TASK_PERIOD);
	imu_TaskCreate(MPU_TASK_PRIORITY, MPU_TASK_PERIOD);
	//inTest_TaskCreate(IN_TEST_TASK_PRIORITY, IN_TEST_TASK_PERIOD); // comment this line when not testing the robot components

	vTaskDelay(INITIAL_WAIT_TIME);

	while(!GPIO_ReadPinInput(BOARD_INITPINS_KEY_FRONT_GPIO, BOARD_INITPINS_KEY_FRONT_PIN))
	{

	}

	communic_NotifyInitSignal();
	vTaskDelay(portMAX_DELAY);
	//vTaskSuspend(NULL);
	for(;;);
}

/**********************************************************************
* Function		:   RobotCtrlTask
*
* Description	:   Contains the task twist controllers implementation,
* 					called through robot_CrtlTaskCreate.
*
* Inputs		:   *pvParameters - user arguments.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
static void RobotCtrlTask(void *pvParameters)
{
	TickType_t xLastWakeTime, xActualWakeTime;
	twist_t cmdVel;

	communic_WaitInitSignal();
	xActualWakeTime = xLastWakeTime = communic_GetInitalTime(); // Get the first task activation
	for(;;)
    {
		communic_ReceiveObstFrontDist(&obstDist.front);
		communic_ReceiveObstRightDist(&obstDist.right);
		communic_ReceiveObstLeftDist(&obstDist.left);

		if(/*obstDist.front < 15 || obstDist.right < 15 ||*/ obstDist.left < 15)
		{
			robot_SetToStop(&cmdVel);
		}
		else
		{
				//communic_ReceiveLineDir(&lineDir);
				switch(GPIOE->PDIR & IR_PINS_MASK)
				{
				case 0x20E00000: // very right IR
				case 0x00E00000: // very right IR and right IR
				case 0x00600000: // very right IR and right IR and front IR
					robot_SetToGoVeryRight(&cmdVel);
					break;
				case 0x40E00000: // right IR
					robot_SetToGoRight(&cmdVel);
					break;
				case 0x40600000: // right IR and front IR
				case 0x60600000: // front IR
				case 0x60200000: // left IR and front IR
					robot_SetToGoForward(&cmdVel);
					break;
				case 0x60A00000: // left IR
					robot_SetToGoLeft(&cmdVel);;
					break;
				case 0x60C00000: // very left IR
				case 0x60800000: // very left IR and left IR
				case 0x60000000: // very left IR and left IR and front IR
					robot_SetToGoVeryLeft(&cmdVel);
					break;
				case 0x60E00000: // no line
					robot_SetToGoForward(&cmdVel);
					break;
				//default: // no pattern in sensors reading, so spin to find a pattern.
					//robot_SetToGoRight(&cmdVel);
				}

		}

		communic_SendCmdVel(&cmdVel);
		measure_CodeEndCatch();
	    /* Delays until next activation. */
        vTaskDelayUntil(&xActualWakeTime, xCtrlTaskPeriod);
        measure_CodeBeginCatch();
        fault_VerifyDeadline(xLastWakeTime, xActualWakeTime, xCtrlTaskPeriod);
        xLastWakeTime = xActualWakeTime;
    }

}

/**********************************************************************
* Function		:   robot_Init
*
* Description	:   Initialize all modules necessary to robot to operates.
*
* Inputs		:   None.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_Init(void)
{
	_maxLinearSpeed  = motorMAX_ROTATIONAL_SPEED*motorWHELL_RADIUS;
	_maxAngularSpeed = 3.14;

	delayer_Init();
	communic_Init();
	fault_Init();
	display_Init();
	motor_Init();
	ultrasonic_Init();
	lineDetect_Init();
	poseHand_Init();
	robotASSERT(imu_Init());

	robot_InitTaskCreate(INIT_TASK_PRIORITY);
}

/**********************************************************************
* Function		:   robot_InitTaskCreate
*
* Description	:   Initialize the task responsible to start all system
* 					tasks. After initialization, this tasks will not
* 					operate anymore.
*
* Inputs		:   uxPriority : task priority.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_InitTaskCreate(UBaseType_t uxPriority)
{
	if (xTaskCreate(RobotInitTask, "init", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL) != pdPASS)
    {
		while(1);
    }
}

/**********************************************************************
* Function		:   robot_CrtlTaskCreate
*
* Description	:   Initialize the task whose control robot twist.
*
* Inputs		:   - uxPriority : task priority.
* 					- xPeriod	 : task activation period.
*
* Outputs 		:   None.
*
* Comments 		:   None.
* ********************************************************************/
void robot_CrtlTaskCreate(UBaseType_t uxPriority, TickType_t xPeriod)
{
	xCtrlTaskPeriod = xPeriod;

    if (xTaskCreate(RobotCtrlTask, "ctrlr", configMINIMAL_STACK_SIZE + 10, NULL, uxPriority, NULL) != pdPASS)
    {
    	while(1);
    }
}

/***************************************************************************************
 * END: Module - diff_robot.c
 ***************************************************************************************/
