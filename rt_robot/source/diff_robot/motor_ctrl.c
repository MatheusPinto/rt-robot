/*
 * motor_ctrl.c
 *
 *  Created on: 1 de jul de 2019
 *      Author: Matheus_Pinto
 */
#include <diff_robot/communic.h>
#include "motor_ctrl.h"

#include "MKL25Z4.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_tpm.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include <diff_robot/fault.h>
#include <diff_robot/measure.h>

#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "emb_util/emb_util.h"

#define motorENCODER_STEPS 21
#define motorWHEEL_IS_FORWARD 0
#define motorWHEEL_IS_BACKWARD 1
#define motorLEFT_WHEEL_ERROR_SPEED 0.000
#define motorRIGHT_WHEEL_ERROR_SPEED 0.000

#define motorMEASURE_ENCODER_TASK
#ifndef motorMEASURE_ENCODER_TASK
#define motorMEASURE_ENCODER_ISR
#endif

//#define motorMEASURE_CTRL_TASK
#ifndef motorMEASURE_CTRL_TASK
#define motorMEASURE_PWM_CTRL_BLOCK
#endif

tpm_config_t tpm1_config;

tpm_chnl_pwm_signal_param_t tpm1_chnls_param[2] = {{kTPM_Chnl_0, kTPM_HighTrue, 0}, {kTPM_Chnl_1, kTPM_HighTrue, 0}};

static const gpio_pin_config_t config_as_output_low = {kGPIO_DigitalOutput, 0};

// zero (0) is front, one (1) is back
static uint8_t rightWheelDir = motorWHEEL_IS_FORWARD, leftWheelDir = motorWHEEL_IS_FORWARD;

static TickType_t xMotorCtrlTaskPeriod, xMotorOdomTaskPeriod;

static float _maxWheelSpeed;

QueueHandle_t xRightEncoderQueue, xLeftEncoderQueue;

#define ENCODER_ISR_MIN_INTERARRIVEL	0 //(3 / portTICK_PERIOD_MS) //ms

static void MotorCtrlTask(void *pvParameters);

static void EncondersHandlerTask(void *pvParameters);


void PORTD_IRQHandler(void)
{
	//TickType_t xStartTime, xCompleteTime;
#ifdef motorMEASURE_ENCODER_ISR
	measure_CodeBeginCatch();
#endif

	BaseType_t pxHigherPriorityTaskWoken;

	static uint32_t rightEncoderCounter = 0, leftEncoderCounter = 0;

	// xTaskGetTickCountFromISR() get the tick count before ISR, so its necessary
	// in future, measure execution time with the hardware timer.
	//xStartTime = xTaskGetTickCountFromISR();

	if(PORT_GetPinsInterruptFlags(PORTD) & 1<<2) // PTD2 causou interrupção?
    {
		if(xQueueIsQueueFullFromISR(xRightEncoderQueue))
		{
			xQueueReceiveFromISR(xRightEncoderQueue, &rightEncoderCounter, &pxHigherPriorityTaskWoken);
			++rightEncoderCounter;
		}
		else
		{
			rightEncoderCounter = 1;
		}
		xQueueSendFromISR(xRightEncoderQueue, &rightEncoderCounter, &pxHigherPriorityTaskWoken);

	    PORT_ClearPinsInterruptFlags(PORTD, 1 << 2);
    }
	else
	{
		if(PORT_GetPinsInterruptFlags(PORTD) & 1<<3)// PTD3 causou interrupção?
		{

			if(xQueueIsQueueFullFromISR(xLeftEncoderQueue))
			{
				xQueueReceiveFromISR(xLeftEncoderQueue, &leftEncoderCounter, &pxHigherPriorityTaskWoken);
				++leftEncoderCounter;
			}
			else
			{
				leftEncoderCounter = 1;
			}
			xQueueSendFromISR(xRightEncoderQueue, &rightEncoderCounter, &pxHigherPriorityTaskWoken);

		    PORT_ClearPinsInterruptFlags(PORTD, 1 << 3);
		}
	}

    portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
    //xCompleteTime = xTaskGetTickCountFromISR();
    //fault_VerifyDeadlineFromISR(xStartTime, xCompleteTime, ENCODER_ISR_MIN_INTERARRIVEL);
#ifdef motorMEASURE_ENCODER_ISR
	measure_CodeEndCatch();
#endif
}

void motor_Init(void)
{
	xLeftEncoderQueue = xQueueCreate(1, sizeof(uint32_t));
	xRightEncoderQueue = xQueueCreate(1, sizeof(uint32_t));

	_maxWheelSpeed = motorMAX_ROTATIONAL_SPEED*motorWHELL_RADIUS;

	// Motor Control peripherals
	GPIO_PinInit(GPIOC, 7, &config_as_output_low); // Motor IN1
	GPIO_PinInit(GPIOC, 0, &config_as_output_low); // Motor IN2
	GPIO_PinInit(GPIOC, 3, &config_as_output_low); // Motor IN3
	GPIO_PinInit(GPIOC, 4, &config_as_output_low); // Motor IN4

	// Basic TPM1 hardware initialization for channel 0 (PTA12) and channel 1 (PTB1) like PWM
	TPM_GetDefaultConfig(&tpm1_config);
	tpm1_config.prescale = kTPM_Prescale_Divide_128;
	TPM_Init(TPM1, &tpm1_config);

	CLOCK_SetTpmClock(1);

	TPM_SetupPwm(TPM1,
				 tpm1_chnls_param,
				 2,
				 kTPM_EdgeAlignedPwm,
				 50, // Hertz
				 CLOCK_GetPllFllSelClkFreq());

	TPM_StartTimer(TPM1, kTPM_SystemClock);

	// Motor Odometry peripherals
	NVIC_EnableIRQ(PORTD_IRQn);
}

#ifdef motorTEST_ENCODER_TASK
void motor_SetIncrementRightEncoder(uint32_t encoderCounter)
{
	xQueueSend(xRightEncoderQueue, &encoderCounter, portMAX_DELAY);
}

void motor_SetIncrementLeftEncoder(uint32_t encoderCounter)
{
	xQueueSend(xLeftEncoderQueue, &encoderCounter, portMAX_DELAY);
}

#endif

float motor_GetMaxSpeed(void)
{
	return _maxWheelSpeed;
}

void motor_CtrlTaskCreate(UBaseType_t uxPriority, TickType_t xPeriod)
{
	xMotorCtrlTaskPeriod = xPeriod;
    if (xTaskCreate(MotorCtrlTask, "motorCtrl", configMINIMAL_STACK_SIZE + 10, NULL, uxPriority, NULL) != pdPASS)
    {
           while(1);
    }
}

void motor_OdomTaskCreate(UBaseType_t uxPriority, TickType_t xPeriod)
{
	xMotorOdomTaskPeriod = xPeriod;
    if (xTaskCreate(EncondersHandlerTask, "motorOdom", configMINIMAL_STACK_SIZE + 10, NULL, uxPriority, NULL) != pdPASS)
    {
           while(1);
    }
}


static void MotorCtrlTask(void *pvParameters)
{
	TickType_t xLastWakeTime, xActualWakeTime;
	uint8_t rightDutycycle, leftDutycycle;
	twist_t cmdVel;

	communic_WaitInitSignal();
	xActualWakeTime = xLastWakeTime = communic_GetInitalTime(); // Get the first task activation
	for(;;)
    {
		communic_ReceiveCmdVel(&cmdVel);
		// calculate kinematics
		float rightWheelSpeed = (cmdVel.angular.z*motorWHEEL_MIDDLE_AXLE)/2 + cmdVel.linear.x;
		float leftWheelSpeed = cmdVel.linear.x*2-rightWheelSpeed;

		// calculate duty cycles before sending commands to pins
		if(emb_Abs(rightWheelSpeed) > (_maxWheelSpeed + motorRIGHT_WHEEL_ERROR_SPEED))
		{
			rightDutycycle = 100U;
		}
		else
		{
			rightDutycycle = (uint8_t)(emb_Abs(rightWheelSpeed)*100U/_maxWheelSpeed);
		}

		if(emb_Abs(leftWheelSpeed) > (_maxWheelSpeed + motorLEFT_WHEEL_ERROR_SPEED))
		{
			leftDutycycle = 100U;
		}
		else
		{
			leftDutycycle = (uint8_t)(emb_Abs(leftWheelSpeed)*100U/_maxWheelSpeed);
		}

		// Evaluate motor right direction
		if(rightWheelSpeed > 0)
		{
			// M_IN3 = 1 and M_IN4 = 0, motor right forward
			GPIO_WritePinOutput(GPIOC, 3, 1);
			GPIO_WritePinOutput(GPIOC, 4, 0);
	        rightWheelDir = motorWHEEL_IS_FORWARD;
		}
		else
		{
			// M_IN3 = 0 and M_IN4 = 1, motor right backward
			GPIO_WritePinOutput(GPIOC, 3, 0);
			GPIO_WritePinOutput(GPIOC, 4, 1);
	        rightWheelDir = motorWHEEL_IS_BACKWARD;
		}

		// Evaluate motor left direction
		if(leftWheelSpeed > 0)
		{
			// M_IN1 = 1 and M_IN2 = 0, motor left forward
			GPIO_WritePinOutput(GPIOC, 7, 1);
			GPIO_WritePinOutput(GPIOC, 0, 0);
	        leftWheelDir = motorWHEEL_IS_FORWARD;
		}
		else
		{
			// M_IN1 = 0 and M_IN2 = 1, motor left backward
			GPIO_WritePinOutput(GPIOC, 7, 0);
			GPIO_WritePinOutput(GPIOC, 0, 1);
	        leftWheelDir = motorWHEEL_IS_BACKWARD;
		}

#ifdef motorMEASURE_PWM_CTRL_BLOCK
		measure_CodeBeginCatch();
#endif
		taskENTER_CRITICAL();
        TPM_UpdatePwmDutycycle(TPM1, kTPM_Chnl_0, kTPM_EdgeAlignedPwm, rightDutycycle);
        TPM_UpdatePwmDutycycle(TPM1, kTPM_Chnl_1, kTPM_EdgeAlignedPwm, leftDutycycle);
        taskEXIT_CRITICAL();

        measure_CodeEndCatch();

        /* Delays to see the change of LED brightness. */
        vTaskDelayUntil(&xActualWakeTime, xMotorCtrlTaskPeriod);

#ifdef motorMEASURE_CTRL_TASK
        measure_CodeBeginCatch();
#endif
        fault_VerifyDeadline(xLastWakeTime, xActualWakeTime, xMotorCtrlTaskPeriod);
        xLastWakeTime = xActualWakeTime;

        /* Starts PWM mode with an updated duty cycle. */
    }
}

static void EncondersHandlerTask(void *pvParameters)
{
	TickType_t xLastWakeTime, xActualWakeTime;
	float robotDistance = 0;
	float distanceLeft, distanceRight;
	uint32_t leftCount, rightCount;

	communic_WaitInitSignal();
	xActualWakeTime = xLastWakeTime = communic_GetInitalTime(); // Get the first task activation
	for(;;)
    {
		leftCount = rightCount = 0;
		xQueueReceive(xRightEncoderQueue, &rightCount, 0);
		xQueueReceive(xLeftEncoderQueue, &leftCount, 0);

		distanceLeft = (emb_2_PI * motorWHELL_RADIUS * (float)(leftCount))/(float)(motorENCODER_STEPS);
		distanceRight = (emb_2_PI * motorWHELL_RADIUS * (float)(rightCount))/(float)(motorENCODER_STEPS);

		// the direction is defined is motor_control task
		if(leftWheelDir == motorWHEEL_IS_BACKWARD)
		{
			distanceLeft = -distanceLeft;
		}
		if(rightWheelDir == motorWHEEL_IS_BACKWARD)
		{
			distanceRight = -distanceRight;
		}

		robotDistance = (distanceLeft + distanceRight)/2.0000;
		//robotAngle = (distanceRight - distanceLeft)/motorWHEEL_MIDDLE_AXLE;

		communic_SendDelthaDist(&robotDistance);
		//communic_SendDelthaTheta(&robotAngle);

#ifdef motorMEASURE_ENCODER_TASK
		measure_CodeEndCatch();
#endif
		/* Delays until next activation. */
        vTaskDelayUntil(&xActualWakeTime, xMotorOdomTaskPeriod);

#ifdef motorMEASURE_ENCODER_TASK
        measure_CodeBeginCatch();
#endif

        fault_VerifyDeadline(xLastWakeTime, xActualWakeTime, xMotorOdomTaskPeriod);
        xLastWakeTime = xActualWakeTime;
    }

}

