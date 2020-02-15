/*
 * ultrasonic.c
 *
 *  Created on: 2 de jul de 2019
 *      Author: Matheus_Pinto
 */

#include <diff_robot/communic.h>
#include "FreeRTOS.h"
#include "delayer/delayer.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_tpm.h"
#include "pin_mux.h"
#include <diff_robot/fault.h>
#include <diff_robot/ultrasonic.h>
#include <diff_robot/measure.h>

#define ultraTIMER_PERIOD 0xBFFFU
#define ultraTIMER_RESOLUTION 2667 //ns

typedef enum
{
	ECHO_NOT_TRIGGERED = 0,
	ECHO_WAITING_RESPONSE = 1,
	ECHO_IS_CAPTURED = 2
}echoStatus_t;

/*The echo timeout is necessary in the case that timer IRQ handler is disabled by the RTOS, and for that the pulse trigger
 * has to be launched again.*/
static uint8_t _echoTimeOut;

// TPM0 configurations
static tpm_config_t tpm0_config;

// Stores the times when echo pulses is rising or falling
static uint16_t pulseTimeRiseLeft, pulseTimeFallLeft, pulseTimeRiseFront, pulseTimeFallFront, pulseTimeRiseRight, pulseTimeFallRight;

// Indicates to TPM0 ISR if the echo pulse is a rising or falling edge
static uint8_t isLeftRise = 1, isFrontRise = 1, isRightRise = 1;

// Ultrasonic task parameters
//static TaskHandle_t pxUltraTaskHandle;
static TickType_t xUltraTaskPeriod;

static SemaphoreHandle_t xRightEchoSemphr, xLeftEchoSemphr, xFrontEchoSemphr;
//static QueueHandle_t xRightEchoStatusQueue, xLeftEchoStatusQueue, xFrontEchoStatusQueue;

#define SetRightEchoStatus(status) {xSemaphoreTake(xRightEchoStatusMutex, portMAX_DELAY); rightEchoStatus = status; xSemaphoreGive(xRightEchoStatusMutex);}
#define SetLeftEchoStatus(status) {xSemaphoreTake(xLeftEchoStatusMutex, portMAX_DELAY); leftEchoStatus = status; xSemaphoreGive(xLeftEchoStatusMutex);}
#define SetFrontEchoStatus(status) {xSemaphoreTake(xFrontEchoStatusMutex, portMAX_DELAY); frontEchoStatus = status; xSemaphoreGive(xFrontEchoStatusMutex);}

#define SetRightEchoStatusFromISR(status, isWokenTask) {xSemaphoreTakeFromISR(xRightEchoStatusMutex, isWokenTask); rightEchoStatus = status; xSemaphoreGiveFromISR(xRightEchoStatusMutex, isWokenTask);}
#define SetLeftEchoStatusFromISR(status, isWokenTask) {xSemaphoreTakeFromISR(xLeftEchoStatusMutex, isWokenTask); leftEchoStatus = status; xSemaphoreGiveFromISR(xLeftEchoStatusMutex, isWokenTask);}
#define SetFrontEchoStatusFromISR(status, isWokenTask) {xSemaphoreTakeFromISR(xFrontEchoStatusMutex, isWokenTask); frontEchoStatus = status; xSemaphoreGiveFromISR(xFrontEchoStatusMutex, isWokenTask);}

static void UltrassonicTask(void *pvParameters);

void TPM0_IRQHandler(void)
{
#ifdef ultraMEASURE_ULTRASONIC_ISR
	measure_CodeBeginCatch();
#endif
	BaseType_t pxHigherPriorityTaskWoken;

	if(TPM_GetStatusFlags(TPM0) & kTPM_Chnl4Flag)// TPM0 channel 4 cause IRQ?
	{
		if(isRightRise) // is rising edge?
		{
			pulseTimeRiseRight = TPM0->CONTROLS[4].CnV; // Get value from TPM0_C4V
			isRightRise = 0;
		}
		else // is falling edge?
		{
			pulseTimeFallRight = TPM0->CONTROLS[4].CnV; // Get value from TPM0_C4V
			isRightRise = 1;
			// Semaphore that indicate that a echo value was captured
			xSemaphoreGiveFromISR(xRightEchoSemphr, &pxHigherPriorityTaskWoken);
		}
	    TPM_ClearStatusFlags(TPM0, kTPM_Chnl4Flag);
    }
	else
	{
		if(TPM_GetStatusFlags(TPM0) & kTPM_Chnl0Flag)// PTD0 causou interrupção?
		{
			if(isFrontRise) // is rising edge?
			{
				pulseTimeRiseFront = TPM0->CONTROLS[0].CnV; // Get value from TPM0_C0V
				isFrontRise = 0;
			}
			else // is falling edge?
			{
				pulseTimeFallFront = TPM0->CONTROLS[0].CnV; // Get value from TPM0_C0V
			    isFrontRise = 1;
			    // Semaphore that indicate that a echo value was captured
			    xSemaphoreGiveFromISR(xFrontEchoSemphr, &pxHigherPriorityTaskWoken);
			}
		    TPM_ClearStatusFlags(TPM0, kTPM_Chnl0Flag);
	    }
		else
		{
			if(TPM_GetStatusFlags(TPM0) & kTPM_Chnl1Flag)// PTC2 causou interrupção?
			{
				if(isLeftRise) // is rising edge?
				{
					pulseTimeRiseLeft = TPM0->CONTROLS[1].CnV; // Get value from TPM0_C1V
					isLeftRise = 0;
				}
				else // is falling edge?
				{
					pulseTimeFallLeft = TPM0->CONTROLS[1].CnV; // Get value from TPM0_C1V
					isLeftRise = 1;
					// Semaphore that indicate that a echo value was captured
					xSemaphoreGiveFromISR(xLeftEchoSemphr, &pxHigherPriorityTaskWoken);
				}
			    TPM_ClearStatusFlags(TPM0, kTPM_Chnl1Flag);
		    }
		}
	}

	portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
#ifdef ultraMEASURE_ULTRASONIC_ISR
	measure_CodeEndCatch();
#endif
}

void ultrasonic_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod)
{
	xUltraTaskPeriod = xPeriod;

	_echoTimeOut = 200/xPeriod;

	if(_echoTimeOut == 0){
		_echoTimeOut = 1;
	}

//    if (xTaskCreate(UltrassonicFrontTask, "ul1", configMINIMAL_STACK_SIZE, NULL, uxPriority, &pxUltraTaskHandle) != pdPASS)
    if (xTaskCreate(UltrassonicTask, "uls", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL) != pdPASS)
    {
           while(1);
    }
}


static void UltrassonicTask(void *pvParameters)
{
	// The echo pulse pin status transaction status.
	uint8_t echoRightStatus, echoLeftStatus, echoFrontStatus, echoLeftTimeOut = 0, echoRightTimeOut = 0, echoFrontTimeOut = 0;
	uint16_t distanceCm;
	uint32_t pulseWidth;
	TickType_t xLastWakeTime, xActualWakeTime;

	echoRightStatus = echoLeftStatus = echoFrontStatus = ECHO_NOT_TRIGGERED;
	communic_WaitInitSignal();
	xActualWakeTime = xLastWakeTime = communic_GetInitalTime(); // Get the first task activation
	for(;;)
    {
		/*If trigger pulse was not sent or a timeout response from echo pin was passed,
		 * a new trigger pulse will be launched. */
		if((echoRightStatus == ECHO_NOT_TRIGGERED) || (echoRightTimeOut == _echoTimeOut))
		{
			echoRightStatus = ECHO_WAITING_RESPONSE;
			isRightRise = 1; /*In timeout scenario, this will warn the IRQ handler that a new pulse was generated.*/

			GPIO_WritePinOutput(BOARD_INITPINS_ULTRA_TRIG_R_GPIO, BOARD_INITPINS_ULTRA_TRIG_R_PIN, 1); //trigger ultrasonic right
			delayer_Waitus(10);
			GPIO_WritePinOutput(BOARD_INITPINS_ULTRA_TRIG_R_GPIO, BOARD_INITPINS_ULTRA_TRIG_R_PIN, 0); //trigger ultrasonic right

			echoRightTimeOut = 0;
		}
		else
		{
			if(xSemaphoreTake(xRightEchoSemphr, 0) == pdTRUE)
			{
				// Calculate the pulse width in nanoseconds
				if(pulseTimeFallRight > pulseTimeRiseRight)
				{
					pulseWidth = (ultraTIMER_RESOLUTION*(pulseTimeFallRight - pulseTimeRiseRight));
				}
				else
				{
					pulseWidth = (ultraTIMER_RESOLUTION*(ultraTIMER_PERIOD - pulseTimeRiseRight + pulseTimeFallRight));
				}

				// calculate the distance measured in cm and inch
				// dived to 1000 because timer resolution is in ns and want to us
				distanceCm = (uint16_t)(pulseWidth/58000);
				//distanceInch = (uint16_t)(pulseWidth/148000);
				/***************************************************************************************************/

				communic_SendObstRightDist(&distanceCm);
				echoRightStatus = ECHO_NOT_TRIGGERED;
			}
			else
			{
				++echoRightTimeOut;
			}
		}

		if((echoLeftStatus == ECHO_NOT_TRIGGERED) || (echoLeftTimeOut == _echoTimeOut))
		{
			echoLeftStatus = ECHO_WAITING_RESPONSE;
			isLeftRise = 1; /*In timeout scenario, this will warn the IRQ handler that a new pulse was generated.*/

			GPIO_WritePinOutput(BOARD_INITPINS_ULTRA_TRIG_L_GPIO, BOARD_INITPINS_ULTRA_TRIG_L_PIN, 1); //trigger ultrasonic right
			delayer_Waitus(10);
			GPIO_WritePinOutput(BOARD_INITPINS_ULTRA_TRIG_L_GPIO, BOARD_INITPINS_ULTRA_TRIG_L_PIN, 0); //trigger ultrasonic right

			echoLeftTimeOut = 0;
		}
		else
		{
			if(xSemaphoreTake(xLeftEchoSemphr, 0) == pdTRUE)
			{
				// Calculate the pulse width in nanoseconds
				if(pulseTimeFallLeft > pulseTimeRiseLeft)
				{
					pulseWidth = (ultraTIMER_RESOLUTION*(pulseTimeFallLeft - pulseTimeRiseLeft));
				}
				else
				{
					pulseWidth = (ultraTIMER_RESOLUTION*(ultraTIMER_PERIOD - pulseTimeRiseLeft + pulseTimeFallLeft));
				}

				// calculate the distance measured in cm and inch
				// dived to 1000 because timer resolution is in ns and want to us
				distanceCm = (uint16_t)(pulseWidth/58000);
				//distanceInch = (uint16_t)(pulseWidth/148000);
				/***************************************************************************************************/

				communic_SendObstLeftDist(&distanceCm);
				echoLeftStatus = ECHO_NOT_TRIGGERED;
			}
			else
			{
				++echoLeftTimeOut;
			}
		}

		if((echoFrontStatus == ECHO_NOT_TRIGGERED) || (echoFrontTimeOut == _echoTimeOut))
		{
			echoFrontStatus = ECHO_WAITING_RESPONSE;
			isFrontRise = 1; /*In timeout scenario, this will warn the IRQ handler that a new pulse was generated.*/

			GPIO_WritePinOutput(BOARD_INITPINS_ULTRA_TRIG_F_GPIO, BOARD_INITPINS_ULTRA_TRIG_F_PIN, 1); //trigger ultrasonic right
			delayer_Waitus(10);
			GPIO_WritePinOutput(BOARD_INITPINS_ULTRA_TRIG_F_GPIO, BOARD_INITPINS_ULTRA_TRIG_F_PIN, 0); //trigger ultrasonic right

			echoFrontTimeOut = 0;
		}
		else
		{
			if(xSemaphoreTake(xFrontEchoSemphr, 0) == pdTRUE)
			{
				// Calculate the pulse width in nanoseconds
				if(pulseTimeFallFront > pulseTimeRiseFront)
				{
					pulseWidth = (ultraTIMER_RESOLUTION*(pulseTimeFallFront - pulseTimeRiseFront));
				}
				else
				{
					pulseWidth = (ultraTIMER_RESOLUTION*(ultraTIMER_PERIOD - pulseTimeRiseFront + pulseTimeFallFront));
				}

				// calculate the distance measured in cm and inch
				// dived to 1000 because timer resolution is in ns and want to us
				distanceCm = (uint16_t)(pulseWidth/58000);
				//distanceInch = (uint16_t)(pulseWidth/148000);
				/***************************************************************************************************/

				communic_SendObstFrontDist(&distanceCm);
				echoFrontStatus = ECHO_NOT_TRIGGERED;
			}
			else
			{
				++echoFrontTimeOut;
			}
		}

#ifdef ultraMEASURE_ULTRASONIC_TASK
	measure_CodeEndCatch();
#endif
		/* Delays until next activation. */
	    vTaskDelayUntil(&xActualWakeTime, xUltraTaskPeriod);
#ifdef ultraMEASURE_ULTRASONIC_TASK
	measure_CodeBeginCatch();
#endif
        fault_VerifyDeadline(xLastWakeTime, xActualWakeTime, xUltraTaskPeriod);
        xLastWakeTime = xActualWakeTime;

    }

}

void ultrasonic_Init(void)
{
	delayer_Waitms(500); // Wait GPIOs for trigger and echo pins stabilize on low level

	//xFrontEchoStatusQueue = xQueueCreate(1, sizeof(uint16_t));
	//xRightEchoStatusQueue = xQueueCreate(1, sizeof(uint16_t));
	//xLeftEchoStatusQueue = xQueueCreate(1, sizeof(uint16_t));

	xFrontEchoSemphr = xSemaphoreCreateBinary();
	xRightEchoSemphr = xSemaphoreCreateBinary();
	xLeftEchoSemphr = xSemaphoreCreateBinary();

	// Basic TPM0 channel 0 hardware initialization on PTB2 pin like input capture
	TPM_GetDefaultConfig(&tpm0_config);
	tpm0_config.prescale = kTPM_Prescale_Divide_128;
	TPM_Init(TPM0, &tpm0_config);

	CLOCK_SetTpmClock(1);
	TPM_SetTimerPeriod(TPM0, ultraTIMER_PERIOD);

	TPM_SetupInputCapture(TPM0, kTPM_Chnl_0, kTPM_RisingEdge|kTPM_FallingEdge); // PTD0
	TPM_SetupInputCapture(TPM0, kTPM_Chnl_1, kTPM_RisingEdge|kTPM_FallingEdge); // PTC2
	TPM_SetupInputCapture(TPM0, kTPM_Chnl_4, kTPM_RisingEdge|kTPM_FallingEdge); // PTE31

	TPM_EnableInterrupts(TPM0, kTPM_Chnl0InterruptEnable|kTPM_Chnl1InterruptEnable|kTPM_Chnl4InterruptEnable);
	NVIC_EnableIRQ(TPM0_IRQn);

	TPM_StartTimer(TPM0, kTPM_SystemClock);
}
