/*
 * fault.c
 *
 *  Created on: 5 de ago de 2019
 *      Author: Matheus_Pinto
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "fault.h"
#include "fsl_gpio.h"
#include "fsl_tpm.h"
#include "pin_mux.h"

#include "delayer/delayer.h"


void fault_Init(void)
{
}

void fault_Throw(void)
{
	// Increase task priority to not be preempted by others,
	// specially the motor task that can update the motors with a no zero value.
	vTaskPrioritySet(NULL, configMAX_PRIORITIES-1);
	// ??? For some reason, a delay is necessary in this point.
	// If not delayed, the PWM doesnt update...
	delayer_Waitms(5);
	// Stop the motors.
	portENTER_CRITICAL(); // To not be bored by ISRs.
	TPM_UpdatePwmDutycycle(TPM1, kTPM_Chnl_0, kTPM_EdgeAlignedPwm, 0);
    TPM_UpdatePwmDutycycle(TPM1, kTPM_Chnl_1, kTPM_EdgeAlignedPwm, 0);
    portEXIT_CRITICAL(); // Enable interrupts, if not, PWM will not be generate.
    //delayer_Waitms(500);
    // Call NMI ISR
	SCB->ICSR = SCB_ICSR_NMIPENDSET_Msk;
}

void fault_ThrowFromISR(void)
{
	// Stop the motors
	portENTER_CRITICAL(); // To not be bored by other ISRs
	TPM_UpdatePwmDutycycle(TPM1, kTPM_Chnl_0, kTPM_EdgeAlignedPwm, 0);
    TPM_UpdatePwmDutycycle(TPM1, kTPM_Chnl_1, kTPM_EdgeAlignedPwm, 0);
    portEXIT_CRITICAL(); // Enable interrupts, if not, PWM will not be generate.
    // Call NMI ISR
	SCB->ICSR = SCB_ICSR_NMIPENDSET_Msk;
}


void fault_VerifyDeadline(TickType_t xLastWakeTime, TickType_t xActualWakeTime, TickType_t xMpuTaskPeriod)
{
	//taskDISABLE_INTERRUPTS();
	if((xActualWakeTime - xLastWakeTime) > xMpuTaskPeriod)
	{
		fault_Throw();
	}
}

void fault_VerifyDeadlineFromISR(TickType_t xLastWakeTime, TickType_t xActualWakeTime, TickType_t xMpuTaskPeriod)
{
	//taskDISABLE_INTERRUPTS();
	if((xActualWakeTime - xLastWakeTime) > xMpuTaskPeriod)
	{
		fault_ThrowFromISR();
	}
}


void NMI_Handler(void)
{
    //	Signalize.
	for(;;)
	{
		GPIO_TogglePinsOutput(BOARD_INITPINS_LED_RED_GPIO, 1 << BOARD_INITPINS_LED_RED_PIN);
		delayer_Waitms(500);
	}
}
