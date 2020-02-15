/*
 * fault.h
 *
 *  Created on: 5 de ago de 2019
 *      Author: Matheus_Pinto
 */

#ifndef DIFF_ROBOT_FAULT_H_
#define DIFF_ROBOT_FAULT_H_

void fault_Init(void);

void fault_VerifyDeadline(TickType_t xLastWakeTime, TickType_t xActualWakeTime, TickType_t xMpuTaskPeriod);

void fault_VerifyDeadlineFromISR(TickType_t xLastWakeTime, TickType_t xActualWakeTime, TickType_t xMpuTaskPeriod);

void fault_Throw(void);

void fault_ThrowFromISR(void);

#endif /* DIFF_ROBOT_FAULT_H_ */
