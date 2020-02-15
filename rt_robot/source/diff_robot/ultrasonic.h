/*
 * ultrasonic.h
 *
 *  Created on: 2 de jul de 2019
 *      Author: Matheus_Pinto
 */

#ifndef DIFF_ROBOT_ULTRASONIC_H_
#define DIFF_ROBOT_ULTRASONIC_H_

#define ultraMAX_DISTANCE 400 //cm
//#define ultraMEASURE_ULTRASONIC_ISR
#ifndef ultraMEASURE_ULTRASONIC_ISR
#define ultraMEASURE_ULTRASONIC_TASK
#endif

void ultrasonic_Init(void);

void ultrasonic_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod);

void ultrasonic_GetinCm(void);

#endif /* DIFF_ROBOT_ULTRASONIC_H_ */
