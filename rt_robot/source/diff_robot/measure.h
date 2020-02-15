/*
 * measure.h
 *
 *  Created on: 7 de ago de 2019
 *      Author: Matheus_Pinto
 */

#ifndef DIFF_ROBOT_MEASURE_H_
#define DIFF_ROBOT_MEASURE_H_
#include "fsl_gpio.h"
#include "pin_mux.h"

#define MEASURE_IS_SET
#define MEASURE_TASK_PERIOD 3

#ifdef MEASURE_IS_SET
#define	measure_CodeBeginCatch() GPIOD->PSOR = 0x00000010;
#define	measure_CodeEndCatch() GPIOD->PCOR = 0x00000010;
#else
#define	measure_CodeBeginCatch()
#define	measure_CodeEndCatch()
#endif

#endif /* DIFF_ROBOT_MEASURE_H_ */
