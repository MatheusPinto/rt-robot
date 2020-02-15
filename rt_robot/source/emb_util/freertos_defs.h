/***************************************************************************************
 * Module      : freertos_defs.h
 * Revision    : 1.0
 * Date        : 19/08/2015
 * Description : It contains macros and typedefs used by FreeRTOS to be used in "baremetal"
 * 				 projects.
 * Comments    : None.
 * Author(s)   : Matheus Leitzke Pinto
 ***************************************************************************************/

/* Comment the include below, if NOT using FreeRTOS*/
#include "FreeRTOS.h"
#ifndef INC_FREERTOS_H
#ifndef FREERTOS_BAREMETAL_DEFS_H_
#define FREERTOS_BAREMETAL_DEFS_H_
#include "stdint.h"

#define pdTRUE 	0U
#define pdFALSE 1U
#define pdPASS 	0U
#define pdFAIL 	1U

#define BaseType_t uint32_t
#define TickType_t BaseType_t

#endif /*FREERTOS_BAREMETAL_DEFS */
#endif /*INC_FREERTOS_H*/

/***************************************************************************************
 * END: Module - freertos_defs.h
 ***************************************************************************************/
