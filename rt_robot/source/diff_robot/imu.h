/*
 * gyro.h
 *
 *  Created on: 2 de jul de 2019
 *      Author: Matheus_Pinto
 */

#ifndef DIFF_ROBOT_IMU_H_
#define DIFF_ROBOT_IMU_H_

#include "fsl_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "communic.h"
#include "fault.h"

#define IMU_GYRO_CONFIG 0x1B
#define IMU_ACCEL_CONFIG 0x1C
#define IMU_ACCEL_XOUT_H 0x3B
#define IMU_GYRO_XOUT_H 0x43
#define IMU_PWR_MGMT_1 0x6B

int imu_Init(void);

int imu_GetRawGyroNoOffset(int16_t *gyX, int16_t *gyY, int16_t *gyZ);

int imu_GetRawGyro(int16_t *gyX, int16_t *gyY, int16_t *gyZ);

int imu_GetGyro(float *gyX, float *gyY, float *gyZ);

int imu_GetRawSamples(int16_t samples[]);

int imu_ReadRegister(uint8_t addr, uint8_t *data, uint8_t dataSize);

int imu_WriteRegister(uint8_t addr, uint8_t *data, uint8_t dataSize);

void imu_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod);


#endif /* DIFF_ROBOT_IMU_H_ */
