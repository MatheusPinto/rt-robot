/*
 * gyro.c
 *
 *  Created on: 2 de jul de 2019
 *      Author: Matheus_Pinto
 */

#include <delayer/delayer.h>
#include <diff_robot/imu.h>
#include <emb_util/emb_util.h>
#include <math.h>
#include <diff_robot/measure.h>

#define BUFFER_SIZE 2

//Endereco I2C do MPU6050
const int _mpuSlaveAddress=0x68;

int16_t _gyXOffset = 0, _gyYOffset = 0, _gyZOffset = 0;
static TickType_t xMpuTaskPeriod;
static float _sampleRate;

static void MpuTask(void *pvParameters)
{
	float actual_angle = 0;
	int16_t samples[6];

	TickType_t xLastWakeTime, xActualWakeTime;

	communic_WaitInitSignal();
	xActualWakeTime = xLastWakeTime = communic_GetInitalTime(); // Get the first task activation
	for(;;)
    {
		if(!imu_GetRawSamples(samples))
		{
			fault_Throw();
		}
		actual_angle = ((float)samples[5]* _sampleRate)/65.5;

		if((actual_angle > 0) && (actual_angle < 0.001))
		{
			actual_angle = 0;
		}
		else
		{
			if((actual_angle < 0) && (actual_angle > -0.05))
			{
				actual_angle = 0;
			}
		}

		actual_angle *= (emb_PI/180);

		communic_SendDelthaAngle(&actual_angle);

		measure_CodeEndCatch();
		/* Delays until next activation. */
        vTaskDelayUntil(&xActualWakeTime, xMpuTaskPeriod);
        measure_CodeBeginCatch();
        fault_VerifyDeadline(xLastWakeTime, xActualWakeTime, xMpuTaskPeriod);
        xLastWakeTime = xActualWakeTime;
    }

}



int imu_Init(void)
{
	int16_t gyX, gyY, gyZ;
	uint8_t data[6];

	//Activate the MPU-6050
	data[0] = 0x00;
	imu_WriteRegister(IMU_PWR_MGMT_1, data, 1);

	//Configure the accelerometer (+/-8g)
	data[0] = 0x10;
	imu_WriteRegister(IMU_ACCEL_CONFIG, data, 1);

	//Configure the accelerometer (+/-8g)
	data[0] = 0x10;
	imu_WriteRegister(IMU_ACCEL_CONFIG, data, 1);

	//Configure the gyro (500dps full scale)
	data[0] = 0x08;
	imu_WriteRegister(IMU_GYRO_CONFIG, data, 1);

	for(int i = 0; i < 3000; i++)
	{                  //Read the raw gyro data from the MPU-6050 for 1000 times
		imu_GetRawGyro(&gyX, &gyY, &gyZ);
		_gyXOffset += gyX;                                              //Add the gyro x offset to the gyro_x_cal variable
		_gyYOffset += gyY;                                              //Add the gyro y offset to the gyro_y_cal variable
		_gyZOffset += gyZ;                                              //Add the gyro z offset to the gyro_z_cal variable
		delayer_Waitus(3000);                                           //Delay 3ms to have 250Hz for-loop
	}

	_gyXOffset /= 3000;                                              //Add the gyro x offset to the gyro_x_cal variable
	_gyYOffset /= 3000;                                              //Add the gyro y offset to the gyro_y_cal variable
	_gyZOffset /= 3000;                                              //Add the gyro z offset to the gyro_z_cal variable

	return 1;
}

int imu_GetRawGyro(int16_t *gyX, int16_t *gyY, int16_t *gyZ)
{
	uint8_t status;
	status_t result;
	const uint8_t txBuff = IMU_GYRO_XOUT_H;
	uint8_t rxBuff[6];

	/* Send start and slave address.*/
	I2C_MasterStart(I2C1, _mpuSlaveAddress, kI2C_Write);
	/* Wait address sent out.*/
	while(!((status = I2C_MasterGetStatusFlags(I2C1)) & kI2C_IntPendingFlag))
	{
	}
	if(status & kI2C_ReceiveNakFlag)
	{
		return 0;
	}

	result = I2C_MasterWriteBlocking(I2C1, &txBuff, 1, kI2C_TransferRepeatedStartFlag);
	if(result != kStatus_Success)
	{
		return 0;
	}

	I2C_MasterStart(I2C1, _mpuSlaveAddress, kI2C_Read);
	/* Wait address sent out.*/
	while(!((status = I2C_MasterGetStatusFlags(I2C1)) & kI2C_IntPendingFlag))
	{
	}

	I2C_MasterReadBlocking(I2C1, rxBuff, 6, kI2C_TransferDefaultFlag);
	if(result != kStatus_Success)
	{
		return 0;
	}

	*gyX = (rxBuff[0] << 8) | rxBuff[1];  //0x3B (MPU_GYRO_XOUT_H) & 0x3C (MPU_GYRO_XOUT_L)
	*gyY = (rxBuff[2] << 8) | rxBuff[3];  //0x3D (MPU_GYRO_YOUT_H) & 0x3E (MPU_GYRO_YOUT_L)
	*gyZ = (rxBuff[4] << 8) | rxBuff[5];  //0x3F (MPU_GYRO_ZOUT_H) & 0x3G (MPU_GYRO_ZOUT_L)

	return 1;

}

int imu_GetRawGyroNoOffset(int16_t *gyX, int16_t *gyY, int16_t *gyZ)
{
	uint8_t status;
	status_t result;
	const uint8_t txBuff = IMU_GYRO_XOUT_H;
	uint8_t rxBuff[6];

	/* Send start and slave address.*/
	I2C_MasterStart(I2C1, _mpuSlaveAddress, kI2C_Write);
	/* Wait address sent out.*/
	while(!((status = I2C_MasterGetStatusFlags(I2C1)) & kI2C_IntPendingFlag))
	{
	}
	if(status & kI2C_ReceiveNakFlag)
	{
		return 0;
	}

	result = I2C_MasterWriteBlocking(I2C1, &txBuff, 1, kI2C_TransferRepeatedStartFlag);
	if(result != kStatus_Success)
	{
		return 0;
	}

	I2C_MasterStart(I2C1, _mpuSlaveAddress, kI2C_Read);
	/* Wait address sent out.*/
	while(!((status = I2C_MasterGetStatusFlags(I2C1)) & kI2C_IntPendingFlag))
	{
	}

	I2C_MasterReadBlocking(I2C1, rxBuff, 6, kI2C_TransferDefaultFlag);
	if(result != kStatus_Success)
	{
		return 0;
	}

	*gyX = ((rxBuff[0] << 8) | rxBuff[1]) + _gyXOffset;  //0x3B (MPU_GYRO_XOUT_H) & 0x3C (MPU_GYRO_XOUT_L)
	*gyY = ((rxBuff[2] << 8) | rxBuff[3]) + _gyYOffset;  //0x3D (MPU_GYRO_YOUT_H) & 0x3E (MPU_GYRO_YOUT_L)
	*gyZ = ((rxBuff[4] << 8) | rxBuff[5]) + _gyZOffset;  //0x3F (MPU_GYRO_ZOUT_H) & 0x3G (MPU_GYRO_ZOUT_L)

	return 1;

}

int imu_GetRawSamples(int16_t samples[])
{
	uint8_t rxBuff[14];

	if(!imu_ReadRegister(IMU_ACCEL_XOUT_H, rxBuff, 14))
	{
		return 0;
	}

	samples[0] = (((int16_t)rxBuff[0])   << 8) | rxBuff[1];  				//0x3B (IMU_ACCEL_XOUT_H) & 0x3C (IMU_ACCEL_XOUT_L)
	samples[1] = (rxBuff[2]   << 8) | rxBuff[3];  				//0x3D (IMU_ACCEL_YOUT_H) & 0x3E (IMU_ACCEL_YOUT_L)
	samples[2] = (rxBuff[4]   << 8) | rxBuff[5];  				//0x3F (IMU_ACCEL_ZOUT_H) & 0x40 (IMU_ACCEL_ZOUT_L)
	samples[3] = ((rxBuff[8]  << 8) | rxBuff[9])  - _gyXOffset; //0x4B (IMU_GYRO_XOUT_H) & 0x4C (IMU_GYRO_XOUT_L)
	samples[4] = ((rxBuff[10] << 8) | rxBuff[11]) - _gyYOffset; //0x4D (IMU_GYRO_YOUT_H) & 0x4E (IMU_GYRO_YOUT_L)
	samples[5] = ((rxBuff[12] << 8) | rxBuff[13]) - _gyZOffset; //0x4F (IMU_GYRO_ZOUT_H) & 0x50 (IMU_GYRO_ZOUT_L)

	return 1;

}

int imu_GetGyro(float *gyX, float *gyY, float *gyZ)
{
	int16_t rawGyX, rawGyY, rawGyZ;

	if(!imu_GetRawGyroNoOffset(&rawGyX, &rawGyY, &rawGyZ))
	{
		return 0;
	}

	// divide raw value by degrees/s step: 65.5.
	*gyX = ((float)rawGyX)/65.5;
	*gyY = ((float)rawGyY)/65.5;
	*gyZ = ((float)rawGyZ)/65.5;

	return 1;
}

void imu_TaskCreate(UBaseType_t uxPriority, TickType_t xPeriod)
{
	xMpuTaskPeriod = xPeriod;

	_sampleRate = (float)(xMpuTaskPeriod/portTICK_PERIOD_MS)/1000;

    if (xTaskCreate(MpuTask, "mpu", configMINIMAL_STACK_SIZE, NULL, uxPriority, NULL) != pdPASS)
    {
           while(1);
    }
}

int imu_ReadRegister(uint8_t addr, uint8_t *data, uint8_t dataSize)
{
	uint8_t status;
	status_t result;

	/* Send start and slave address.*/
	I2C_MasterStart(I2C1, _mpuSlaveAddress, kI2C_Write);
	/* Wait address sent out.*/
	while(!((status = I2C_MasterGetStatusFlags(I2C1)) & kI2C_IntPendingFlag))
	{
	}
	if(status & kI2C_ReceiveNakFlag)
	{
		return 0;
	}

	result = I2C_MasterWriteBlocking(I2C1, &addr, 1, kI2C_TransferRepeatedStartFlag);
	if(result != kStatus_Success)
	{
		return 0;
	}

	I2C_MasterStart(I2C1, _mpuSlaveAddress, kI2C_Read);
	/* Wait address sent out.*/
	while(!((status = I2C_MasterGetStatusFlags(I2C1)) & kI2C_IntPendingFlag))
	{
	}

	I2C_MasterReadBlocking(I2C1, data, dataSize, kI2C_TransferDefaultFlag);
	if(result != kStatus_Success)
	{
		return 0;
	}

	return 1;
}

int imu_WriteRegister(uint8_t addr, uint8_t *data, uint8_t dataSize)
{
	uint8_t status;
	status_t result;

	/* Send start and slave address.*/
	I2C_MasterStart(I2C1, _mpuSlaveAddress, kI2C_Write);
	/* Wait address sent out.*/
	while(!((status = I2C_MasterGetStatusFlags(I2C1)) & kI2C_IntPendingFlag))
	{
	}
	if(status & kI2C_ReceiveNakFlag)
	{
		return 0;
	}

	/* Send register address */
	result = I2C_MasterWriteBlocking(I2C1, &addr, 1, kI2C_TransferNoStopFlag);
	if(result != kStatus_Success)
	{
		return 0;
	}

	/* Send register data */
	result = I2C_MasterWriteBlocking(I2C1, data, dataSize, kI2C_TransferDefaultFlag);
	if(result != kStatus_Success)
	{
		return 0;
	}
	return 1;
}
