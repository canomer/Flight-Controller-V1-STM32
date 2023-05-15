/*
 * mpu6050.h
 *
 *  Created on: Dec 30, 2021
 *      Author: Kazim YEL
 * 
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>


#define MPU6050_ADDR (0x68<<1)
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define LPF_REG 0x1A

#define RAD_TO_DEG 57.295779513082320876798154814105


/* Gyro sensitivities */
#define MPU6050_GYRO_SENS_250		((float) 131)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Accelerometer sensitivities */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)

typedef enum {
	MPU6050_Accelerometer_2G = 0x00, /* Range is +- 2G */
	MPU6050_Accelerometer_4G = 0x01, /* Range is +- 4G */
	MPU6050_Accelerometer_8G = 0x02, /* Range is +- 8G */
	MPU6050_Accelerometer_16G = 0x03 /* Range is +- 16G */
}MPU6050_Accelerometer;


typedef enum {
	MPU6050_Gyroscope_250s = 0x00,  /* Range is +- 250 degrees/s */
	MPU6050_Gyroscope_500s = 0x01,  /* Range is +- 500 degrees/s */
	MPU6050_Gyroscope_1000s = 0x02, /* Range is +- 1000 degrees/s */
	MPU6050_Gyroscope_2000s = 0x03  /* Range is +- 2000 degrees/s */
}MPU6050_Gyroscope;

//MPU6050 Structure
typedef struct {
	uint8_t Address;         /*!< I2C address of device */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s" */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g" */
}MPU6050_Struct;


uint8_t data;
uint8_t buffer_temp[2], buffer_acc[6], buffer_gyro[6];	//raw gyro ve acc değerleri
int8_t gyro_raw[3], acc_raw[3];				//anlamlandırılmış raw veriler
float gyro_cal[3];							//gyro calibration list
int16_t acc_total_vector;
float gyro_pitch_angle, gyro_roll_angle;
float acc_pitch_angle, acc_roll_angle;
float pitch_angle, roll_angle;
int16_t raw_temp;
float temp;
int i;
float prevtime, prevtime1, time1, elapsedtime1, prevtime_cal, time_cal, elapsedtime_cal;
HAL_StatusTypeDef set_gyro;
float timediv;

void MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_Struct* DataStruct, MPU6050_Accelerometer AccSens, MPU6050_Gyroscope GyroSens);
void MPU6050_GetCalibration(I2C_HandleTypeDef *hi2c);
void MPU6050_GetAngleComplementary(I2C_HandleTypeDef *hi2c, MPU6050_Struct* DataStruct);


#endif /* INC_MPU6050_H_ */
