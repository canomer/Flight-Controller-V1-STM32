/*
 * mpu6050.c
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
 * 
 */

#include "mpu6050.h"


void MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_Struct* DataStruct, MPU6050_Accelerometer AccSens, MPU6050_Gyroscope GyroSens)
{
	uint8_t check, data;
	/* Is device ready*/
	HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 3, HAL_MAX_DELAY);

	/* Check who I am */
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);
	if(check == 104)
	{
		/* Wake up */
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);

        data = (uint8_t)AccSens<<3;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);

        data = (uint8_t)GyroSens<<3;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, HAL_MAX_DELAY);
        //Low pass filter cfg
        data = 0x03;
        HAL_I2C_Mem_Write (hi2c, MPU6050_ADDR, LPF_REG, 1, &data, 1, HAL_MAX_DELAY);
	}

	switch (AccSens) {
		case MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
			break;
		case MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
			break;
		case MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
			break;
		case MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
		default:
			break;
	}

	switch (GyroSens) {
		case MPU6050_Gyroscope_250s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
			break;
		case MPU6050_Gyroscope_500s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
			break;
		case MPU6050_Gyroscope_1000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
			break;
		case MPU6050_Gyroscope_2000s:
			DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
		default:
			break;
	}
}

void MPU6050_GetCalibration(I2C_HandleTypeDef *hi2c)
{
	  for(i=0; i<2000; i++)
	  {
    	  prevtime_cal = time_cal;
    	  time_cal = HAL_GetTick();
    	  elapsedtime_cal = (time_cal - prevtime_cal) * 1000;
		  //gyro address
		  buffer_gyro[0] = 0x43;
		  HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buffer_gyro, 1, HAL_MAX_DELAY);
		  HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, buffer_gyro, 6, HAL_MAX_DELAY);
		  //gyro raw values
		  gyro_raw[0] = (buffer_gyro[0] << 8 | buffer_gyro[1]);	//x
		  gyro_raw[1] = (buffer_gyro[2] << 8 | buffer_gyro[3]);	//y
		  gyro_raw[2] = (buffer_gyro[4] << 8 | buffer_gyro[5]);	//z

		  gyro_cal[0] += gyro_raw[0];
		  gyro_cal[1] += gyro_raw[1];
		  gyro_cal[2] += gyro_raw[2];

		  HAL_Delay(3);
	  }

	  gyro_cal[0] /=2000;
	  gyro_cal[1] /=2000;
	  gyro_cal[2] /=2000;
}

void MPU6050_GetAngleComplementary(I2C_HandleTypeDef *hi2c, MPU6050_Struct* DataStruct)
{
	  prevtime1 = time1;
	  time1 = HAL_GetTick();
	  elapsedtime1 = (time1- prevtime1) * 1000;
	  timediv = elapsedtime1 / 1000000;

	  //accelerometer address
	  buffer_acc[0] = 0x3B;
	  HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buffer_acc, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, buffer_acc, 6, HAL_MAX_DELAY);
	  //accelerometer raw values
	  acc_raw[0] = (buffer_acc[0] << 8 | buffer_acc[1]);	//x
	  acc_raw[1] = (buffer_acc[2] << 8 | buffer_acc[3]);	//y
	  acc_raw[2] = (buffer_acc[4] << 8 | buffer_acc[5]);	//z

	  //temperature address
	  buffer_temp[0] = 0x41;
	  HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buffer_temp, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, buffer_temp, 2, HAL_MAX_DELAY);
	  //temperature raw values
	  raw_temp = (buffer_temp[0] << 8 | buffer_temp[1]);
	  temp = (raw_temp / 340.0) + 36.53;

	  //gyro address
	  buffer_gyro[0] = 0x43;
	  HAL_I2C_Master_Transmit(hi2c, MPU6050_ADDR, buffer_gyro, 1, HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(hi2c, MPU6050_ADDR, buffer_gyro, 6, HAL_MAX_DELAY);
	  //gyro raw values
	  gyro_raw[0] = (buffer_gyro[0] << 8 | buffer_gyro[1]);	//x
	  gyro_raw[1] = (buffer_gyro[2] << 8 | buffer_gyro[3]);	//y
	  gyro_raw[2] = (buffer_gyro[4] << 8 | buffer_gyro[5]);	//z
	  //
	  gyro_raw[0] -= gyro_cal[0];
	  gyro_raw[1] -= gyro_cal[1];
	  gyro_raw[2] -= gyro_cal[2];

	  //complementary filter
	  gyro_pitch_angle += gyro_raw[0] * DataStruct->Gyro_Mult * timediv ;
	  gyro_roll_angle += gyro_raw[1] * DataStruct->Gyro_Mult * timediv ;

	  //1 / (pi/180) = 57.296 = RAD_TO_DEG
	  gyro_pitch_angle += gyro_roll_angle * sin(gyro_raw[2] * DataStruct->Gyro_Mult * timediv / RAD_TO_DEG);
	  gyro_roll_angle -= gyro_pitch_angle * sin(gyro_raw[2] * DataStruct->Gyro_Mult * timediv / RAD_TO_DEG);

	  acc_total_vector = sqrt((acc_raw[0]*acc_raw[0])+(acc_raw[1]*acc_raw[1])+(acc_raw[2]*acc_raw[2]));

	  acc_pitch_angle = asin((float)acc_raw[1]/acc_total_vector)* RAD_TO_DEG;
	  acc_roll_angle = asin((float)acc_raw[0]/acc_total_vector)* -RAD_TO_DEG;

	  acc_pitch_angle -= 0.00;//0.08;
	  acc_roll_angle -= 0.00;//-1.17;

	  if(set_gyro)
	  {
		  //low pass-high pass filter
		  pitch_angle = gyro_pitch_angle * 0.9996 + acc_pitch_angle * 0.0004;
		  roll_angle = gyro_roll_angle * 0.9996 + acc_roll_angle * 0.0004;
	  }
	  else
	  {
		  pitch_angle = acc_pitch_angle;
		  set_gyro = true;
	  }

	  while((HAL_GetTick() - prevtime)*1000 < 4000);
	  prevtime = HAL_GetTick();
}
