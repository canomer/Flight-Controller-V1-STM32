/*
 * bmp180.c
 *
 *  Created on: Feb 17, 2022
 *      Author: Kazim YEL
 * 
 *  
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>. 
 * 
 */

#include "bmp180.h"

void BMP180_Init(void)
{
	if(HAL_I2C_IsDeviceReady(&hi2c1, BMP180_Write_Address, 1, 100) != HAL_OK);

	Get_Calibration_Value();
}
void Get_Calibration_Value(void)
{
	uint8_t Callib_Data[22] = {0};
	uint16_t Callib_Start = 0xAA;
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_Read_Address, Callib_Start, 1, Callib_Data, 22, HAL_MAX_DELAY);

	AC1 = ((Callib_Data[0] << 8) | Callib_Data[1]);
	AC2 = ((Callib_Data[2] << 8) | Callib_Data[3]);
	AC3 = ((Callib_Data[4] << 8) | Callib_Data[5]);
	AC4 = ((Callib_Data[6] << 8) | Callib_Data[7]);
	AC5 = ((Callib_Data[8] << 8) | Callib_Data[9]);
	AC6 = ((Callib_Data[10] << 8) | Callib_Data[11]);
	B1 = ((Callib_Data[12] << 8) | Callib_Data[13]);
	B2 = ((Callib_Data[14] << 8) | Callib_Data[15]);
	MB = ((Callib_Data[16] << 8) | Callib_Data[17]);
	MC = ((Callib_Data[18] << 8) | Callib_Data[19]);
	MD = ((Callib_Data[20] << 8) | Callib_Data[21]);
}

uint16_t Get_UncomTemperature(void)
{
	uint8_t datatowrite = 0x2E;
	uint8_t Temp_RAW[2] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_Write_Address, 0xF4, 1, &datatowrite, 1, 1000);
	HAL_Delay(5);	//4.5ms
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_Read_Address, 0xF6, 1, Temp_RAW, 2, 1000);
	return ((Temp_RAW[0] << 8) + Temp_RAW[1]);
}

float BMP180_GetTemperature(void)
{
	UT = Get_UncomTemperature();
	X1 = ((UT - AC6) * (AC5 / (pow(2, 15))));
	X2 = MC * (pow(2, 11)) / (X1 + MD);
	B5 = X1 + X2;
	Temp = (B5 + 8) / (pow(2, 4));
	return Temp / 10.00;
}

uint32_t Get_UncomPressure(int16_t oss)
{
	uint8_t datatowrite = 0x34 + (oss << 6);
	uint8_t Press_RAW[3] = {0};
	HAL_I2C_Mem_Write(BMP180_I2C, BMP180_Write_Address, 0xF4, 1, &datatowrite, 1, 1000);
	switch(oss)
	{
	case 0:
		HAL_Delay(5);	//4.5
		break;
	case 1:
		HAL_Delay(8);	//7.5
		break;
	case 2:
		HAL_Delay(14);	//13.5
		break;
	case 3:
		HAL_Delay(26);	//25.5
		break;
	default:
		break;
	}
	HAL_I2C_Mem_Read(BMP180_I2C, BMP180_Read_Address, 0xF6, 1, Press_RAW, 3, 1000);
	return (((Press_RAW[0] << 16) + (Press_RAW[1] << 8) + Press_RAW[2]) >> (8-oss));
}

float BMP180_GetPressure(int16_t oss)
{
	UP = Get_UncomPressure(oss);
	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 / (pow(2, 12)))) / pow(2, 11);
	X2 = AC2 * B6 / (pow(2, 11));
	X3 = X1 + X2;
	B3 = ((((long)AC1 * 4 + X3) << oss) + 2) / 4;
	X1 = AC3 * B6 / (pow(2, 13));
	X2 = (B1 * (B6* B6 / (pow(2,12)))) / (pow(2, 16));
	X3 = ((X1 + X2) + 2) / (pow(2,2));
	B4 = AC4 * (unsigned long)(X3 + 32768) / (pow(2, 15));
	B7 = ((unsigned long)UP - B3) * (50000 >> oss);
	if(B7 < 0x80000000) Press = (B7 * 2) / B4;
	else Press = (B7 / B4) * 2;
	X1 = (Press / (pow(2, 8))) * (Press / (pow(2, 8)));
	X1 = (X1 * 3038) / (pow(2, 16));
	X2 = (-7357 * Press) / (pow(2, 16));
	Press = Press + (X1 + X2 +3791) / (pow(2, 4));
	return Press;
}

float BMPO180_GetAltitude(int16_t oss)
{
	BMP180_GetPressure(oss);
	return 44330 * (1 - (pow((Press / (float)atmPressure) , (1 / 5.255))));
}

