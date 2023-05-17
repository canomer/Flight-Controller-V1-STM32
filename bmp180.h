/*
 * bmp180.h
 *
 *  Created on: Feb 17, 2022
 *      Author: Ömer Can VURAL
 * 
 * 
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>. 
 * 
 * 
 */

#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "math.h"

#define BMP180_Write_Address 0xEE
#define BMP180_Read_Address 0xEF
#define atmPressure 101325	//Pa

extern I2C_HandleTypeDef hi2c1;
#define BMP180_I2C &hi2c1

short AC1, AC2, AC3;
unsigned short AC4, AC5, AC6;
short B1, B2, MB, MC, MD;
long UT;
short oss;
long UP, X1, X2, X3, B3, B5;
unsigned long B4;
long B6;
unsigned long B7;
long Temp, Press;


void BMP180_Init(void);
float BMP180_GetTemperature(void);
float BMP180_GetPressure(int16_t oss);
float BMPO180_GetAltitude(int16_t oss);

#endif /* INC_BMP180_H_ */
