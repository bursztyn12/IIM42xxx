/*
 * iim42652.cpp
 *
 *  Created on: 20 mar 2023
 *      Author: bursztyn
 */

#include <stdio.h>

#include "iim42652.hpp"
#include "main.h"
#include "stm32g431xx.h"

extern SPI_HandleTypeDef hspi3;

IIm42652::IIm42652(): accelSensitivity_(IIM42652::ACCEL_SEN_2G){}

bool IIm42652::config(IIM42652::ODR accelODR, IIM42652::AccelFS accelFs,
					IIM42652::ODR gyroODR, IIM42652::GyroFS gyroFs,
					IIM42652::SampleBW accelBW, IIM42652::SampleBW gyroBW,
					uint8_t rtcEnable)
{
	if(!checkWhoAmI())
	{
		return false;
	}

	reset();

	configAccel(accelODR, accelFs, accelBW);
	configGyro(gyroODR, gyroFs, gyroBW);

	if(rtcEnable)
	{
		enableCLKINMode();
	}

	return true;
}

/** default config
 *
*/
bool IIm42652::config()
{
	if(!checkWhoAmI())
	{
		return false;
	}

	reset();

	getAccelMode();

	configAccel(IIM42652::ODR_2_kHZ, IIM42652::ACCEL_FS_2G,
				IIM42652::ODR_DIV_2);
	configGyro(IIM42652::ODR_2_kHZ, IIM42652::GYRO_FS_250_DPS,
				IIM42652::ODR_DIV_2);
	enableCLKINMode();

	return true;
}

void IIm42652::reset()
{
	uint8_t data[2] = {IIM42652::DEVICE_CONFIG, IIM42652::SOFT_RESET_CONFIG};

	setBank(IIM42652::BANK0);
	write(data, 2);
	HAL_Delay(1);
}

void IIm42652::configInt1(IIM42652::IntConfig &config)
{
	uint8_t val = 0;
	uint8_t dataOption[2] = {IIM42652::INT_CONFIG, 0};

	setBank(IIM42652::BANK0);
	read(IIM42652::INT_CONFIG, &val, 1);
	clearMask(&val, IIM42652::LATCHED | IIM42652::PUSH_PULL | IIM42652::ACTIVE_HIGH);
	setMask(&val, config.mode | config.drive | config.polarity);
	dataOption[1] = val;
	write(dataOption, 2);
}

void IIm42652::configDRDY(IIM42652::DrdyIntConfig0 clrOption)
{
	uint8_t val = 0;
	uint8_t data[2] = {IIM42652::INT_CONFIG0, 0};

	setBank(IIM42652::BANK0);
	read(IIM42652::INT_CONFIG0, &val, 1);
	clearMask(&val, IIM42652::DRDY_INT_CLR_MASK);
	setMask(&val, clrOption);
	data[1] = val;
	write(data, 2);

	data[0] = IIM42652::INT_SOURCE0;
	data[1] = IIM42652::DRDY_INT1_EN;
	write(data, 2);
}

void IIm42652::fetchAccelSample()
{
	uint8_t data[6] = {0, 0, 0, 0, 0, 0};

	setBank(IIM42652::BANK0);

	read(IIM42652::ACCEL_DATA_X1, data, 6);

	rawAccelData_[0] = ((data[1] << 8 | data[0]));
	rawAccelData_[1] = ((data[3] << 8 | data[2]));
	rawAccelData_[2] = ((data[5] << 8 | data[4]));
}

void IIm42652::convertSamplesToG(float *samples)
{
	samples[0] = rawAccelData_[0] / (float)accelSensitivity_;
	samples[1] = rawAccelData_[1] / (float)accelSensitivity_;
	samples[2] = rawAccelData_[2] / (float)accelSensitivity_;
}

void IIm42652::accelLowNoiseMode()
{
	uint8_t val = 0;
	uint8_t data[2] = {IIM42652::PWR_MGMT0, 0};

	setBank(IIM42652::BANK0);

	read(IIM42652::PWR_MGMT0, &val, 1);

	if(checkMask(val, IIM42652::ACCEL_MODE_CLEAR_MSK, IIM42652::MODE_OFF))
	{
		clearMask(&val, IIM42652::ACCEL_MODE_CLEAR_MSK);
		setMask(&val, IIM42652::ACCEL_MODE_LOW_NOISE);
		data[1] = val;

		write(data, 2);
		//wait for 200 us
		HAL_Delay(1);
	}
}

void IIm42652::accelTurnOff()
{
	uint8_t val;
	uint8_t data[2] = {IIM42652::PWR_MGMT0, 0};

	setBank(IIM42652::BANK0);
	read(IIM42652::PWR_MGMT0, &val, 1);

	clearMask(&val, IIM42652::ACCEL_MODE_CLEAR_MSK);
	data[1] = val;

	write(data, 2);
}

uint8_t IIm42652::getAccelMode()
{
	uint8_t data = 0;
	read(IIM42652::PWR_MGMT0, &data, 1);

	return data;
}

void IIm42652::configAccel(IIM42652::ODR odr, IIM42652::AccelFS fs,
						IIM42652::SampleBW bw)
{
	//ODR & FS config
	uint8_t val = 0;
	uint8_t data[2] = {IIM42652::ACCEL_CONFIG0, 0};

	setBank(IIM42652::BANK0);
	read(IIM42652::ACCEL_CONFIG0, &val, 1);
	clearMask(&val, IIM42652::ODR_CLEAR_MSK | IIM42652::ACCEL_FS_CLEAR_MSK);
	setMask(&val, odr | fs);
	data[1] = val;
	write(data, 2);

	//Filter BW config
	val = 0;
	data[0] = IIM42652::GYRO_ACCEL_CONFIG0;
	data[1] = 0;

	read(IIM42652::GYRO_ACCEL_CONFIG0, &val, 1);
	clearMask(&val, IIM42652::BANDWIDTH_CLEAR_MSK << 4);
	setMask(&val, bw <<  4);
	data[1] = val;
	write(data, 2);
}

void IIm42652::configGyro(IIM42652::ODR odr, IIM42652::GyroFS fs,
						IIM42652::SampleBW bw)
{
	//ODR & FS config
	uint8_t val = 0;
	uint8_t data[2] = {IIM42652::GYRO_CONFIG0, 0};

	setBank(IIM42652::BANK0);
	read(IIM42652::GYRO_CONFIG0, &val, 1);
	clearMask(&val, IIM42652::ODR_CLEAR_MSK | IIM42652::GYRO_FS_CLEAR_MSK);
	setMask(&val, odr | fs);
	data[1] = val;
	write(data, 2);

	//Filter BW config
	val = 0;
	data[0] = IIM42652::GYRO_ACCEL_CONFIG0;
	data[1] = 0;

	read(IIM42652::GYRO_ACCEL_CONFIG0, &val, 1);
	clearMask(&val, IIM42652::BANDWIDTH_CLEAR_MSK);
	setMask(&val, bw);
	data[1] = val;
	write(data, 2);
}

void IIm42652::configFIFO(IIM42652::FifoConfig1 accelEn, IIM42652::FifoConfig1 gyroEn,
						  IIM42652::FifoIntConfig0 clrOption)
{
	uint8_t val = 0;
	uint8_t data[2] = {IIM42652::FIFO_CONFIG1, 0};

	setBank(IIM42652::BANK0);
	setMask(&val, accelEn | gyroEn);
	data[1] = val;
	write(data, 2);

	read(IIM42652::INT_CONFIG0, &val, 1);
	clearMask(&val, IIM42652::FIFO_FULL_INT_CLR_MASK);
	setMask(&val, clrOption);
	data[0] = IIM42652::INT_CONFIG0;
	data[1] = val;
	write(data, 2);

	data[0] = IIM42652::INT_SOURCE0;
	data[1] = IIM42652::FIFO_FULL_INT1_EN;
	write(data, 2);

	data[0] = IIM42652::FIFO_CONFIG;
	data[1] = IIM42652::STREAM_TO_FIFO;
	write(data, 2);
}

void IIm42652::enableCLKINMode()
{
	uint8_t val = 0;
	uint8_t data[2] = {IIM42652::INTF_CONFIG5, 0};

	//select CLKIN as a pin 9 function
	setBank(IIM42652::BANK1);
	read(IIM42652::INTF_CONFIG5, &val, 1);
	if(!checkMask(val, IIM42652::PIN_9_CLEAR_MSK, IIM42652::CLKIN))
	{
		clearMask(&val, IIM42652::PIN_9_CLEAR_MSK);
		setMask(&val, IIM42652::CLKIN);
		data[1] = val;

		write(data, 2);
	}

	//set RTC clock input is required
	setBank(IIM42652::BANK0);
	data[0] = IIM42652::INTF_CONFIG1;
	read(IIM42652::INTF_CONFIG1, &val, 1);
	clearMask(&val, IIM42652::RTC_MODE_EN);
	setMask(&val, IIM42652::RTC_MODE_EN);
	data[1] = val;

	write(data, 2);
}

bool IIm42652::checkWhoAmI()
{
	uint8_t data = 0;

	setBank(IIM42652::BANK0);
	read(IIM42652::WHO_AM_I, &data, 1);

	if(data != whoAmI_)
	{
		printf("ERROR: who_am_i not valid \r\n");
		return false;
	}

	printf("INFO: who_am_i valid");
	return true;
}

void IIm42652::setBank(uint8_t bank)
{
	uint8_t data[2] = {IIM42652::REG_BANK_SEL, bank};
	write(data, 2);
}

void IIm42652::read(uint8_t reg, uint8_t *data, uint16_t size)
{
	//the first bit of the first byte contains the read/write bit, the following 7 bits contins register address
	setMask(&reg, IIM42652::READ_WRITE_BIT);
	HAL_GPIO_WritePin(IIM_CS_GPIO_Port, IIM_CS_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi3, &reg, 1, 0xFFFF) != HAL_OK)
	{
		printf("ERROR [read]: HAL_SPI_Transmit\r\n");
		return;
	}

	if(HAL_SPI_Receive(&hspi3, data, size, 0xFFFF) != HAL_OK)
	{
		printf("ERROR [read]: HAL_SPI_Receive\r\n");
		return;
	}
	HAL_GPIO_WritePin(IIM_CS_GPIO_Port, IIM_CS_Pin, GPIO_PIN_SET);
}

void IIm42652::write(uint8_t *data, uint16_t size)
{
	HAL_GPIO_WritePin(IIM_CS_GPIO_Port, IIM_CS_Pin, GPIO_PIN_RESET);
	clearMask(data, IIM42652::READ_WRITE_BIT);
	HAL_SPI_Transmit(&hspi3, data, size, 0xFFFF);
	HAL_GPIO_WritePin(IIM_CS_GPIO_Port, IIM_CS_Pin, GPIO_PIN_SET);
}

