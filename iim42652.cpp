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

extern UART_HandleTypeDef hlpuart1;
extern I2C_HandleTypeDef hi2c1;
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
	reset();

	if(!checkWhoAmI())
	{
		return false;
	}

	configAccel(IIM42652::ODR_2_kHZ, IIM42652::ACCEL_FS_2G,
				IIM42652::ODR_DIV_2);
	configGyro(IIM42652::ODR_2_kHZ, IIM42652::GYRO_FS_250_DPS,
				IIM42652::ODR_DIV_2);
	enableCLKINMode();

	uint8_t val = 0;
	read(IIM42652::INTF_CONFIG0, &val, 1);

	if(val)
	{
		return true;
	}

	return true;
}

void IIm42652::reset()
{
	uint8_t data[2] = {IIM42652::DEVICE_CONFIG, IIM42652::SOFT_RESET_CONFIG};

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
	uint8_t data2[6] = {0, 0, 0, 0, 0, 0};

	setBank(IIM42652::BANK0);

	read(IIM42652::ACCEL_DATA_X1, data, 6);
	read(IIM42652::ACCEL_DATA_X1, data2, 1);
	read(IIM42652::ACCEL_DATA_X0, &data2[1], 1);


	rawAccelData_[0] = ((data[1] << 8 | data[0]));
	rawAccelData_[1] = ((data[3] << 8 | data[2]));
	rawAccelData_[2] = ((data[5] << 8 | data[4]));

	rawAccelData_2[0] = ((data[0] << 8 | data[1]));
	rawAccelData_2[1] = ((data[2] << 8 | data[3]));
	rawAccelData_2[2] = ((data[4] << 8 | data[5]));
}

void IIm42652::fetchAccelSampleDMA(uint8_t *buff, uint16_t size)
{
	setBank(IIM42652::BANK0);
	readDMA(IIM42652::ACCEL_DATA_X1, buff, size);
}

void IIm42652::convertSamplesToG2(float *samples)
{
	samples[0] = rawAccelData_2[0] / (float)accelSensitivity_;
	samples[1] = rawAccelData_2[1] / (float)accelSensitivity_;
	samples[2] = rawAccelData_2[2] / (float)accelSensitivity_;
}

void IIm42652::convertSamplesToG(float *samples)
{
	samples[0] = rawAccelData_[0] / (float)accelSensitivity_;
	samples[1] = rawAccelData_[1] / (float)accelSensitivity_;
	samples[2] = rawAccelData_[2] / (float)accelSensitivity_;
}

void IIm42652::convertSamplesToG(int16_t *rawData, float *samples)
{
	samples[0] = rawData[0] / (float)accelSensitivity_;
	samples[1] = rawData[1] / (float)accelSensitivity_;
	samples[2] = rawData[2] / (float)accelSensitivity_;
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
		for(int i = 0; i < 200; i++){}
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

void IIm42652::changeSlewRate()
{
	uint8_t data[2] = {IIM42652::DRIVE_CONFIG, 0x2D};
	write(data, 2);
}

void IIm42652::configAccel(IIM42652::ODR odr, IIM42652::AccelFS fs,
						IIM42652::SampleBW bw)
{
	//ODR & FS config
	uint8_t val = 0;
	uint8_t data[2] = {IIM42652::ACCEL_CONFIG0, 0};

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
							IIM42652::FifoConfig1 tempEn, IIM42652::FifoIntConfig0 clrOption)
{
	uint8_t val = 0;
	uint8_t data[2] = {IIM42652::FIFO_CONFIG1, 0};

	setBank(IIM42652::BANK0);
	setMask(&val, accelEn | gyroEn| tempEn);
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

uint16_t IIm42652::getFIFOCount()
{
	uint8_t count[2];

	read(IIM42652::FIFO_COUNTH, count, 2);

	return (count[0] << 8) | count[1];
}

void IIm42652::fetchFIFOSamples(uint16_t size)
{
	IIM42652::Packet1 packets[size / 8];
	int16_t rawData[3];
	float accelVals[3];
	char buff[50];
	uint16_t len;


	read(IIM42652::FIFO_DATA, (uint8_t*) packets, size);

	for(int i = 0; i < size / 8; i++)
	{
		rawData[0] = packets[0].xHigh << 8 | packets[0].xLow;
		rawData[1] = packets[0].yHigh << 8 | packets[0].yLow;
		rawData[2] = packets[0].zHigh << 8 | packets[0].zLow;

		//convertSamplesToG(rawData, accelVals);
	}
	convertSamplesToG(rawData, accelVals);
	len = snprintf(buff, 50, "X: %.2f, Y:%.2f, Z: %.2f \r\n", accelVals[0], accelVals[1], accelVals[2]);
	HAL_UART_Transmit(&hlpuart1, (uint8_t*)buff, len, 0xFFFF);
}

uint16_t IIm42652::getFIFOLostCount()
{
	uint8_t count[2];

	read(IIM42652::FIFO_LOST_PKT0, count, 2);

	return (count[1] << 8) | count[0];
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

	read(IIM42652::WHO_AM_I, &data, 1);
	if(data != whoAmI_)
	{
		printf("ERROR: who_am_i not valid \r\n");
		return false;
	}

	printf("INFO: who_am_i valid");
	return true;
}
/*
void IIm42652::test()
{
	uint8_t data[2] = {0, 0};
	reset();
	read(IIM42652::WHO_AM_I, data, 2);
}
*/

void IIm42652::setBank(uint8_t bank)
{
	uint8_t data[2] = {IIM42652::REG_BANK_SEL, bank};
	write(data, 2);
}

#ifdef IIM42652_SPI
bool IIm42652::read(uint8_t reg, uint8_t *data, uint16_t size)
{
	//the first bit of the first byte contains the read/write bit, the following 7 bits contins register address
	setMask(&reg, IIM42652::READ_WRITE_BIT);
	HAL_GPIO_WritePin(IIM_CS_GPIO_Port, IIM_CS_Pin, GPIO_PIN_RESET);
	if(HAL_SPI_Transmit(&hspi3, &reg, 1, 0xFFF) != HAL_OK)
	{
		printf("ERROR [read]: HAL_SPI_Transmit\r\n");
		return false;
	}

	if(HAL_SPI_Receive(&hspi3, data, size, 0xFFF) != HAL_OK)
	{
		printf("ERROR [read]: HAL_SPI_Receive\r\n");
		return false;
	}
	//HAL_SPI_TransmitReceive(&hspi3, &reg, data, size, 0xFFF);
	HAL_GPIO_WritePin(IIM_CS_GPIO_Port, IIM_CS_Pin, GPIO_PIN_SET);

	return true;
}

bool IIm42652::write(uint8_t *data, uint16_t size)
{
	//clearMask(data, IIM42652::READ_WRITE_BIT);
	HAL_GPIO_WritePin(IIM_CS_GPIO_Port, IIM_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, data, size, 0xFFFF);
	HAL_GPIO_WritePin(IIM_CS_GPIO_Port, IIM_CS_Pin, GPIO_PIN_SET);

	return true;
}

void IIm42652::readDMA(uint8_t reg, uint8_t *data, uint16_t size)
{
	//the first bit of the first byte contains the read/write bit, the following 7 bits contins register address
	setMask(&reg, IIM42652::READ_WRITE_BIT);
	HAL_SPI_TransmitReceive_DMA(&hspi3, &reg, data, size);
}

void IIm42652::writeDMA(uint8_t *data, uint16_t size)
{
	HAL_SPI_Transmit_DMA(&hspi3, data, size);
}
#else

bool IIm42652::read(uint8_t reg, uint8_t *data, uint16_t size)
{
	/*if(HAL_I2C_Master_Transmit(&hi2c1, address_, &reg, 1, 0xFFF)  != HAL_OK)
	{
		return false;
	}

	if(HAL_I2C_Master_Receive(&hi2c1, address_, data, size, 0xFFF)  != HAL_OK)
	{
		return false;
	}*/
	if(HAL_I2C_Mem_Read(&hi2c1, address_, reg, I2C_MEMADD_SIZE_8BIT, data, size, 0xFFF) != HAL_OK)
	{
		return false;
	}

	return true;
}

bool IIm42652::write(uint8_t *data, uint16_t size)
{
	if(HAL_I2C_Master_Transmit(&hi2c1, address_, data, size, 0xFFF) != HAL_OK)
	{
		return false;
	}
	/*if(HAL_I2C_Mem_Write(&hi2c1, address_, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], size - 1, 0xFFF) != HAL_OK)
	{
		return false;
	}*/
	return true;
}

#endif

