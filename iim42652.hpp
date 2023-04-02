/*
 * iim42652.hpp
 *
 *  Created on: 19 mar 2023
 *      Author: bursztyn
 */

#ifndef INC_IIM42652_HPP_
#define INC_IIM42652_HPP_

#include "stdint.h"

namespace IIM42652
{
	enum RegMap
	{
		//BANK 0
		SENSOR_CONFIG0 = 0x03,
		DEVICE_CONFIG = 0x11,
		INT_CONFIG = 0x14,
		FIFO_CONFIG = 0x16,
		ACCEL_DATA_X1 = 0x1E,
		ACCEL_DATA_X0,
		ACCEL_DATA_Y1,
		ACCEL_DATA_Y0,
		ACCEL_DATA_Z1,
		ACCEL_DATA_Z0,
		INT_STATUS = 0x2D,
		FIFO_DATA = 0x30,
		INTF_CONFIG1 = 0x4D,
		PWR_MGMT0,
		GYRO_CONFIG0 = 0x50,
		ACCEL_CONFIG0,
		GYRO_CONFIG1,
		GYRO_ACCEL_CONFIG0,
		ACCEL_CONFIG1,
		INT_CONFIG0 = 0x63,
		INT_SOURCE0 = 0x65,
		WHO_AM_I = 0x75,
		REG_BANK_SEL,
		FIFO_CONFIG1 = 0x95,
		FIFO_CONFIG2,
		FIFO_CONFIG3,

		//BANK 1
		INTF_CONFIG5 = 0x7B,

		//BANK 2
		ACCEL_CONFIG_STATIC2 = 0x03
	};

	enum Int1Mode
	{
		PULSED = 0x00,
		LATCHED = 0x04,
	};

	enum Int1Drive
	{
		OPEN_DRAIN = 0x00,
		PUSH_PULL = 0x02
	};

	enum Int1Polarity
	{
		ACTIVE_LOW = 0x00,
		ACTIVE_HIGH = 0x01,
	};

	struct IntConfig
	{
		Int1Mode mode;
		Int1Drive drive;
		Int1Polarity polarity;
	};

	enum DeviceConfig
	{
		SOFT_RESET_CONFIG = 0x01,
	};

	//by default is enabled
	enum SensorConfig0
	{
		GYRO_DISABLE = 0x38,
		ACCEL_DISABLE = 0x7,
		SENSOR_DISABLE = 0x3f
	};

	enum IntfConfig1
	{
		CLKSEL_INTERNAL,
		CLKSEL_PLL = 0x01,
		CLKSEL_DISABLE = 0x03,
		RTC_MODE_EN = 0x04,
	};

	enum IntfConfig5
	{
		INT2,
		FSYNC = 0x02,
		CLKIN = 0x04,
		PIN_9_CLEAR_MSK = 0x06
	};

	enum DrdyIntConfig0
	{
		DRDY_INT_CLR_STATUS_READ = 0,
		DRDY_INT_CLR_SENSOR_REG_READ = 0x20,
		DRDY_INT_CLR_SENSOR_REG_STATUS_READ = 0x30,
		DRDY_INT_CLR_MASK = 0x30
	};

	enum FifoIntConfig0
	{
		FIFO_FULL_INT_CLR_STATUS_READ = 0x00,
		FIFO_FULL_INT_CLR_1BYTE_READ = 0x02,
		FIFO_FULL_INT_CLR_STATUS_1BYTE_READ = 0x03,
		FIFO_FULL_INT_CLR_MASK = 0x03,
	};

	enum IntSource0
	{
		FIFO_FULL_INT1_EN = 0x01,
		FIFO_THS_INT1_EN = 0x02,
		DRDY_INT1_EN = 0x08,
		RESET_DONE_INT1_EN = 0x10,
		PLL_RDY_INT1_EN = 0x20,
	};

    /*
    Gyroscope needs to be kept ON for a minimum of 45ms
    */

    /*
    When transitioning from OFF to any of the other modes, do not issue any
    register writes for 200µs.
    */
    enum PwrMgmt0
    {
		MODE_OFF = 0x0,

        ACCEL_MODE_LOW_NOISE = 0x03,
        GYRO_MODE_LOW_NOISE = 0x0C,


		ACCEL_MODE_CLEAR_MSK = 0x03,
		GYRO_MODE_CHEC_MSK = 0x0C,
	};

	enum Bank
	{
		BANK0,
		BANK1,
		BANK2,
		BANK3,
		BANK4
	};

	enum SensorMode
	{
		TURN_OFF,
		LOW_POWER,
		LOW_NOISE
	};

    enum ODR
    {
        ODR_32_kHz = 0x01,
        ODR_16_kHZ,
        ODR_2_kHZ = 0x05,
		ODR_1_kHZ = 0x06,
		ODR_200_HZ = 0x07,
		ODR_CLEAR_MSK = 0xE0
    };

    enum AccelSensitivity
	{
    	ACCEL_SEN_16G = 2048,
		ACCEL_SEN_8G = 4096,
		ACCEL_SEN_4G = 8192,
		ACCEL_SEN_2G = 16384
	};

    enum AccelFS
    {
        ACCEL_FS_8G = 0x20,
        ACCEL_FS_4G = 0x40,
        ACCEL_FS_2G = 0x60,
		ACCEL_FS_CLEAR_MSK = 0x0F
    };

    enum GyroFS
    {
        GYRO_FS_250_DPS = 0x60,
        GYRO_FS_150_DPS = 0x80,
		GYRO_FS_CLEAR_MSK = 0x0F
    };

	enum AddressFormat
	{
		READ_WRITE_BIT = 0x80,
	};

	enum SampleBW
	{
		ODR_DIV_2 = 0x0,
		ODR_DIV_4 = 0x1, //default
		ODR_DIV_5 = 0x2,
		ODR_DIV_8 = 0x3,
		ODR_DIV_10 = 0x4,

		BANDWIDTH_CLEAR_MSK = 0x0F
	};

	enum FifoConfig
	{
		FIFO_BYPASS = 0x00,
		STREAM_TO_FIFO = 0x40,
		FIFO_CONFIG_CLEAR_MSK = 0xC0
	};

	enum FifoConfig1
	{
		FIFO_ACCEL_EN = 0x01,
		FIFO_TEMP_EN = 0x04,
		FIFO_WM_GT_EN = 0x20,
		FIFO_RESUME_PARTIAL_RD = 0x40
	};

	struct Packet1
	{
		uint8_t header;
		uint8_t xHigh;
		uint8_t xLow;
		uint8_t yHigh;
		uint8_t yLow;
		uint8_t zHigh;
		uint8_t zLow;
		uint8_t temp;
	};
}

class IIm42652
{
public:
	IIm42652();
    /** Required registers:

        * PWR_MGMT0
        * GYRO_CONFIG0
        * ACCEL_CONFIG0
        * INTF_CONFIG1 //rtc_mode
        * INT_CONFIG0 //for testing - data ready int clear on ...
    */
	bool config(IIM42652::ODR accelODR, IIM42652::AccelFS accelFs,
				IIM42652::ODR gyroODR, IIM42652::GyroFS gyroFs,
				IIM42652::SampleBW accelBW, IIM42652::SampleBW gyroBW,
				uint8_t rtcEnable);
	bool config();
    void configDRDY(IIM42652::DrdyIntConfig0 clrOption);
    void configInt1(IIM42652::IntConfig &config);
    void configFIFO(IIM42652::FifoConfig1 accelEn, IIM42652::FifoConfig1 gyroEn,
    				IIM42652::FifoIntConfig0 clrOption);
	void reset();
	void fetchAccelSample();
	void convertSamplesToG(float *samples);
	void accelLowNoiseMode();
	void gyroLowNoiseMode();
private:
	bool checkWhoAmI();
    void setBank(uint8_t bank);
	void configAccel(IIM42652::ODR odr, IIM42652::AccelFS fs,
					IIM42652::SampleBW bw);
	void configGyro(IIM42652::ODR odr, IIM42652::GyroFS fs,
				IIM42652::SampleBW bw);
	void enableCLKINMode();
	void accelTurnOff();
	uint8_t getAccelMode();

	inline void read(uint8_t reg, uint8_t *data, uint16_t size);
    inline void write(uint8_t *data, uint16_t size);

    inline void setMask(uint8_t *val, uint8_t mask)
	{
		*val |= mask;
	}

	inline void clearMask(uint8_t *val, uint8_t mask)
	{
		*val &= ~mask;
	}

	inline uint8_t checkMask(uint8_t val, uint8_t mask, uint8_t check)
	{
		return (val & mask) == check;
	}

	const uint8_t whoAmI_ = 0x6F;
	IIM42652::AccelSensitivity accelSensitivity_;
	int16_t rawAccelData_[3];
};


#endif /* INC_IIM42652_HPP_ */
