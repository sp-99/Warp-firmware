/*
	Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	2018-onwards, see git log.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA219.h"


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress			= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	/* 13: 16V, 12/11: default, 10-7: default, 6-3: default, default */
	uint16_t payload_conf = 0b0001100110011111;

	WarpStatus init_status = configureSensorINA219(payload_conf);

	warpPrint("init status INA219: %d", (int) init_status);
	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
	uint8_t		payloadBytes[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00: case 0x05:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadBytes[0] = (uint8_t) ((payload & 0xFF00) >> 8);
	payloadBytes[1] = (uint8_t) (payload & 0x00FF);
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadBytes,
							2,
							gWarpI2cTimeoutMilliseconds);

	// uint32_t instance,
 //                                            const i2c_device_t * device,
 //                                            const uint8_t * cmdBuff,
 //                                            uint32_t cmdSize,f
 //                                            const uint8_t * txBuff,
 //                                            uint32_t txSize,
 //                                            uint32_t timeout_ms)

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint16_t payload_conf)
{
	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	return writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219F_CONF /* register address F_SETUP */,
							payload_conf /* payload: Disable FIFO */
							);
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	/* INA219 registers are two bytes in length */
	numberOfBytes = 2;

	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: 
		case 0x04: case 0x05:
		{
			/* OK */
			break;
		}
		
		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceINA219State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

// void
// printSensorDataINA219(bool hexModeFlag)
// {
// 	uint16_t	readSensorRegisterValueLSB;
// 	uint16_t	readSensorRegisterValueMSB;
// 	int16_t		readSensorRegisterValueCombined;
// 	WarpStatus	i2cReadStatus;


// 	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

// 	/*
// 	 *	From the INA219 datasheet:
// 	 *
// 	 *		"A random read access to the LSB registers is not possible.
// 	 *		Reading the MSB register and then the LSB register in sequence
// 	 *		ensures that both bytes (LSB and MSB) belong to the same data
// 	 *		sample, even if a new data sample arrives between reading the
// 	 *		MSB and the LSB byte."
// 	 *
// 	 *	We therefore do 2-byte read transactions, for each of the registers.
// 	 *	We could also improve things by doing a 6-byte read transaction.
// 	 */
// 	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_X_MSB, 2 /* numberOfBytes */);
// 	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
// 	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
// 	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

// 	/*
// 	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
// 	 */
// 	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		warpPrint(" ----,");
// 	}
// 	else
// 	{
// 		if (hexModeFlag)
// 		{
// 			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
// 		}
// 		else
// 		{
// 			warpPrint(" %d,", readSensorRegisterValueCombined);
// 		}
// 	}

// 	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_Y_MSB, 2 /* numberOfBytes */);
// 	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
// 	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
// 	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

// 	/*
// 	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
// 	 */
// 	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		warpPrint(" ----,");
// 	}
// 	else
// 	{
// 		if (hexModeFlag)
// 		{
// 			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
// 		}
// 		else
// 		{
// 			warpPrint(" %d,", readSensorRegisterValueCombined);
// 		}
// 	}

// 	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_Z_MSB, 2 /* numberOfBytes */);
// 	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
// 	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
// 	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 6) | (readSensorRegisterValueLSB >> 2);

// 	/*
// 	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
// 	 */
// 	readSensorRegisterValueCombined = (readSensorRegisterValueCombined ^ (1 << 13)) - (1 << 13);

// 	if (i2cReadStatus != kWarpStatusOK)
// 	{
// 		warpPrint(" ----,");
// 	}
// 	else
// 	{
// 		if (hexModeFlag)
// 		{
// 			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
// 		}
// 		else
// 		{
// 			warpPrint(" %d,", readSensorRegisterValueCombined);
// 		}
// 	}
// }