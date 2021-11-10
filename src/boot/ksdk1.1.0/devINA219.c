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

	/* 13: 16V, 12/11: gain 1, 10-7: default, 6-3: default, default */
	uint16_t payload_conf = 0b0000000110011111;

	// uint16_t payload_cal = 20480u;
	uint16_t payload_cal = 0xFFFF;

	WarpStatus init_status = configureSensorINA219(payload_conf, payload_cal);

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
configureSensorINA219(uint16_t payload_conf, uint16_t payload_cal)
{
	WarpStatus status1, status2;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	status1 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219F_CONF /* register address */,
							payload_conf
							);

	status2 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219F_CALIB /* register address */,
							payload_cal
							);

	if (status1 + status2 > 0)
	{
		return kWarpStatusCommsError;
	}

	return kWarpStatusOK;
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

uint8_t
readyToReadINA219()
{
	WarpStatus i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_BUS_V, 2 /* numberOfBytes */);
	uint16_t readSensorRegisterValue = (deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];

	if (i2cReadStatus != kWarpStatusOK)
	{
		/* force failure */
		readSensorRegisterValue = 0;
	}

	/* b1 is CNVR, conversion ready */
	return (readSensorRegisterValue & (1 << 1)) > 0;
}

void
printSensorDataINA219(int meas_reads)
{
	WarpStatus	i2cReadStatus;
	uint16_t readSensorRegisterValue;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);



	// kWarpSensorOutputRegisterINA219OUT_SHUNT_V
	// kWarpSensorOutputRegisterINA219OUT_BUS_V
	// kWarpSensorOutputRegisterINA219OUT_POWER
	// kWarpSensorOutputRegisterINA219OUT_CURRENT

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_SHUNT_V, 2 /* numberOfBytes */);
	readSensorRegisterValue = (deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		warpPrint("\r\nShunt: 0x%04x, %duV\r\n", readSensorRegisterValue, readSensorRegisterValue * 10);
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_BUS_V, 2 /* numberOfBytes */);
	readSensorRegisterValue = (deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		warpPrint("Bus: 0x%04x, %dmV\r\n", readSensorRegisterValue, (readSensorRegisterValue >> 3) * 4);
	}



	for (uint16_t meas_count = 0; meas_count < meas_reads; meas_count++)
	{
		while(readyToReadINA219() == 0)
		{
			__asm volatile ("nop");
		}

		warpPrint("Current meas. %d: ", meas_count);

		i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_CURRENT, 2 /* numberOfBytes */);
		readSensorRegisterValue = (deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];

		if (i2cReadStatus != kWarpStatusOK)
		{
			warpPrint(" ----,");
		}
		else
		{
			/* LSB = 6.25uA */
			warpPrint("Current: 0x%04x, %duA\r\n", readSensorRegisterValue, (readSensorRegisterValue * 6250)/1000);
		}


		/* Read power to reset conversion ready flag */	
		i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_POWER, 2 /* numberOfBytes */);
		readSensorRegisterValue = (deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];

		if (i2cReadStatus != kWarpStatusOK)
		{
			warpPrint(" ----,");
		}
		else
		{
			/* LSB = 400uW */
			warpPrint("Power: 0x%04x, %duW\r\n", readSensorRegisterValue, (readSensorRegisterValue * 12500)/1000);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219OUT_POWER, 2 /* numberOfBytes */);
	readSensorRegisterValue = (deviceINA219State.i2cBuffer[0] << 8) | deviceINA219State.i2cBuffer[1];

	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		/* LSB = 400uW */
		warpPrint("Power: 0x%04x, %duW\r\n", readSensorRegisterValue, (readSensorRegisterValue * 12500)/1000);
	}



}