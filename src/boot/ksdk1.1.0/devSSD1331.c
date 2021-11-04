#include <stdint.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 11),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};


static int
writeOLED(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);


	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	// char status_string[10];

	// sprintf(status_string, "SPI: [%d]\r\n", (int) status);

	// SEGGER_RTT_WriteString(0, status_string);

	return status;
}

static int
writeCommand(uint8_t commandByte)
{
	OSA_TimeDelay(100);
	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	return writeOLED(commandByte);
}

static int
writeData(uint8_t commandByte)
{
	/*
	 *	Drive DC high (data).
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinDC);

	return writeOLED(commandByte);
}







int
devSSD1331init(void)
{
	SEGGER_RTT_WriteString(0, "OLED INIT\r\n");
	OSA_TimeDelay(100);

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	warpEnableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 11u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);




	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color 01110010
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B); // disable power save mode
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	// writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	// writeCommand(0x80);
	// writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	// writeCommand(0x80);
	// writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	// writeCommand(0x80);


	writeCommand(kSSD1331CommandSETCOLUMN);		// 0x15
	writeCommand(0x00);
	writeCommand(0x5F);

	writeCommand(kSSD1331CommandSETROW);		// 0x75
	writeCommand(0x00);
	writeCommand(0x3F);

	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel 0xAF


	writeCommand(kSSD1331CommandDISPLAYALLON);
	


	// /*
	//  *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	//  */
	// writeCommand(kSSD1331CommandFILL);	// 0x26
	// writeCommand(0x01);

	// /*
	//  *	Clear Screen
	//  */
	// writeCommand(kSSD1331CommandCLEAR); // 0x25
	// writeCommand(0x00);
	// writeCommand(0x00);
	// writeCommand(0x5F);
	// writeCommand(0x3F);

#if 0
	SEGGER_RTT_WriteString(0, "OLED Writing Pixel Data\r\n");

	uint16_t row_count = 0;
	for (row_count = 0; row_count < 96; row_count += 1)
	{
		uint16_t col_count = 0;

		for (col_count = 0; col_count < 64; col_count++)
		{	
			// writeData(0x07E0);
			writeData(0xF800);
			// writeData(0x001F);
			// writeData(0x0000);
			// writeData(0xFFFF);
		}
	}

	SEGGER_RTT_WriteString(0, "OLED Pixel Data Written\r\n");
#endif


	// writeCommand(kSSD1331CommandDISPLAYALLOFF);

	// writeCommand(kSSD1331CommandDISPLAYALLON);


	// writeCommand(0xBC);
	// writeCommand(0xBC);
	// writeCommand(0xBC);
	// writeCommand(0xBC);
	// writeCommand(0x5F);

	// writeCommand(0xBC);
	// writeCommand(0xBC);
	// writeCommand(0xBC);
	// writeCommand(0x5F);



	// writeCommand(kSSD1331CommandDRAWRECT);

	// writeCommand(0x02);
	// writeCommand(0x02);

	// writeCommand(0x5F);
	// writeCommand(0x30);

	// writeCommand(0x1C);
	// writeCommand(0x00);
	// writeCommand(0x00);

	// writeCommand(0x00);
	// writeCommand(0x00);
	// writeCommand(0x28);



	// writeCommand(kSSD1331CommandDRAWLINE); // 0x21
	// writeCommand(0x30); // col start
	// writeCommand(0x10); // row start
	// writeCommand(0x5E); // col end
	// writeCommand(0x10); // row end
	// writeCommand(0x00); // colour C
	// writeCommand(0x00); // colour B
	// writeCommand(0x80); // colour A

	/*
	 *	Clear Screen
	 */
	// writeCommand(kSSD1331CommandCLEAR); // 0x25
	// writeCommand(0x00);
	// writeCommand(0x00);
	// writeCommand(0x5F);
	// writeCommand(0x3F);


	// writeCommand(kSSD1331CommandCLEAR); // 0x25
	// writeCommand(0x00);
	// writeCommand(0x00);
	// writeCommand(0x00);
	// writeCommand(0x00);


	/*
	 *	Any post-initialization drawing commands go here.
	 */
	//...


	// writeCommand(kSSD1331CommandDISPLAYALLON);



	// writeCommand(kSSD1331CommandDRAWLINE);
	// writeCommand(0x01);
	// writeCommand(0x10);
	// writeCommand(0x28);
	// writeCommand(0x04);
	// writeCommand(0xFF);
	// writeCommand(0x00);
	// writeCommand(0x00);



	// writeCommand(kSSD1331CommandDRAWRECT);

	// // writeCommand(kSSD1331CommandSETCOLUMN);
	// writeCommand(0x03);
	// // writeCommand(kSSD1331CommandSETROW);
	// writeCommand(0x02);

	// // writeCommand(kSSD1331CommandSETCOLUMN);
	// writeCommand(0x12);
	// // writeCommand(kSSD1331CommandSETROW);
	// writeCommand(0x15);

	// // writeCommand(kSSD1331CommandCONTRASTC);
	// writeCommand(28u);
	// // writeCommand(kSSD1331CommandCONTRASTB);
	// writeCommand(0u);
	// // writeCommand(kSSD1331CommandCONTRASTA);
	// writeCommand(0u);

	// // writeCommand(kSSD1331CommandCONTRASTC);
	// writeCommand(0u);
	// // writeCommand(kSSD1331CommandCONTRASTB);
	// writeCommand(0u);
	// // writeCommand(kSSD1331CommandCONTRASTA);
	// writeCommand(40u);




	return 0;
}
