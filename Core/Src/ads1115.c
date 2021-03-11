/*
 * ads1115.c
 *
 *  Created on: Sep 24, 2020
 *      Author: SchofieChen
 */
#include "main.h"
//#include "Delay.h"
#include "ads1115.h"

extern I2C_HandleTypeDef hi2c1;
extern unsigned char ADSwrite[6];
extern uint8_t ADS1115_ADDRESS;
extern int16_t reading;
//extern int16_t result;
extern const float voltageConv;
//extern const float currentConv;

int maxRaw = 0;
int16_t minRaw = 1000;
float maxVal = 0.00;
float minVal = 0.00;

void Read_I2C(void)
{
	unsigned char TX_buffer[1];
	unsigned char RX_buffer[1];

	/* i2c_addr
	 * {7'b1001000, {R/W}}
	 * W: 10010001(0x91)
	 * R: 10010000(0x90)
	 */
	unsigned char i2c_addr = 0x90;

	TX_buffer[0] = 0x55;
	HAL_I2C_Master_Transmit(&hi2c1, i2c_addr, (uint8_t *)TX_buffer, 1, 100);
	HAL_I2C_Master_Receive(&hi2c1, i2c_addr+1, RX_buffer, 1, 100);
	//delay_ms(1);
}

void Read_ads1115(ADS1115_BOARD * ADS1115_BOARD_SELECT,uint8_t ADDRESS)
{
	/* ADSwrite[x]
	 * ----------------------------------------------------------------------
	 * [14:12] 	| Input multiplexer configuration
	 * 			| 000 : CH0 - CH1
	 * 			| 011 : CH2 - CH3
	 * ----------------------------------------------------------------------
	 * [11:9] 	| PGA mode
	 * 			| 000 : FSR = ±6.144 V
	 * ----------------------------------------------------------------------
	 * [8] 		| Device operating mode
	 * 			| 0 : Continuous-conversion mode
	 *			| 1 : Single-shot mode or power-down state (default)
	 * ----------------------------------------------------------------------
	 * [7:5] 	| Data rate
	 * 		 	| 000 : 8 SPS
	 * 		 	| 100 : 128 SPS (default)
	 * 		 	| 111 : 860 SPS
	 * ----------------------------------------------------------------------
	 * [4]		| Compare Mode
	 * 			| 0: tradition comparator (default
	 * 			| 1: Window comparator
	 * ----------------------------------------------------------------------
	 * [3]		| Comparator polarity
	 * 			| 0 : Active low (default)
	 * 			| 1 : Active high
	 * ----------------------------------------------------------------------
	 * [2]		| Latching polarity
	 * 			| 0 : Nonlatching comparator (default)
	 * 			| 1 : Latching comparator
	 * ----------------------------------------------------------------------
	 * [1:0]	| Comparator queue and disable
	 * 			| 00 : Assert after one conversion
	 *			| 01 : Assert after two conversions
	 *			| 10 : Assert after four conversions
	 *			| 11 : Disable comparator and set ALERT/RDY pin to high-impedance (default)
	 * ----------------------------------------------------------------------
	 *
	 * ADSwrite[0] : Frame 2: Address Pointer Register
	 * ADSwrite[1] : Frame 3: Data Byte 1 (MSB)
	 * ADSwrite[2] : Frame 4: Data Byte 2 (LSB)
	 *
	 * Frame2: {8'b0,P[1],P[0]}
	 * 		> {P[1],P[0]}
	 * 		> 00: Conversion register
	 * 		> 01: Config register
	 * Frame3: D15 - D8
	 * Frame4: D7 - D0
	 *
	 */

	/* ADDRESS
	 * {6'b10010, {A1,A0}}
	 * {A1,A0}: 00: GND (8'b1001000 - 48)
	 * 			01: VDD (8'b1001001 - 49)
	 * 			10: SDA (8'b1001010 - 4A)
	 * 			11: SCL (8'b1001011 - 4B)
	 *
	 */
	for( int i = 0; i < 2; i++) {

		/* Config register
		 * using HAL_I2C_Master_Transmit
		 * --------------------------------------------
		 * ADSwrite[0]	| pointer address
		 * 				| 00: Conversion register
		 * 				| 01: Config register
		 * --------------------------------------------
		 * ADSwrite[1]	| config MSB
		 * ADSwrite[2]	| config LSB
		 * --------------------------------------------
		 */
		ADSwrite[0] = 0x01;
		switch(i)
		{
			case(0): //MSB
				ADSwrite[1] = 0x81; // 1.000.000.1 CH0 + CH1
			break;
			case(1):
				ADSwrite[1] = 0xB1;// 1.011.000.1 CH2 + CH3
			break;
		}
		ADSwrite[2] = 0xE3; // 1.000.0011 LSB 03/23/83/E3
		HAL_I2C_Master_Transmit(&hi2c1, ADDRESS<<1, ADSwrite, 3, 100);


		/* Conversion register
		 * transform to Conversion register
		 * ADSwrite[0]: 00
		 */
		ADSwrite[0] = 0x00;
		HAL_I2C_Master_Transmit(&hi2c1, ADDRESS<<1, ADSwrite, 1, 100);
		//HAL_Delay(20);
		osDelay(2); // millisec, must have delay time for catch new raw data


		/* Read from ADS
		 * 2Byte, FILO
		 * MSB into ADSwrite[0]
		 * LSB into ADSwrite[1]
		 */
		HAL_I2C_Master_Receive(&hi2c1, ADDRESS<<1, ADSwrite, 2, 100);
		reading = (ADSwrite[0] << 8 | ADSwrite[1]);
		//result = reading;

		if(reading >= 0x8000 && reading <= 0xffff)
		{
			reading = 0xffff - reading;
		}
		else if(reading >= 0xffff)
		{
			reading = 0;
		}



		// update value
		if(i == 0)
		{
			// for voltage mode
			//ADS1115_BOARD_SELECT->ADS1115_CH1.data = reading * voltageConv;
			ADS1115_BOARD_SELECT->ADS1115_CH1.data = reading;

			//for current mode
			//ADS1115_BOARD_SELECT->ADS1115_CH1.data = ((reading * currentConv) + 4);
		}
		else if(i == 1)
		{
			// for voltage mode
			//ADS1115_BOARD_SELECT->ADS1115_CH2.data = reading * voltageConv;
			//ADS1115_BOARD_SELECT->ADS1115_CH2.data = reading;

			//for current mode
			//ADS1115_BOARD_SELECT->ADS1115_CH2.data = ((reading * currentConv) + 4);
			ADS1115_BOARD_SELECT->ADS1115_CH2.data = reading;
		}
	}

}

int Convert2Modbus(float Data)
{
	int ConvertData = 0;

	ConvertData = ((Data+5)*65535)/10;

	return ConvertData;
}
