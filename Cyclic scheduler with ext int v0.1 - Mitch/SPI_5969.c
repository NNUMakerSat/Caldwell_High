/*
 * SPI_Main.c
 *
 *  Created on: Sep 10, 2016
 *      Author: aaronewing
 *
 * SPI_Pulling.c
 *
 *  Created on: Aug 16, 2016
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

//////////////      FOR CONNOR    /////////////////////
#define DAC_CS 0x20;								///
#define POT_CS 0x02;								///
///////////////////////////////////////////////////////

uint8_t g_RXData;
uint32_t debug = 0;

////////////////////// SPI INIT /////////////////////////////////////
void init_SPI (uint8_t pin_Setting, bool mast_slav) {
	switch (pin_Setting) {
	case 0:											// Hub <- MSP430FR5969
	default:
		// Configure Primary Function Pins
		P1SEL0 |= BIT4 | BIT6 | BIT7;       		// P1.4 - CLK, P1.6 - SIMO, P1.7 - SOMI
		P1SEL1 &= ~(BIT4 | BIT6 | BIT7);

		// configure as GPIO used to enable SPI write from Hub
		P4SEL0 &= ~BIT1;
		P4SEL1 &= ~BIT1;							// P4.1 - GPIO Slave Ready Line (input)
	//	P4DIR &= ~BIT1;
		P4DIR |= BIT1;
		P4OUT &= ~BIT1;
	//.	P4IN |= BIT1;
		break;

	case 1:											// MSP430FR5969 -> Hub
		// Configure Secondary Function Pins
		P1SEL1 |= BIT6 | BIT7;              		// P1.6 - SIMO, P1.7 - SOMI
		P2SEL1 |= BIT2;              				// P2.2 - CLK

		// configure as GPIO used to enable SPI write to Hub
		P3SEL0 &= ~BIT5;
		P3SEL1 &= ~BIT5;							// P4.3 - GPIO Slave Ready Line (output)
		P3DIR |= BIT5;
		P3OUT &= ~BIT5;
		break;

	case 2:											// Polymer degradation board - POT
		// Configure Primary Function Pins
		P1SEL0 |= BIT6 | BIT7;              		// P1.6 - SIMO, P1.7 - SOMI
		P2SEL0 |= BIT2;              				// P2.2 - CLK

		// configure as GPIO used to enable SPI write for POT
		P1SEL0 &= ~POT_CS;
		P1SEL1 &= ~POT_CS;							// P4.1 - SYNC/Slave Select
		P1DIR |= POT_CS;
		P1OUT |= POT_CS;
		break;

	case 3:											// Polymer degradation board - DAC
		// Configure Primary Function Pins
		P1SEL0 |= BIT6 | BIT7;              		// P1.6 - SIMO, P1.7 - SOMI
		P2SEL0 |= BIT2;              				// P2.2 - CLK

		// configure as GPIO used to enable SPI write to DAC
		P2SEL0 &= ~DAC_CS;
		P2SEL1 &= ~DAC_CS;							// P2.5 - SYNC/Slave Select
		P2DIR |= DAC_CS;
		P2OUT |= DAC_CS;
		break;
	}

	 // Configure USCI_B0 for SPI operation
	 UCB0CTLW0 |= UCSWRST;                      		// **Put state machine in reset**

	 switch (mast_slav) {
	 case 0:
		 UCB0CTLW0 |= UCMST | UCSYNC | UCMSB | UCCKPL;	// 3-pin, 8-bit SPI master
	 case 1:
	 default:
		 UCB0CTLW0 |= UCSYNC | UCMSB | UCCKPL;			// 3-pin, 8-bit SPI slave
	 }
	                                          	  		// Clock polarity high, MSB
	 UCB0CTLW0 |= UCSSEL__SMCLK;              	 		// SMCLK

	 UCB0BRW = 0x0008;									// Divides SMCLK module by 8 (8MHz/8 = 1MHz)

	 UCB0CTLW0 &= ~UCSWRST;                   	 		// **Initialize USCI state machine**
}

////////////////////// SPI WRITE 8 BIT /////////////////////////////////////
void write_uint8_SPI (uint8_t tx_Data_8, uint8_t device_CS) {
	while (!(UCB0IFG & UCTXIFG)){};							// If able to TX

	switch (device_CS) {
		case 0:												// Hub <- MSP430FR5969
		default:
			while (!(UCB0IFG & UCTXIFG)) {};				// While TXing
			UCB0TXBUF = tx_Data_8;							// 8 bits transmitted
			while (UCB0STATW & UCBUSY) {};					// While not busy
			break;

		case 1:												// MSP430FR5969 -> Hub
			UCB0TXBUF = tx_Data_8;							// Loads data into TX buffer
//			P3OUT |= BIT5;									// Pulls GPIO high
	//		P4OUT |= BIT3;
			while (!(UCB0IFG & UCTXIFG)) {};				// While TXing
			while (UCB0STATW & UCBUSY) {};
//			P3OUT &= ~BIT5;
	//		P4OUT &= ~BIT3;
			++debug;
			break;

		case 2:												// POT
			P1OUT &= ~POT_CS;								// Pulls SYNC low
			while (!(UCB0IFG & UCTXIFG)) {};    			// While TXing
			UCB0TXBUF = tx_Data_8;							// 8 bits transmitted
			while (UCB0STATW & UCBUSY) {};
			P1OUT |= POT_CS;
			break;

		case 3:												// DAC
			P2OUT &= ~DAC_CS;								// Pulls SYNC low
			while (!(UCB0IFG & UCTXIFG)) {};    			// While TXing
			UCB0TXBUF = tx_Data_8;							// 8 bits transmitted
			while (UCB0STATW & UCBUSY) {};
			P2OUT |= DAC_CS;
			break;
	}
}

////////////////////// SPI WRITE 16 BIT /////////////////////////////////////
void write_uint16_SPI (uint16_t tx_Data_16, uint8_t device_CS) {
	while (!(UCB0IFG & UCTXIFG)){};							// If able to TX

	switch (device_CS) {
		case 0:												// Hub <- MSP430FR5969
		default:
			while (!(UCB0IFG & UCTXIFG)) {};				// While TXing
			UCB0TXBUF = (tx_Data_16 >> 8);					// First 8 bits transmitted
			while (!(UCB0IFG & UCTXIFG)) {};
			UCB0TXBUF = tx_Data_16;							// Last 8 bits transmitted (overflow is alright)
			while (UCB0STATW & UCBUSY) {};					// While not busy
			break;

		case 1:												// MSP430FR5969 -> Hub
			UCB0TXBUF = (tx_Data_16 >> 8);					// First 8 bits loaded into TX buffer
			P4OUT |= BIT1;									// Pulls GPIO high
			while (!(UCB0IFG & UCTXIFG)) {};				// While TXing
			while (UCB0STATW & UCBUSY) {};
//			P4OUT &= ~BIT1;
//			P4OUT |= BIT1;
			UCB0TXBUF = tx_Data_16;							// Last 8 bits into TX buffer (overflow is alright)
			while (UCB0STATW & UCBUSY) {};
//			P4OUT &= ~BIT1;
			break;

		case 2:												// POT
			P1OUT &= ~POT_CS;								// Pulls SYNC low
			while (!(UCB0IFG & UCTXIFG)) {};    			// While TXing
			UCB0TXBUF = (tx_Data_16 >> 8);					// First 8 bits transmitted
			while (!(UCB0IFG & UCTXIFG)) {};
			UCB0TXBUF = tx_Data_16;							// Last 8 bits transmitted (overflow is alright)
			while (UCB0STATW & UCBUSY) {};
			P1OUT |= POT_CS;
			break;

		case 3:												// DAC
			P2OUT &= ~DAC_CS;								// Pulls SYNC low
			while (!(UCB0IFG & UCTXIFG)) {};    			// While TXing
			UCB0TXBUF = (tx_Data_16 >> 8);					// First 8 bits transmitted
			while (!(UCB0IFG & UCTXIFG)) {};
			UCB0TXBUF = tx_Data_16;							// Last 8 bits transmitted (overflow is alright)
			while (UCB0STATW & UCBUSY) {};
			P2OUT |= DAC_CS;
			break;

		case 4:												// DAC-2
			P2OUT &= ~DAC_CS;								// Pulls SYNC low
			while (!(UCB0IFG & UCTXIFG)) {};    			// While TXing
			UCB0TXBUF = (tx_Data_16 >> 8);					// First 8 bits transmitted
			while (!(UCB0IFG & UCTXIFG)) {};
			UCB0TXBUF = tx_Data_16;							// Last 8 bits transmitted (overflow is alright)
			while (UCB0STATW & UCBUSY) {};
			P2OUT |= DAC_CS;
			break;
	}
}

////////////////////// SPI READ POLLING //////////////////////////////////
uint8_t read_SPI (void) {
	while (!(UCB0IFG & UCRXIFG)) {};    			// While RX flag is high
	g_RXData = UCB0RXBUF;							// 8 bits read
	return g_RXData;
}

uint16_t Read_SB_SPI (void) {						// Hub only
	while (!(UCB0IFG & UCTXIFG)) {};    			// While TX flag is high
	UCB0TXBUF = 0xAA;								// Transmits read_byte (ACK)
	while (UCB0STATW & UCBUSY) {};					// While not busy

	while (!(UCB0IFG & UCRXIFG)) {};    				// While RX flag is high
	g_RXData = UCB0RXBUF;							// 8 bits read
	return g_RXData;
}
