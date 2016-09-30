/*
 * SPI_Main.h
 *
 *  Created on: Sep 10, 2016
 *      Author: aaronewing
 */
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

#ifndef SPI_MAIN_H_
#define SPI_MAIN_H_

void init_SPI (uint8_t pin_Setting, bool mast_slav);				// initalizes SPI clk rate and which pins are being used
// pin_Setting (0 - Hub, 1 - SB -> Hub, 2 - POT, 3 - DAC)
// mast_slav (0 if Master, 1 if slave (defaults to slave))

void write_uint8_SPI (uint8_t tx_Data_8, uint8_t device_CS);		// writes 8 bits with SPI
// tx_Data_8 (single 8 bit variable to transmit through SPI)
// device_CS (0 - Hub, 1 - SB -> Hub, 2 - POT, 3 - DAC)

void write_uint16_SPI (uint16_t tx_Data_16, uint8_t device_CS);		// writes 16 bits with SPI
// tx_Data_16 (single 16 bit variable to transmit through SPI)
// device_CS (0 - Hub, 1 - SB -> Hub, 2 - POT, 3 - DAC, 4 - DAC 2)

uint8_t read_SPI (void);											// reads 8 bits with SPI
// returns 8 bit variable called "g_RXData"

uint16_t Read_SB_SPI (void);										// Hub only
// Hub reads what SB transmits (by TXing and then reading what's on RX buffer)

#endif /* SPI_MAIN_H_ */
