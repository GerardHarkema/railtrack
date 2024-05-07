/* Original version Copyright (c) 2007 Fabian Greif
 * Adapted by Gerard Harkema januari 2024
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
// ----------------------------------------------------------------------------


//#include <avr/io.h>
//#include <util/delay.h>
#include "Arduino.h"
#include "mcp2515.hpp"
#include "mcp2515_defs.h"


#ifdef __cplusplus
extern "C" {
#endif
#include <SPI.h>
/* original C header content */

#ifdef __cplusplus
}
#endif 

#define bit_is_set(sfr, bit) (_SFR_BYTE(sfr) & _BV(bit))
#define bit_is_clear(sfr, bit) (!(_SFR_BYTE(sfr) & _BV(bit)))

void activateMCP2512(){
	// SPI 10 MHz
	SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
	digitalWrite(MCP2512_CS_PINn, LOW);
}

void deactivateMCP2512(){
	digitalWrite(MCP2512_CS_PINn, HIGH);
	SPI.endTransaction();	
}

// -------------------------------------------------------------------------
// Schreibt/liest ein Byte ueber den Hardware SPI Bus

uint8_t spi_putc( uint8_t data )
{
	return SPI.transfer(data);
}

// -------------------------------------------------------------------------
void can_write_register( uint8_t adress, uint8_t data )
{
	activateMCP2512();
	
	spi_putc(SPI_WRITE);
	spi_putc(adress);
	spi_putc(data);
	
	deactivateMCP2512();
}

// -------------------------------------------------------------------------
uint8_t can_read_register(uint8_t adress)
{
	uint8_t data;
	
	activateMCP2512();
	
	spi_putc(SPI_READ);
	spi_putc(adress);
	
	data = spi_putc(0xff);	
	
	deactivateMCP2512();

	return data;
}

// -------------------------------------------------------------------------
void can_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{
	activateMCP2512();
	
	spi_putc(SPI_BIT_MODIFY);
	spi_putc(adress);
	spi_putc(mask);
	spi_putc(data);
	
	deactivateMCP2512();
}

// ----------------------------------------------------------------------------
uint8_t can_read_status(uint8_t type)
{
	uint8_t data;
	
	activateMCP2512();
	
	spi_putc(type);
	data = spi_putc(0xff);
	
	deactivateMCP2512();
	
	return data;
}

// ----------------------------------------------------------------------------
uint8_t can_reset(){
	activateMCP2512();
	spi_putc(SPI_RESET);
	deactivateMCP2512();

	// wait a little bit until the MCP2515 has restarted
#if defined(__AVR_ATmega328P__)
	_delay_us(10);
#else
	ets_delay_us(10);
#endif

	return true;
}

// -------------------------------------------------------------------------
uint8_t can_init(uint8_t speed, bool loopback)
{
	pinMode(MCP2512_CS_PINn, OUTPUT); // set the SS pin as an output
	deactivateMCP2512();

	SPI.begin();         // initialize the SPI library
	// SPI 10 MHz
	//SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

	// reset MCP2515 by software reset.
	// After this he is in configuration mode.
	can_reset();

	// Bitrate 250 kbps at 16 MHz
	can_write_register(CNF3, (1<<PHSEG21));
	can_write_register(CNF2, (1<<BTLMODE)|(1<<PHSEG11));

	uint8_t expect_cnf1_result;
#ifdef MCP2515_XTAL_FREQ
	#if MCP2515_XTAL_FREQ == XTAL_16MHZ
		#pragma message "compiled for 16MHz XTAL"
		can_write_register(CNF1, (1<<BRP1)|(1<<BRP0));
		expect_cnf1_result = (1<<BRP1)|(1<<BRP0);
	#elif MCP2515_XTAL_FREQ == XTAL_8MHZ
		#pragma message "compiled for 8MHz XTAL"
		can_write_register(CNF1, (1<<BRP0));
		expect_cnf1_result = (1<<BRP0);
	#else
		#error message "invalid xtal frequency defined"
	#endif
#else
	#error message "No xtal frequency defined"
#endif
	// activate interrupts
	can_write_register(CANINTE, (1<<RX1IE)|(1<<RX0IE));

	// test if we could read back the value => is the chip accessible?

	if (can_read_register(CNF1) != expect_cnf1_result) {

		Serial.println("Error readback CNF1");
		Serial.println(can_read_register(CNF1), HEX);

		//SET(LED2_HIGH);

		return false;
	}
	Serial.println("Oke readback CNF1");
	Serial.println(can_read_register(CNF1), HEX);
	
	// deaktivate the RXnBF Pins (High Impedance State)
	can_write_register(BFPCTRL, 0);
	
	// set TXnRTS as inputs
	can_write_register(TXRTSCTRL, 0);
	
	// turn off filters => receive any message
	can_write_register(RXB0CTRL, (1<<RXM1)|(1<<RXM0));
	can_write_register(RXB1CTRL, (1<<RXM1)|(1<<RXM0));
	
	// reset device to normal mode
	can_write_register(CANCTRL, loopback ? 64 : 0);
	return true;
	
}

// ----------------------------------------------------------------------------
// check if there are any new messages waiting

uint8_t can_check_message(void) {
//	return (!IS_SET(MCP2515_INT));
	return digitalRead(MCP1512_INT_PINn) ? 0 : ~0;
	//return 0;
}

// ----------------------------------------------------------------------------
// check if there is a free buffer to send messages

uint8_t can_check_free_buffer(void)
{
	uint8_t status = can_read_status(SPI_READ_STATUS);
	
	if ((status & 0x54) == 0x54) {
		// all buffers used
		return false;
	}
	
	return true;
}

// ----------------------------------------------------------------------------
uint8_t can_get_message(tCAN *message)
{
	// read status
	uint8_t status = can_read_status(SPI_RX_STATUS);
	uint8_t addr;
	uint8_t t;
	if (bit_is_set(status,6)) {
		// message in buffer 0
		addr = SPI_READ_RX;
	}
	else if (bit_is_set(status,7)) {
		// message in buffer 1
		addr = SPI_READ_RX | 0x04;
	}
	else {
		// Error: no message available
		return 0;
	}

	activateMCP2512();
	spi_putc(addr);
	
    uint32_t id1 = spi_putc(0xff);
    uint32_t id2 = spi_putc(0xff);
    uint32_t id3 = spi_putc(0xff);
    uint32_t id4 = spi_putc(0xff);

    message->flags.extended = bit_is_set(id2, 3) ? 1 : 0;

	// read id
	message->id  = id1 << 21;
	message->id |= (((id2 & 0xE0) >> 3) | (id2 & 0x03)) << 16;
    message->id |= id3 << 8;
    message->id |= id4;


	// read DLC
	uint8_t length = spi_putc(0xff) & 0x0f;
	
	message->length = length;
	message->flags.rtr = (bit_is_set(status, 3)) ? 1 : 0;
	
	// read data
	for (t=0;t<length;t++) {
		message->data[t] = spi_putc(0xff);
	}
	deactivateMCP2512();
	
	// clear interrupt flag
	if (bit_is_set(status, 6)) {
		can_bit_modify(CANINTF, (1<<RX0IF), 0);
	}
	else {
		can_bit_modify(CANINTF, (1<<RX1IF), 0);
	}
	
	return (status & 0x07) + 1;
}

// ----------------------------------------------------------------------------
uint8_t can_send_message(tCAN *message)
{
	uint8_t status = can_read_status(SPI_READ_STATUS);
	
	/* Statusbyte:
	 *
	 * Bit	Function
	 *  2	TXB0CNTRL.TXREQ
	 *  4	TXB1CNTRL.TXREQ
	 *  6	TXB2CNTRL.TXREQ
	 */
	uint8_t address;
	uint8_t t;

	if (bit_is_clear(status, 2)) {
		address = 0x00;
	}
	else if (bit_is_clear(status, 4)) {
		address = 0x02;
	} 
	else if (bit_is_clear(status, 6)) {
		address = 0x04;
	}
	else {
		// all buffer used => could not send message
		return 0;
	}
	
	activateMCP2512();
	spi_putc(SPI_WRITE_TX | address);
	
        uint32_t id1 = message->id >> 21;

        uint32_t id2 = ((message->id >> 13) & 0xE0) | ((message->id >> 16) & 0x03);

        if (message->flags.extended) {
            id2 |= 0x08;
        }

        uint32_t id3 = message->id >> 8;
        uint32_t id4 = message->id;

	spi_putc(id1); // 3
    spi_putc(id2); // 5
	
	spi_putc(id3);
	spi_putc(id4);
	
	uint8_t length = message->length & 0x0f;
	
	if (message->flags.rtr) {
		// a rtr-frame has a length, but contains no data
		spi_putc((1<<RTR) | length);
	}
	else {
		// set message length
		spi_putc(length);
		
		// data
		for (t=0;t<length;t++) {
			spi_putc(message->data[t]);
		}
	}
	deactivateMCP2512();
	
#if defined(__AVR_ATmega328P__)
	_delay_us(1);
#else
	//ets_delay_us(1);
	vTaskDelay(1);
#endif
	
	// send message
	activateMCP2512();
	address = (address == 0) ? 1 : address;
	spi_putc(SPI_RTS | address);
	deactivateMCP2512();
	
	return address;
}
