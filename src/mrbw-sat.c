/*************************************************************************
Title:    MRBus Wireless Simple Analog Throttle
Authors:  Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2015 Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include "mrbee.h"

uint8_t mrbus_dev_addr = 0;
uint8_t pkt_count = 0;


// ******** Start 100 Hz Timer, 0.16% error version (Timer 0)
// If you can live with a slightly less accurate timer, this one only uses Timer 0, leaving Timer 1 open
// for more advanced things that actually need a 16 bit timer/counter

// Initialize a 100Hz timer for use in triggering events.
// If you need the timer resources back, you can remove this, but I find it
// rather handy in triggering things like periodic status transmissions.
// If you do remove it, be sure to yank the interrupt handler and ticks/secs as well
// and the call to this function in the main function

volatile uint8_t ticks;
volatile uint16_t decisecs=0;
volatile uint16_t update_decisecs=10;

uint8_t readDipSwitch()
{
	uint8_t val = PINB & 0x07;
	val |= 0x08 & (PIND>>1);
	val |= 0x10 & (PINC>>1);
	return val;
}

void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0xC2;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
	}
}

// End of 100Hz timer

// **** Bus Voltage Monitor
// Uncomment this block (and the ADC initialization in the init() function) if you want to continuously monitor bus voltage

volatile uint8_t throttlePot=0;

ISR(ADC_vect)
{
	static uint16_t throttlePotAccumulator=0;
	static uint8_t thottlePotCount=0;

	throttlePotAccumulator += ADC;
	if (++thottlePotCount >= 64)
	{
//		throttlePotAccumulator = ADC;
		// Divide by 64 samples, and then whack off another 8 to drop it from 10 bits to 7
		throttlePot = throttlePotAccumulator>>9;
		throttlePotAccumulator = 0;
		thottlePotCount = 0;
	}
}


void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	if (0 == mrbusPktQueuePop(&mrbeeRxQueue, rxBuffer, sizeof(rxBuffer)))
		return;


	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	} 
	else if ('W' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM WRITE Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'w';
		eeprom_write_byte((uint8_t*)(uint16_t)rxBuffer[6], rxBuffer[7]);
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = rxBuffer[7];
		if (MRBUS_EE_DEVICE_ADDR == rxBuffer[6])
			mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;	
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM READ Packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 8;			
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];
		txBuffer[7] = eeprom_read_byte((uint8_t*)(uint16_t)rxBuffer[6]);			
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 16;
		txBuffer[MRBUS_PKT_TYPE] = 'v';
		txBuffer[6]  = MRBUS_VERSION_WIRELESS;
		txBuffer[7]  = 0xFF & ((uint32_t)(GIT_REV))>>16; // Software Revision
		txBuffer[8]  = 0xFF & ((uint32_t)(GIT_REV))>>8; // Software Revision
		txBuffer[9]  = 0xFF & (GIT_REV); // Software Revision
		txBuffer[10]  = 1; // Hardware Major Revision
		txBuffer[11]  = 0; // Hardware Minor Revision
		txBuffer[12] = 'S';
		txBuffer[13] = 'A';
		txBuffer[14] = 'T';
		txBuffer[15] = ' ';
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}

	// FIXME:  Insert code here to handle incoming packets specific
	// to the device.

	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	return;	
}

void init(void)
{
	// FIXME:  Do any initialization you need to do here.
	
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif	

	pkt_count = 0;

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Bogus addresses, fix to default address
	if (0xFF == mrbus_dev_addr || 0x00 == mrbus_dev_addr)
	{
		mrbus_dev_addr = 0x03;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	
	if (0x03 == mrbus_dev_addr)
		mrbus_dev_addr = 0x20 + readDipSwitch();
	
	
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	// This line assures that update_decisecs is at least 1
	update_decisecs = max(1, update_decisecs);
	
	// FIXME: This line assures that update_decisecs is 2 seconds or less
	// You probably don't want this, but it prevents new developers from wondering
	// why their new node doesn't transmit (uninitialized eeprom will make the update
	// interval 64k decisecs, or about 110 hours)  You'll probably want to make this
	// something more sane for your node type, or remove it entirely.
	update_decisecs = min(20, update_decisecs);


	// Setup ADC
	ADMUX  = _BV(REFS0) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0); // AVCC reference, ADC7 channel
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	throttlePot = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}

#define MRBUS_TX_BUFFER_DEPTH 2
#define MRBUS_RX_BUFFER_DEPTH 2

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

int main(void)
{
	uint8_t lastThrottlePot = 0;	
	uint8_t lastDir = 0, dir=0;
	
	// Application initialization
	init();

	DDRD |= _BV(PD7) | _BV(PD6) | _BV(PD5);
	DDRC |= _BV(PC4) | _BV(PC0);
	
	PORTD &= ~_BV(PD7);

	// Ground the bottom of the pot
	PORTC &= ~_BV(PC0);

	// Enable pullups for dip switch
	PORTB |= _BV(PB0) | _BV(PB1) | _BV(PB2);
	PORTC |= _BV(PC5);	
	PORTD |= _BV(PD4);

	// Enable direction switch and aux switch pullups
	PORTC |= _BV(PC3) | _BV(PC2) | _BV(PC1);

	// Initialize a 100 Hz timer.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	sei();	

	while (1)
	{
		wdt_reset();

		switch (PINC & 0x0C)
		{
			case 0x08:
				dir = 1;
				break;
			case 0x04:
				dir = 2;
				break;
			case 0x0C:
				dir = 0;
				break;
		}
		
		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
			PktHandler();		
		
		// Transmission criteria...
		// > 2 decisec from last transmission
		// 
		
		if ((((throttlePot != lastThrottlePot || dir != lastDir) && decisecs > 1) || (decisecs >= update_decisecs))
				&& !(mrbusPktQueueFull(&mrbeeTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			lastThrottlePot = throttlePot;
			lastDir = dir;
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 8;
			txBuffer[5] = 'C';
			txBuffer[6] = dir;
			// Don't send a speed if we're in neutral
			if (0 == dir)
				txBuffer[7] = 0;
			else
				txBuffer[7] = lastThrottlePot;			
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
		}
			
		if (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			mrbeeTransmit();
		}
	}
}



