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
#include <avr/sleep.h>
#include <util/delay.h>

#include "mrbee.h"

#define MRBUS_EE_DEVICE_SLEEP_TIMEOUT 0x10

// Default sleep time in minutes
#define DEFAULT_SLEEP_TIME 5

// VBATT_OKAY is the battery voltage (in decivolts) above which the batteries are considered "fine"
#define VBATT_OKAY  220 
// VBATT_WARN is the battery voltage (in decivolts) above which the batteries are considered a warning, but not
//  in critical shape.  This should *always* be less than VBATT_OKAY
#define VBATT_WARN  200

#define XBEE_SLEEP_PD             PD7
#define LED_GREEN_PD              PD6
#define LED_RED_PD                PD5

#define THROTTLE_DIR_COMMON_PC    PC4
#define THROTTLE_DIR_REVERSE_PC   PC3
#define THROTTLE_DIR_FORWARD_PC   PC2
#define THROTTLE_AUX_BUTTON_PC    PC1
#define THROTTLE_POT_ENABLE_PC    PC0

#define THROTTLE_DIR_FORWARD  (_BV(THROTTLE_DIR_REVERSE_PC))
#define THROTTLE_DIR_REVERSE  (_BV(THROTTLE_DIR_FORWARD_PC))
#define THROTTLE_DIR_IDLE     (_BV(THROTTLE_DIR_REVERSE_PC) | _BV(THROTTLE_DIR_FORWARD_PC))
#define THROTTLE_DIR_MASK     (_BV(THROTTLE_DIR_REVERSE_PC) | _BV(THROTTLE_DIR_FORWARD_PC))

uint8_t mrbus_dev_addr = 0;
volatile uint8_t pktTimeout = 0;

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
volatile uint8_t status = 0;
uint8_t sleep_tmr_reset_value;

void enableAddressSwitch()
{
	// Make DIP switch pins inputs, to revert what may have been done in sleep
	DDRB &= ~(_BV(PB0) | _BV(PB1) | _BV(PB2));
	DDRC &= ~_BV(PC5);
	DDRD &= ~_BV(PD4);

	// Enable pullups for dip switch
	PORTB |= _BV(PB0) | _BV(PB1) | _BV(PB2);
	PORTC |= _BV(PC5);	
	PORTD |= _BV(PD4);
}

void disableAddressSwitch()
{
	// Kill pull-ups for dip switch and aux button
	PORTB &= ~(_BV(PB0) | _BV(PB1) | _BV(PB2));
	PORTC &= ~_BV(PC5);	
	PORTD &= ~_BV(PD4);

	// Make DIP switch pins outputs to prevent floating
	DDRB |= _BV(PB0) | _BV(PB1) | _BV(PB2);
	DDRC |= _BV(PC5);
	DDRD |= _BV(PD4);
}


uint8_t readDipSwitch()
{
	uint8_t val = PINB & 0x07;
	val |= 0x08 & (PIND>>1);
	val |= 0x10 & (PINC>>1);
	return val;
}


void createVersionPacket(uint8_t destAddr, uint8_t *buf)
{
	buf[MRBUS_PKT_DEST] = destAddr;
	buf[MRBUS_PKT_SRC] = mrbus_dev_addr;
	buf[MRBUS_PKT_LEN] = 16;
	buf[MRBUS_PKT_TYPE] = 'v';
	buf[6]  = MRBUS_VERSION_WIRELESS;
	// Software Revision
	buf[7]  = 0xFF & ((uint32_t)(GIT_REV))>>16; // Software Revision
	buf[8]  = 0xFF & ((uint32_t)(GIT_REV))>>8; // Software Revision
	buf[9]  = 0xFF & (GIT_REV); // Software Revision
	buf[10]  = HWREV_MAJOR; // Hardware Major Revision
	buf[11]  = HWREV_MINOR; // Hardware Minor Revision
	buf[12] = 'S';
	buf[13] = 'A';
	buf[14] = 'T';
	buf[15] = ' ';
}


void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0x6C;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	TIMSK0 |= _BV(OCIE0A);
}

typedef enum
{
	LED_OFF = 0x00,
	LED_GREEN,
	LED_RED,
	LED_YELLOW,
	LED_GREEN_SLOWBLINK,
	LED_GREEN_FASTBLINK,
	LED_RED_SLOWBLINK,
	LED_RED_FASTBLINK,
	LED_YELLOW_SLOWBLINK,
	LED_YELLOW_FASTBLINK

} LEDStatus;

volatile LEDStatus led;

volatile uint8_t sleepTimer;

inline void ledGreenOff()
{
	PORTD &= ~_BV(LED_GREEN_PD);
}
inline void ledGreenOn()
{
	PORTD |= _BV(LED_GREEN_PD);
}
inline void ledRedOff()
{
	PORTD &= ~_BV(LED_RED_PD);
}
inline void ledRedOn()
{
	PORTD |= _BV(LED_RED_PD);
}

#define STATUS_READ_SWITCHES 0x01


void ledUpdate()
{
	static uint8_t ledPhase = 0;
	switch(++ledPhase)
	{
		case 1:
		case 3:
			switch(led)
			{
				case LED_GREEN_FASTBLINK:
					ledGreenOn();
					break;
		
				case LED_RED_FASTBLINK:			
					ledRedOn();
					break;

				case LED_OFF:
					ledRedOff();
					ledGreenOff();
				default:
					break;
			}
			break;

		case 2:
			switch(led)
			{
				case LED_GREEN:
				case LED_GREEN_SLOWBLINK:
					ledGreenOn();
					break;
				case LED_GREEN_FASTBLINK:
					ledGreenOff();
					break;
				case LED_RED:
				case LED_RED_SLOWBLINK:
					ledRedOn();
					break;
				case LED_RED_FASTBLINK:			
					ledRedOff();
					break;
				default:
					break;

			}
			break;

		case 6:
			switch(led)
			{
				case LED_GREEN_SLOWBLINK:
					ledGreenOn();
					break;
				case LED_RED_SLOWBLINK:
					ledRedOn();
					break;
				default:
					break;

			}
			break;

		case 4:
		case 8:
			switch(led)
			{
				case LED_GREEN:
				case LED_GREEN_SLOWBLINK:
				case LED_GREEN_FASTBLINK:
					ledGreenOff();
					break;

				case LED_RED:
				case LED_RED_SLOWBLINK:
				case LED_RED_FASTBLINK:			
					ledRedOff();
					break;
				default:
					break;
					
			}
			break;

		case 10:
			ledGreenOff();
			ledRedOff();
			ledPhase = 0;
			break;

	}
}

ISR(TIMER0_COMPA_vect)
{
	static uint16_t internalDeciSecs = 0;

	status |= STATUS_READ_SWITCHES;

	if (++ticks >= 10)
	{
		ticks = 0;
		decisecs++;
		internalDeciSecs++;
		
		if (pktTimeout)
			pktTimeout--;
		
		if (internalDeciSecs >= 600)
		{
			// Things that happen on minutes
			if (sleepTimer != 0)
				sleepTimer--;
			internalDeciSecs -= 600;
		}

		ledUpdate();

	}
}

// End of 100Hz timer

volatile uint8_t throttlePot = 0;
volatile uint8_t batteryVoltage = 0;

typedef enum
{
	ADC_STATE_READ_THROTTLE  = 0,
	ADC_STATE_START_VBATT    = 1,
	ADC_STATE_READ_VBATT     = 2,
	ADC_STATE_START_THROTTLE = 3
} ADCState;

ISR(ADC_vect)
{
	static uint16_t accumulator=0;
	static uint8_t count=0;
	static ADCState state=0;

	accumulator += ADC;
	
	switch(state)
	{
		case ADC_STATE_START_VBATT:
			// Throw away the first conversion, as it starts
			// with the wrong mux settings and therefore will
			// actually be reading the throttle pot
			accumulator = 0;
			count = 0;
			state = ADC_STATE_READ_VBATT;
			return;

		case ADC_STATE_START_THROTTLE:
			// Throw away the first conversion, as it starts
			// with the wrong mux settings and therefore will
			// actually be reading the battery
			accumulator = 0;
			count = 0;
			state = ADC_STATE_READ_THROTTLE;
			return;

		case ADC_STATE_READ_VBATT:
		case ADC_STATE_READ_THROTTLE:
			count++;
			break;

		default:
			// Eh, what are we doing here?
			ADMUX  = _BV(REFS0) | 0x07; // AVCC reference, ADC7 channel (throttle pot)
			count = 0;
			state = ADC_STATE_START_THROTTLE;
			break;
	
	}

	if (count >= 64)
	{
		if (ADC_STATE_READ_THROTTLE == state)
		{
			// Measuring throttle pot
			// Divide by 64 samples, and then whack off another 8 to drop it from 10 bits to 7
			throttlePot = accumulator>>9;
			ADMUX  = _BV(REFS0) | 0x06; // AVCC reference, ADC6 channel (battery voltage)
			state = ADC_STATE_START_VBATT;
		
		} else if (ADC_STATE_READ_VBATT == state) {
			// Measuring battery voltage
			// In 20mV increments
			accumulator >>= 6; // Divide by 64 - get the average measurement
			batteryVoltage = (uint8_t)((accumulator * 5) / 31);
			ADMUX  = _BV(REFS0) | 0x07; // AVCC reference, ADC7 channel (throttle pot)
			state = ADC_STATE_START_THROTTLE;

		}
		accumulator = 0;
		count = 0;
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
		createVersionPacket(rxBuffer[MRBUS_PKT_SRC], txBuffer);
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

	pktTimeout = 0;

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
	
	// We've read it, just go ahead and kill the address switch now to save power
	disableAddressSwitch();
	
	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) 
		| (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);

	if (0xFFFF == update_decisecs)
	{
		// It's uninitialized - go ahead and set it to 2 seconds
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, 20);
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, 0);
	}

	// This line assures that update_decisecs is at least 1
	update_decisecs = max(1, update_decisecs);
	// This assures we're firing off a packet at least every 2 seconds.  Greater than that in 
	// a throttle is a bit ridiculous.
	update_decisecs = min(20, update_decisecs);

	// Read the number of minutes before sleeping from EEP and store it.  If it's not set (255)
	// or not sane (0), set it to the default of five minutes.
	
	sleep_tmr_reset_value = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_SLEEP_TIMEOUT);
	if (0xFF == sleep_tmr_reset_value || 0x00 == sleep_tmr_reset_value)
	{
		sleep_tmr_reset_value = DEFAULT_SLEEP_TIME;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}

	// Setup ADC
	ADMUX  = _BV(REFS0) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0); // AVCC reference, ADC7 channel
	ADCSRA = _BV(ADATE) | _BV(ADIF) | _BV(ADPS2) | _BV(ADPS1); // 128 prescaler
	ADCSRB = 0x00;
	throttlePot = 0;
	ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);
}


volatile uint8_t wdt_tripped=0;

ISR(WDT_vect) 
{
	wdt_tripped=1;  // set global volatile variable
}


uint16_t system_sleep(uint16_t sleep_decisecs)
{
	uint16_t slept = 0;

	while(slept < sleep_decisecs)
	{
		uint16_t remaining_sleep = sleep_decisecs - slept;
		uint8_t planned_sleep = 80;
		uint8_t wdtcsr_bits = _BV(WDIF) | _BV(WDIE);

		if (remaining_sleep == 1)
		{
			wdtcsr_bits |= _BV(WDP1) | _BV(WDP0);
			planned_sleep = 1;
		}
		else if (remaining_sleep <= 3)
		{
			wdtcsr_bits |= _BV(WDP2);
			planned_sleep = 3;
		}
		else if (remaining_sleep <= 5)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP0);
			planned_sleep = 5;
		}
		else if (remaining_sleep <= 10)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP1);
			planned_sleep = 10;
		}
		else if (remaining_sleep <= 20)
		{
			wdtcsr_bits |= _BV(WDP2) | _BV(WDP1) | _BV(WDP0);
			planned_sleep = 20;
		}
		else if (remaining_sleep <= 40)
		{
			wdtcsr_bits |= _BV(WDP3);
			planned_sleep = 40;
		}
		else
		{
			wdtcsr_bits |= _BV(WDP3) | _BV(WDP0);
			planned_sleep = 80;
		}

		// Procedure to reset watchdog and set it into interrupt mode only

		cli();
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // set the type of sleep mode to use
		sleep_enable();                           // enable sleep mode
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = wdtcsr_bits;

		sei();

		wdt_tripped = 0;
		// Wrap this in a loop, so we go back to sleep unless the WDT woke us up
		while (0 == wdt_tripped)
			sleep_cpu();

		wdt_reset();
		WDTCSR |= _BV(WDIE); // Restore WDT interrupt mode
		slept += planned_sleep;
	}

	sleep_disable();

#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	MCUSR &= ~(_BV(WDRF));
	WDTCSR |= _BV(WDE) | _BV(WDCE);
	WDTCSR = _BV(WDE) | _BV(WDP2) | _BV(WDP1); // Set the WDT to system reset and 1s timeout
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	return(slept);
}

void setActivePortDirections()
{
	// Set XBee sleep control as output
	DDRD |= _BV(XBEE_SLEEP_PD);

	// Set LED pins as outputs	
	DDRD |= _BV(LED_GREEN_PD) | _BV(LED_RED_PD);

	// Set direction toggle common as output
	DDRC |= _BV(THROTTLE_DIR_COMMON_PC);

	// Set pot bottom as output
	DDRC |= _BV(THROTTLE_POT_ENABLE_PC);
	
	// Set aux button and direction switch sides as an input
	DDRC &= ~(_BV(THROTTLE_AUX_BUTTON_PC) | _BV(THROTTLE_DIR_FORWARD_PC) | _BV(THROTTLE_DIR_REVERSE_PC));
	
	// Ground the bottom of the pot
	PORTC &= ~_BV(THROTTLE_POT_ENABLE_PC);

	// Enable direction switch and aux switch pullups
	PORTC |= _BV(THROTTLE_DIR_REVERSE_PC) | _BV(THROTTLE_DIR_FORWARD_PC) | _BV(THROTTLE_AUX_BUTTON_PC);
	
	// Drive /RTS low
	PORTD &= ~(_BV(MRBEE_RTS));
	DDRD |= _BV(MRBEE_RTS);
}

void setSleepPortDirections()
{
	// Kill both LEDs just in case
	ledGreenOff();
	ledRedOff();

	// Set the aux button monitor to ground and then as an output
	PORTC &= ~_BV(THROTTLE_AUX_BUTTON_PC);
	DDRC |= _BV(THROTTLE_AUX_BUTTON_PC);

	// Raise bottom of pot to VCC to kill current drain
	PORTC |= _BV(THROTTLE_POT_ENABLE_PC);

	// Drive /RTS high (pull-up in XBee?)
	PORTD |= _BV(MRBEE_RTS);
}

void setXbeeSleep()
{
	PORTD |= _BV(XBEE_SLEEP_PD);
}

void setXbeeActive()
{
	// Unsleep the XBee
	PORTD &= ~_BV(XBEE_SLEEP_PD);		
}


#define MRBUS_TX_BUFFER_DEPTH 4
#define MRBUS_RX_BUFFER_DEPTH 4

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t debounce(uint8_t debouncedState, uint8_t newInputs)
{
	static uint8_t clock_A=0, clock_B=0;
	uint8_t delta = newInputs ^ debouncedState;   //Find all of the changes
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;

	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.

	changes = ~((~delta) | clock_A | clock_B);
	debouncedState ^= changes;
	return(debouncedState);
}

int main(void)
{
	uint8_t lastThrottlePot = 0;	
	uint8_t dir=0;
	uint8_t funcButtons = 0, lastFuncButtons = 0;
	
	// Application initialization
	setActivePortDirections();
	enableAddressSwitch();
	init();
	setXbeeActive();
	sleepTimer = sleep_tmr_reset_value;

	led = LED_OFF;

	// Initialize a 100 Hz timer.
	initialize100HzTimer();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	sei();	

	{
		// Fire off initial reset version packet
		uint8_t txBuffer[MRBUS_BUFFER_SIZE];
		createVersionPacket(0xFF, txBuffer);
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
	}



	while (1)
	{
		wdt_reset();

		if (status & STATUS_READ_SWITCHES)
		{
			status &= ~STATUS_READ_SWITCHES;
			funcButtons = debounce(funcButtons, PINC & (THROTTLE_DIR_MASK | _BV(THROTTLE_AUX_BUTTON_PC)));
		}

		switch (funcButtons & THROTTLE_DIR_MASK)
		{
			case THROTTLE_DIR_FORWARD:
				dir = 1;
				sleepTimer = sleep_tmr_reset_value;
				break;
			case THROTTLE_DIR_REVERSE:
				dir = 2;
				sleepTimer = sleep_tmr_reset_value;
				break;
			case THROTTLE_DIR_IDLE:
			default:
				dir = 0;
				break;
		}
		
		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
		{
			pktTimeout = 100;
			PktHandler();
		}
		
		if (batteryVoltage >= (VBATT_OKAY/2))
		{
			if (0 == pktTimeout)
				led = LED_GREEN;
			else
				led = LED_GREEN_FASTBLINK;
		}
		else if (batteryVoltage >= (VBATT_WARN/2))
			led = LED_RED_FASTBLINK;
		else
			led = LED_RED;

		
		// Transmission criteria...
		// > 2 decisec from last transmission and...
		// current throttle reading is not the same as before and the direction is non-idle
		// the function buttons changed
		// *****  or *****
		// it's been more than the transmission timeout
		
		if ((((((throttlePot != lastThrottlePot) && (0 != dir)) || funcButtons != lastFuncButtons) && decisecs > 1) || (decisecs >= update_decisecs))
				&& !(mrbusPktQueueFull(&mrbeeTxQueue)))
		{
			uint8_t txBuffer[MRBUS_BUFFER_SIZE];

			lastThrottlePot = throttlePot;
			lastFuncButtons = funcButtons;
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_DEST] = 0xFF;
			txBuffer[MRBUS_PKT_LEN] = 10;
			txBuffer[5] = 'C';
			txBuffer[6] = dir;
			// Don't send a speed if we're in neutral
			if (0 == dir)
				txBuffer[7] = 0;
			else
				txBuffer[7] = lastThrottlePot;
			txBuffer[8] = ((funcButtons>>1) & 0x01) ^ 0x01;
			txBuffer[9] = batteryVoltage;	
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
			decisecs = 0;
		}
			
		if (mrbusPktQueueDepth(&mrbeeTxQueue))
		{
			mrbeeTransmit();
		}

		while (0 == sleepTimer)
		{
			// Time to nod off
			led = LED_OFF;
			// Disable internal power-sucking peripherals
			ADCSRA &= ~_BV(ADEN);
	
			setXbeeSleep();
			setSleepPortDirections();

			while (THROTTLE_DIR_IDLE == (PINC & THROTTLE_DIR_MASK))
				system_sleep(10);

			sleepTimer = sleep_tmr_reset_value;
			
			// Re-enable chip internal bits (ADC, etc.)
			ADCSRA |= _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADIF);

			setActivePortDirections();
			setXbeeActive();
		}
	}
}



