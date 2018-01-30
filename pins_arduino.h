/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/
  Copyright (c) 2007 David A. Mellis
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

/* Modified by Grant Hilgert for use with Mizzou Zoumobot
 based off ATMEGA169PA/329PA series MCU
 */



#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            70
#define NUM_ANALOG_INPUTS           6
#define analogInputToDigitalPin(p)  ((p < 16) ? (p) + 54 : -1)
#define digitalPinHasPWM(p)         (((p) >= 2 && (p) <= 13) || ((p) >= 44 && (p)<= 46))

#define PIN_SPI_SS    (30)
#define PIN_SPI_MOSI  (27)
#define PIN_SPI_MISO  (28)
#define PIN_SPI_SCK   (29)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (12)
#define PIN_WIRE_SCL        (13)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define LED_BUILTIN 13

#define PIN_A0   (7)
#define PIN_A1   (8)
#define PIN_A2   (9)
#define PIN_A3   (10)
#define PIN_A4   (11)
#define PIN_A5   (12)


static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;

#define PIN_D0   (1)
#define PIN_D1   (2)
#define PIN_D2   (3)
#define PIN_D3   (4)
#define PIN_D4   (5)
#define PIN_D5   (6)


static const uint8_t D0 = PIN_D0;
static const uint8_t D1 = PIN_D1;
static const uint8_t D2 = PIN_D2;
static const uint8_t D3 = PIN_D3;
static const uint8_t D4 = PIN_D4;
static const uint8_t D5 = PIN_D5;


// A majority of the pins are NOT PCINTs, SO BE WARNED (i.e. you cannot use them as receive pins)
// Only pins available for RECEIVE (TRANSMIT can be on any pin):
// (I've deliberately left out pin mapping to the Hardware USARTs - seems senseless to me)
// Pins: 10, 11, 12, 13,  50, 51, 52, 53,  62, 63, 64, 65, 66, 67, 68, 69

#define digitalPinToPCICR(p)    ( (((p) >= 10) && ((p) <= 13)) || \
                                  (((p) >= 50) && ((p) <= 53)) || \
                                  (((p) >= 62) && ((p) <= 69)) ? (&PCICR) : ((uint8_t *)0) )

#define digitalPinToPCICRbit(p) ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 50) && ((p) <= 53)) ? 0 : \
                                ( (((p) >= 62) && ((p) <= 69)) ? 2 : \
                                0 ) )

#define digitalPinToPCMSK(p)    ( (((p) >= 10) && ((p) <= 13)) || (((p) >= 50) && ((p) <= 53)) ? (&PCMSK0) : \
                                ( (((p) >= 62) && ((p) <= 69)) ? (&PCMSK2) : \
                                ((uint8_t *)0) ) )

#define digitalPinToPCMSKbit(p) ( (((p) >= 10) && ((p) <= 13)) ? ((p) - 6) : \
                                ( ((p) == 50) ? 3 : \
                                ( ((p) == 51) ? 2 : \
                                ( ((p) == 52) ? 1 : \
                                ( ((p) == 53) ? 0 : \
                                ( (((p) >= 62) && ((p) <= 69)) ? ((p) - 62) : \
                                0 ) ) ) ) ) )

#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : ((p) >= 18 && (p) <= 21 ? 23 - (p) : NOT_AN_INTERRUPT)))

#ifdef ARDUINO_MAIN

const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
	(uint16_t) &DDRG,
	
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
	(uint16_t) &PORTG,

};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PIN,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
	(uint16_t) &PING,

};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	// PORTLIST		
	// -------------------------------------------		
	PA	, // PA3 D0	
	PA	, // PA2 D1	
	PA	, // PA1 D2
	PA	, // PA0 D3	
	PF	, // PF7 D4	
	PF	, // PF6 D5	
	PF	, // PF0 A0	
	PF	, // PF1 A1
	PF	, // PF2 A2	
	PF	, // PF3 A3	
	PF	, // PF4 A4	
	PF	, // PF5 A5	
	PE	, // PE5 SDA	
	PE	, // PE4 SCL	
	PA	, // PA7 AUX LED	
	PB	, // PB4 PWM1	
	PB	, // PB5 PWM2	
	PB	, // PB6 PWM3	
	PB	, // PB7 PWM4	
	PG	, // PG4 BIN1A	
	PG	, // PG3 BIN1B	
	PD	, // PD2 BIN2A	
	PD	, // PD3 BIN2B	
	PD	, // PD5 BIN3A	
	PD	, // PD4 BIN3B
	PD	, // PD6 BIN4A
	PD	, // PD7 BIN4B	
	PB	, // PB3 SPI_MOSI	
	PB	, // PB2 SPI_MISO
	PB	, // PB1 SPI_SCK	
	PB	, // PB0 SPI_SS
	PE	, // PE3 DIP1

};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	// PIN IN PORT		
	// -------------------------------------------		
	
	_BV( 3 )	, // PA3 D0	
	_BV( 2 )	, // PA2 D1	
	_BV( 1 )	, // PA1 D2
	_BV( 0 )	, // PA0 D3	
	_BV( 7 )	, // PF7 D4	
	_BV( 6 )	, // PF6 D5	
	_BV( 0 )	, // PF0 A0	
	_BV( 1 )	, // PF1 A1
	_BV( 2 )	, // PF2 A2	
	_BV( 3 )	, // PF3 A3	
	_BV( 4 )	, // PF4 A4	
	_BV( 5 )	, // PF5 A5	
	_BV( 5 )	, // PE5 SDA	
	_BV( 4 )	, // PE4 SCL	
	_BV( 7 )	, // PA7 AUX LED	
	_BV( 4 )	, // PB4 PWM1	
	_BV( 5 )	, // PB5 PWM2	
	_BV( 6 )	, // PB6 PWM3	
	_BV( 7 )	, // PB7 PWM4	
	_BV( 4 )	, // PG4 BIN1A	
	_BV( 3 )	, // PG3 BIN1B	
	_BV( 2 )	, // PD2 BIN2A	
	_BV( 3 )	, // PD3 BIN2B	
	_BV( 5 )	, // PD5 BIN3A	
	_BV( 4 )	, // PD4 BIN3B
	_BV( 6 )	, // PD6 BIN4A
	_BV( 7 )	, // PD7 BIN4B	
	_BV( 3 )	, // PB3 SPI_MOSI	
	_BV( 2 )	, // PB2 SPI_MISO
	_BV( 1 )	, // PB1 SPI_SCK	
	_BV( 0 )	, // PB0 SPI_SS
	_BV( 3 )	, // PE3 DIP1
		
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	// TIMERS		
	// -------------------------------------------		
	NOT_ON_TIMER	, // PA3 D0	
	NOT_ON_TIMER	, // PA2 D1	
	NOT_ON_TIMER	, // PA1 D2
	NOT_ON_TIMER	, // PA0 D3	
	NOT_ON_TIMER	, // PF7 D4	
	NOT_ON_TIMER	, // PF6 D5	
	NOT_ON_TIMER	, // PF0 A0	
	NOT_ON_TIMER	, // PF1 A1
	NOT_ON_TIMER	, // PF2 A2	
	NOT_ON_TIMER	, // PF3 A3	
	NOT_ON_TIMER	, // PF4 A4	
	NOT_ON_TIMER	, // PF5 A5	
	NOT_ON_TIMER	, // PE5 SDA	
	NOT_ON_TIMER	, // PE4 SCL	
	NOT_ON_TIMER	, // PA7 AUX LED	
	TIMER0A	, // PB4 PWM1	
	TIMER1A	, // PB5 PWM2	
	TIMER1B	, // PB6 PWM3	
	TIMER2A	, // PB7 PWM4	
	NOT_ON_TIMER	, // PG4 BIN1A	
	NOT_ON_TIMER	, // PG3 BIN1B	
	NOT_ON_TIMER	, // PD2 BIN2A	
	NOT_ON_TIMER	, // PD3 BIN2B	
	NOT_ON_TIMER	, // PD5 BIN3A	
	NOT_ON_TIMER	, // PD4 BIN3B
	NOT_ON_TIMER	, // PD6 BIN4A
	NOT_ON_TIMER	, // PD7 BIN4B	
	NOT_ON_TIMER	, // PB3 SPI_MOSI	
	NOT_ON_TIMER	, // PB2 SPI_MISO
	NOT_ON_TIMER	, // PB1 SPI_SCK	
	NOT_ON_TIMER	, // PB0 SPI_SS
	NOT_ON_TIMER	, // PE3 DIP1
	
	
};

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial

#endif
