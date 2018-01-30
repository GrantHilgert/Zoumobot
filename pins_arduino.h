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

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

#define NUM_DIGITAL_PINS            70
#define NUM_ANALOG_INPUTS           6
#define analogInputToDigitalPin(p)  ((p < 16) ? (p) + 54 : -1)
#define digitalPinHasPWM(p)         (((p) >= 2 && (p) <= 13) || ((p) >= 44 && (p)<= 46))

#define PIN_SPI_SS    (53)
#define PIN_SPI_MOSI  (51)
#define PIN_SPI_MISO  (50)
#define PIN_SPI_SCK   (52)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_WIRE_SDA        (20)
#define PIN_WIRE_SCL        (21)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define LED_BUILTIN 13

#define PIN_A0   (54)
#define PIN_A1   (55)
#define PIN_A2   (56)
#define PIN_A3   (57)
#define PIN_A4   (58)
#define PIN_A5   (59)


static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;

#define PIN_D0   (54)
#define PIN_D1   (55)
#define PIN_D2   (56)
#define PIN_D3   (57)
#define PIN_D4   (58)
#define PIN_D5   (59)


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
	NOT_ON_TIMER	, // PE 0 ** 0 ** USART0_RX	
	NOT_ON_TIMER	, // PE 1 ** 1 ** USART0_TX	
	TIMER3B	, // PE 4 ** 2 ** PWM2	
	TIMER3C	, // PE 5 ** 3 ** PWM3	
	TIMER0B	, // PG 5 ** 4 ** PWM4	
	TIMER3A	, // PE 3 ** 5 ** PWM5	
	TIMER4A	, // PH 3 ** 6 ** PWM6	
	TIMER4B	, // PH 4 ** 7 ** PWM7	
	TIMER4C	, // PH 5 ** 8 ** PWM8	
	TIMER2B	, // PH 6 ** 9 ** PWM9	
	TIMER2A	, // PB 4 ** 10 ** PWM10	
	TIMER1A	, // PB 5 ** 11 ** PWM11	
	TIMER1B	, // PB 6 ** 12 ** PWM12	
	TIMER0A	, // PB 7 ** 13 ** PWM13	
	NOT_ON_TIMER	, // PJ 1 ** 14 ** USART3_TX	
	NOT_ON_TIMER	, // PJ 0 ** 15 ** USART3_RX	
	NOT_ON_TIMER	, // PH 1 ** 16 ** USART2_TX	
	NOT_ON_TIMER	, // PH 0 ** 17 ** USART2_RX	
	NOT_ON_TIMER	, // PD 3 ** 18 ** USART1_TX	
	NOT_ON_TIMER	, // PD 2 ** 19 ** USART1_RX	
	NOT_ON_TIMER	, // PD 1 ** 20 ** I2C_SDA	
	NOT_ON_TIMER	, // PD 0 ** 21 ** I2C_SCL	
	NOT_ON_TIMER	, // PA 0 ** 22 ** D22	
	NOT_ON_TIMER	, // PA 1 ** 23 ** D23	
	NOT_ON_TIMER	, // PA 2 ** 24 ** D24	
	NOT_ON_TIMER	, // PA 3 ** 25 ** D25	
	NOT_ON_TIMER	, // PA 4 ** 26 ** D26	
	NOT_ON_TIMER	, // PA 5 ** 27 ** D27	
	NOT_ON_TIMER	, // PA 6 ** 28 ** D28	
	NOT_ON_TIMER	, // PA 7 ** 29 ** D29	
	NOT_ON_TIMER	, // PC 7 ** 30 ** D30	
	NOT_ON_TIMER	, // PC 6 ** 31 ** D31	
	NOT_ON_TIMER	, // PC 5 ** 32 ** D32	
	NOT_ON_TIMER	, // PC 4 ** 33 ** D33	
	NOT_ON_TIMER	, // PC 3 ** 34 ** D34	
	NOT_ON_TIMER	, // PC 2 ** 35 ** D35	
	NOT_ON_TIMER	, // PC 1 ** 36 ** D36	
	NOT_ON_TIMER	, // PC 0 ** 37 ** D37	
	NOT_ON_TIMER	, // PD 7 ** 38 ** D38	
	NOT_ON_TIMER	, // PG 2 ** 39 ** D39	
	NOT_ON_TIMER	, // PG 1 ** 40 ** D40	
	NOT_ON_TIMER	, // PG 0 ** 41 ** D41	
	NOT_ON_TIMER	, // PL 7 ** 42 ** D42	
	NOT_ON_TIMER	, // PL 6 ** 43 ** D43	
	TIMER5C	, // PL 5 ** 44 ** D44	
	TIMER5B	, // PL 4 ** 45 ** D45	
	TIMER5A	, // PL 3 ** 46 ** D46	
	NOT_ON_TIMER	, // PL 2 ** 47 ** D47	
	NOT_ON_TIMER	, // PL 1 ** 48 ** D48	
	NOT_ON_TIMER	, // PL 0 ** 49 ** D49	
	NOT_ON_TIMER	, // PB 3 ** 50 ** SPI_MISO	
	NOT_ON_TIMER	, // PB 2 ** 51 ** SPI_MOSI	
	NOT_ON_TIMER	, // PB 1 ** 52 ** SPI_SCK	
	NOT_ON_TIMER	, // PB 0 ** 53 ** SPI_SS	
	NOT_ON_TIMER	, // PF 0 ** 54 ** A0	
	NOT_ON_TIMER	, // PF 1 ** 55 ** A1	
	NOT_ON_TIMER	, // PF 2 ** 56 ** A2	
	NOT_ON_TIMER	, // PF 3 ** 57 ** A3	
	NOT_ON_TIMER	, // PF 4 ** 58 ** A4	
	NOT_ON_TIMER	, // PF 5 ** 59 ** A5	
	NOT_ON_TIMER	, // PF 6 ** 60 ** A6	
	NOT_ON_TIMER	, // PF 7 ** 61 ** A7	
	NOT_ON_TIMER	, // PK 0 ** 62 ** A8	
	NOT_ON_TIMER	, // PK 1 ** 63 ** A9	
	NOT_ON_TIMER	, // PK 2 ** 64 ** A10	
	NOT_ON_TIMER	, // PK 3 ** 65 ** A11	
	NOT_ON_TIMER	, // PK 4 ** 66 ** A12	
	NOT_ON_TIMER	, // PK 5 ** 67 ** A13	
	NOT_ON_TIMER	, // PK 6 ** 68 ** A14	
	NOT_ON_TIMER	, // PK 7 ** 69 ** A15	
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
#define SERIAL_PORT_HARDWARE1       Serial1
#define SERIAL_PORT_HARDWARE2       Serial2
#define SERIAL_PORT_HARDWARE3       Serial3
#define SERIAL_PORT_HARDWARE_OPEN   Serial1
#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
#define SERIAL_PORT_HARDWARE_OPEN2  Serial3

#endif
