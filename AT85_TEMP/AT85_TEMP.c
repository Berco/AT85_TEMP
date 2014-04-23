/*
 * AT85_TEMP.c
 *
 * Created: 20-4-2014 14:57:28
 *  Author: Berco
 */


#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <string.h>
#include "1wire.h"

int16_t ds18b20ReadTemperature(uint8_t bus, uint8_t * id);

#define DS18B20_FAMILY_ID                0x28
#define DS18B20_START_CONVERSION         0x44
#define DS18B20_READ_SCRATCHPAD          0xbe
#define DS18B20_ERROR                    -1000

#define BUS 							ONEWIRE_PIN_3
#define MAX_DEVICES 					4
#define SENDER_PIN						0x10;

#define MAX_REPEATS						20

uint8_t OOK_bit = 0;
uint8_t MAX_OOK;
uint8_t SEND_bit = 11;
uint8_t SEND_shift = 11;
uint16_t SHIFT_MASK = 0x80;
uint8_t REPEAT;

static uint8_t ID		= 0b01011111; //(0-7) ID, 95 in this case
static uint8_t INFO		= 0b00000000;//0b00111000; //(8-13 used) INFO first bit is battery.
uint16_t TEMP			= 0b0000001111111111; //(22-31 used) TEMP 0-1023.
static uint8_t HUMI		= 0b01101110;//0b11111111; //(32-39) HUMIDITY first bit is 65, 7 is 1.
static uint8_t FOOTER	= 0b10111000; //(40-44) footer

void init_Timer_OOK(void){
	/* Using this 8-bit timer to get OOK Modulation.
	 * WGM01 is set to work in CTC mode,
	 * OCIE0A to enable OCR0A interrupts.
	 * CS01 and CS00 gives a prescaler of 64, meaning an effective clock frequency of
	 * 8M/64 = 125 kHz. With OCR0A set to
	 * 66 this means an interrupt every 1/125k = 8 us --> (67 + 1)*8 = 544 us. */
	TCCR0A = 1<<WGM01;
	TIMSK |= 1<<OCIE0A;
	OCR0A=67;
	TCCR0B |= (1<<CS01 | 1<<CS00);
}

int main (void)
{
	signed int temp = 0;
	cli();
		DDRB |= SENDER_PIN; // PB4 for the sender or Control LED
		DDRB &= ~(0x08); //PB3 as input
		static oneWireDevice devices[MAX_DEVICES];
		oneWireDevice *ds18b20;
		init_Timer_OOK();
		oneWireInit(BUS);
		while (oneWireSearchBuses(devices, MAX_DEVICES, BUS) != ONEWIRE_SEARCH_COMPLETE);
		ds18b20 = oneWireFindFamily(DS18B20_FAMILY_ID, devices, MAX_DEVICES);
	sei();

	while (1) {
		if (ds18b20 != NULL){
			temp 	= ds18b20ReadTemperature(ds18b20->bus, ds18b20->id);
			uint16_t fraction	= 625* (temp & 0x0f);
			TEMP = (temp >> 4)*10+(fraction/1000);
			REPEAT = MAX_REPEATS;
		}
		// change this for a sleep with watchdog function.
		_delay_ms(20000);
	}
  return 1;
}

ISR(TIMER0_COMPA_vect){
	uint16_t PACKET=1;
	if (SEND_bit < 8) PACKET = ID;
	if (SEND_bit >= 8 && SEND_bit < 16) PACKET = INFO;
	if (SEND_bit >= 16 && SEND_bit < 32) PACKET = TEMP;
	if (SEND_bit >= 32 && SEND_bit < 40) PACKET = HUMI;
	if (SEND_bit >= 40) PACKET = FOOTER;

	if ((OOK_bit == 0) && REPEAT){
		PORTB |= SENDER_PIN; // HIGH during first OOK
		//Get the value of the bit to be sent
		if (PACKET & (SHIFT_MASK>>SEND_shift)) {
			MAX_OOK = 9;
		} else {
			MAX_OOK = 4;
		}

		SEND_bit++;
		SEND_shift++;
		if (SEND_bit == 8) SEND_shift = 0;
		if (SEND_bit == 14) {
			SEND_shift = 6;
			SEND_bit = 22;
			SHIFT_MASK=0x8000;
		}
		if (SEND_bit == 32) {
			SEND_shift = 0;
			SHIFT_MASK = 0x80;
		}
		if (SEND_bit == 40) SEND_shift = 0;
		if (SEND_bit == 45){ //one extra as footer
			REPEAT--;
			SEND_shift = 0;
			SEND_bit = 0;
			MAX_OOK=18;
		}
	}else {
		PORTB &= ~SENDER_PIN;
	}
	OOK_bit++;
	if (OOK_bit == MAX_OOK) OOK_bit = 0;

}


int16_t ds18b20ReadTemperature(uint8_t bus, uint8_t * id)
{
	int16_t temperature;

	// Reset, presence.
	if (!oneWireDetectPresence(bus))
	return DS18B20_ERROR;

	// Match the id found earlier.
	oneWireMatchRom(id, bus);

	// Send start conversion command.
	oneWireSendByte(DS18B20_START_CONVERSION, bus);

	// Wait until conversion is finished.
	// Bus line is held low until conversion is finished.

	#ifdef ONEWIRE_USE_PARASITIC_POWER
	ONEWIRE_RELEASE_BUS(bus);
	_delay_ms(850);

	#else
	while (!oneWireReadBit(bus))
	;
	#endif
	// Reset, presence.
	if(!oneWireDetectPresence(bus))
	return DS18B20_ERROR;

	// Match id again.
	oneWireMatchRom(id, bus);

	// Send READ SCRATCHPAD command.
	oneWireSendByte(DS18B20_READ_SCRATCHPAD, bus);

	// Read only two first bytes (temperature low, temperature high)
	// and place them in the 16 bit temperature variable.
	temperature = oneWireReceiveByte(bus);
	temperature |= (oneWireReceiveByte(bus) << 8);

	return temperature;
}