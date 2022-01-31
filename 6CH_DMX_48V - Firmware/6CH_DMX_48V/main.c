/* 
Project Name:			6CH_DMX_48V
Date:					24.04.2020 - 7.06.2021
Version:				1.0
Author:					Penkov Alexander
Language:				C

MCU:					Atmega48PA-AU
Voltage:				5v
Clock Speed:			16MHz

Fuses:					Bit		Low					High					Extended				Lock bit
						=========================================================================================
						7		CKDIV8	=	1		RSTDISBL	=	1		Bit 7		=	1		Bit 7	=	1
						6		CKOUT	=	1		DWEN		=	1		Bit 6		=	1		Bit 6	=	1
						5		SUT1	=	1		SPIEN		=	0		Bit 5		=	1		Bit 5	=	1
						4		SUT0	=	1		WDTON		=	1		Bit 4		=	1		Bit 4	=	1
						3		CKSEL3	=	1		EESAVE		=	1		Bit 3		=	1		Bit 3	=	1
						2		CKSEL2	=	1		BODLEVEL2	=	1		Bit 2		=	1		Bit 2	=	1
						1		CKSEL1	=	1		BODLEVEL1	=	1		Bit 1		=	1		Bit 1	=	1
						0		CKSEL0	=	1		BODLEVEL0	=	1		SELFPRGEN	=	1		Bit 1	=	1
						
Output PWM channels:

						OC2B (PD3) - 1 CH
						OC0B (PD5) - 2 CH
						OC0A (PD6) - 3 CH
						OC1A (PB1) - 4 CH
						OC1B (PB2) - 5 CH
						OC2A (PB3) - 6 CH
						
Jumpers:
						JUMP1	JUMP2	JUMP3	JUMP4
						======================================
						---------Output Frequency-------------
						  1		  0						122 HZ
						  0		  1						490 HZ
						  0		  0						3921 Hz
						-----------Output inversion-----------
										  0				NORMAL OUTPUT
										  1				INVERSE OUTPUT
						-------------Output curve-------------
													0	LINEAR CURVE
													1	LOGARITHMIC CURVE

Features:
=========

	- 6 Channel DMX Receiver with PWM output
	- Change settings via DMX input
	- Linear/Logarithmic output curve
	- PWM inverse

BREAK signal detection principle
================================

	UART allocates byte without the last stop bits (Frame Error - FE flag) and program interprets it as a BREAK pulse.
	Device recognizes a BREAK pulse >38 us (8.5 bit UART).

Set settings via DMX
====================
	
	To send settings data via DMX use alternate START code 0x91.
	After START code send manufacturer id, command and 2 byte checksum.
	Command contain command type and settings data slots.
	Depending on the value of the command type, certain data slots are used, the rest are ignored.

	For Example:
	
	BREAK signal
	Byte 1		0x91		Command start code after Brake
	Byte 2		0x7F		Manufacturer id High byte
	Byte 3		0xFF		Manufacturer id Low byte
	Byte 4		0x01		Command type 0x01 - change DMX address
	Byte 5		0x00		Address DMX High byte
	Byte 6		0x01		Address DMX Low byte
	Byte 7		0x01		Checksum High byte
	Byte 8		0x83		Checksum Low byte
	
	Use all bytes except the start code to calculate the checksum
	Checksum = 0x7F + 0xFF + 0x01 + 0x00 + 0x01 = 0x0183
	


Copyright
=========

	- Copyright ©2020 Penkov Alexander. All rights reserved. <penkov187@gmail.com>

License
=======

 */ 

#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

#define		DMX_NUM_CHANNELS			6								// Sets the size of the receive buffer
#define		DMX_START_ADDRESS			1								// Default DMX address
#define		DMX_MIN_ADDRESS				1								// For check valid DMX address
#define		DMX_MAX_ADDRESS				512								// For check valid DMX address
#define		DMX_DATA_START_CODE			0x00							// Start code for the DMX data
#define		DMX_COMMAND_START_CODE		0x91							// Start code for the command data
#define		MANUFACTURER_ID_HI			0x7F							// Manufacturer ID
#define		MANUFACTURER_ID_LO			0xFF							//



//- structure command data packet

enum {
	MFID_HI,
	MFID_LO,
	COMMAND_TYPE,
	START_ADDR_HI,
	START_ADDR_LO,
	DMX_COMMAND_LENGTH													// *Trick for automatic calculation of command length
	};

//- Current state of receiving DMX

enum {
	_DMX_IDLE,															// Wait for the next BREAK
	_DMX_BREAK,															// Received a BREAK: now a new packet will START code
	_DMX_START_CODE,													// Check START code
	_DMX_START,															// Skip received DMX data until start DMX address
	_DMX_RUN,															// Receiving DMX data into the dmx_data buffer
	_DMX_COMMAND,														// Receiving command packet into the dmx_command buffer
	_DMX_COMMAND_CHECKSUMH,												// Receiving checksum H
	_DMX_COMMAND_CHECKSUML												// Receiving checksum L and verify command
	};
	
//- Variables

unsigned char			dmx_state = _DMX_IDLE;							// The current state of the DMX receive algorithm
unsigned char			timer_clk_prescaling;
unsigned char			timer2_clk_prescaling;

unsigned int			dmx_start_addr = DMX_START_ADDRESS;
unsigned int			temp_start_addr;								
EEMEM unsigned int		eep_dmx_start_addr;								// Pointer on EEPROM
unsigned int			dmx_slot_number;

unsigned char			dmx_data[DMX_NUM_CHANNELS];						// Dimmers data buffer
unsigned char			chan_count;										// Index in dmx_data array
unsigned char			dmx_command[DMX_COMMAND_LENGTH];				// Command data buffer
unsigned int			dmx_command_checksum = 0;
unsigned char			dmx_command_available = 0;						// True if command received

unsigned char			log_table[256] = {	0,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											1,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											2,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											3,
											4,
											4,
											4,
											4,
											4,
											4,
											4,
											4,
											4,
											4,
											4,
											4,
											5,
											5,
											5,
											5,
											5,
											5,
											5,
											5,
											5,
											6,
											6,
											6,
											6,
											6,
											6,
											6,
											6,
											7,
											7,
											7,
											7,
											7,
											7,
											8,
											8,
											8,
											8,
											8,
											8,
											9,
											9,
											9,
											9,
											9,
											10,
											10,
											10,
											10,
											10,
											11,
											11,
											11,
											11,
											12,
											12,
											12,
											12,
											13,
											13,
											13,
											14,
											14,
											14,
											14,
											15,
											15,
											15,
											16,
											16,
											17,
											17,
											17,
											18,
											18,
											18,
											19,
											19,
											20,
											20,
											21,
											21,
											21,
											22,
											22,
											23,
											23,
											24,
											24,
											25,
											26,
											26,
											27,
											27,
											28,
											28,
											29,
											30,
											30,
											31,
											32,
											32,
											33,
											34,
											35,
											35,
											36,
											37,
											38,
											39,
											39,
											40,
											41,
											42,
											43,
											44,
											45,
											46,
											47,
											48,
											49,
											50,
											51,
											52,
											53,
											55,
											56,
											57,
											58,
											60,
											61,
											62,
											64,
											65,
											66,
											68,
											69,
											71,
											72,
											74,
											76,
											77,
											79,
											81,
											83,
											84,
											86,
											88,
											90,
											92,
											94,
											96,
											98,
											100,
											103,
											105,
											107,
											109,
											112,
											114,
											117,
											119,
											122,
											125,
											127,
											130,
											133,
											136,
											139,
											142,
											145,
											148,
											152,
											155,
											158,
											162,
											165,
											169,
											173,
											177,
											180,
											184,
											189,
											193,
											197,
											201,
											206,
											210,
											215,
											219,
											224,
											229,
											234,
											239,
											245,
											250,
											255};

ISR(USART_RX_vect)
{
	unsigned char status = UCSR0A;										// Get state before data!
	unsigned char data = UDR0;											// Get data

	if (status & (1 << FE0)){											// Check for BREAK
		dmx_state = _DMX_BREAK;											// BREAK condition detected
		dmx_slot_number = 0;		
	}	
	
	if (dmx_slot_number > DMX_MAX_ADDRESS) {							// Protection dmx_slot_number overflow
		dmx_state = _DMX_IDLE;											
	}
	
	switch (dmx_state){
		case _DMX_IDLE:													// Wait for next BREAK
		break;
		
		case _DMX_BREAK:
			dmx_state = _DMX_START_CODE;								// The next data byte is the start byte
		break;
		
		case _DMX_START_CODE:
			if (data == DMX_DATA_START_CODE){
				dmx_state = _DMX_START;									// DMX data start code detected
			}
			else if (data == DMX_COMMAND_START_CODE){
				dmx_state = _DMX_COMMAND;								// Command start code detected
				dmx_command_checksum = 0;
			}
			else{
				dmx_state = _DMX_IDLE;									// Wait next BREAK if other start codes
			}
		break;
		
		case _DMX_START:
			dmx_slot_number++;
			if (dmx_slot_number == dmx_start_addr){  
				chan_count = 0;											// Now receive first channel
				dmx_data[chan_count++] = data;							// Store received data into DMX data buffer.
				dmx_state = _DMX_RUN;
			}
		break;
		
		case _DMX_RUN:
			dmx_slot_number++;
			dmx_data[chan_count++] = data;								// Store received data into DMX data buffer.
			if (chan_count >= DMX_NUM_CHANNELS){						// All channels done.
				dmx_state = _DMX_IDLE;									// Wait for next BREAK
			}
		break;
		
		case _DMX_COMMAND:
			dmx_command[dmx_slot_number++] = data;
			dmx_command_checksum += data;
			if (dmx_slot_number == DMX_COMMAND_LENGTH){
				dmx_state = _DMX_COMMAND_CHECKSUMH;						// All data received. Now getting checksum!
			}
		break;
		
		case _DMX_COMMAND_CHECKSUMH:
			dmx_command_checksum -= data << 8;
			dmx_state = _DMX_COMMAND_CHECKSUML;
		break;
		
		case _DMX_COMMAND_CHECKSUML:
			dmx_command_checksum -= data;
			if ((dmx_command_checksum == 0) && (dmx_command[MFID_HI] == MANUFACTURER_ID_HI) && (dmx_command[MFID_LO] == MANUFACTURER_ID_LO)){
				dmx_command_available = 1;
			}
			dmx_state = _DMX_IDLE;
		break;	
			
		default:
			dmx_state = _DMX_IDLE;
		break;
	}
}

int main(void)
{
	
	temp_start_addr = eeprom_read_word(&eep_dmx_start_addr);										// Read DMX address from EEPROM
	
	// Is address in EEPROM valid?
	// If EEPROM is empty, loaded value will be 0, then DMX address check fails - default DMX address from definition is used.
	
	if ((temp_start_addr >= DMX_MIN_ADDRESS) && (temp_start_addr <= DMX_MAX_ADDRESS)){
		dmx_start_addr = temp_start_addr;															// Load DMX address from EEPROM
	}
	
    // Port init
	DDRB = (1 << 3) | (1 << 2) | (1 << 1);															// OC2A (PB3), OC1B (PB2), OC1A (PB1)
	DDRD = (1 << 6) | (1 << 5) | (1 << 3) | (1 << 4);												// OC0A (PD6), OC0B (PD5), OC2B (PD3), DE (PD4)
	PORTC = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5);									// Pull-Up Jumpers
	PORTD &= ~( 1 << 4 );																			// DE is low - receiver enable
	
	
	unsigned char read_jumpers_state;
	read_jumpers_state = (PINC >> 1) & 3;															// 3 is mask 0b00000011
	
	switch (read_jumpers_state){
		case 3:
			timer_clk_prescaling = 2;
			timer2_clk_prescaling = 2;
		break;
		case 2:
			timer_clk_prescaling = 3;
			timer2_clk_prescaling = 4;
		break;
		case 1:
			timer_clk_prescaling = 4;
			timer2_clk_prescaling = 6;
		break;
		default:
			timer_clk_prescaling = 2;
			timer2_clk_prescaling = 2;
		break;		
	}
	
		
	// 8-bit Timer/Counter 0 init
	if (PINC & (1 << 0)) {
		TCCR0A |= (1 << COM0A1) | (1 << COM0A0);
		TCCR0A |= (1 << COM0B1) | (1 << COM0B0);
	}else{
		TCCR0A |= (1 << COM0A1);
		TCCR0A |= (1 << COM0B1);
	}
		
	TCCR0A |= (1 << WGM00);																			// Phase correct PWM
	TCCR0B |= timer_clk_prescaling;

	// 16-bit Timer/Counter 1 init
	if (PINC & (1<<0)) {
		TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
		TCCR1A |= (1 << COM1B1) | (1 << COM1B0);
	}else{
		TCCR1A |= (1 << COM1A1);
		TCCR1A |= (1 << COM1B1);
	}
		
	TCCR1A |= (1 << WGM10);																			// Phase correct PWM 8 bit
	TCCR1B |= timer_clk_prescaling;
	
	// 8-bit Timer/Counter 2 init
	if (PINC & (1<<0)) {
		TCCR2A |= (1 << COM2A1) | (1 << COM2A0);
		TCCR2A |= (1 << COM2B1) | (1 << COM2B0);
	}else{
		TCCR2A |= (1 << COM2A1);
		TCCR2A |= (1 << COM2B1);
	}
	
	TCCR2A |= (1 << WGM20);																			// Phase correct PWM	
	TCCR2B |= timer2_clk_prescaling;		
		
	// UART init
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0);															// RX Complete Interrupt Enable , Receiver Enable
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (1 << USBS0);											// 8N2
	UBRR0L = 0x03;																					// 250 000 bps

	sei();																							// Enable interrupts

    while (1) 
    {
		// Change PWM value
		if (PINC & (1<<3)) {
		OCR2B = dmx_data[0];
		OCR0B = dmx_data[1];
		OCR0A = dmx_data[2];
		OCR1AL= dmx_data[3];
		OCR1BL= dmx_data[4];
		OCR2A = dmx_data[5];
		}else{
		OCR2B = log_table[dmx_data[0]];
		OCR0B = log_table[dmx_data[1]];
		OCR0A = log_table[dmx_data[2]];
		OCR1AL= log_table[dmx_data[3]];
		OCR1BL= log_table[dmx_data[4]];
		OCR2A = log_table[dmx_data[5]];
		}
		
		// Commands parsing
		if (dmx_command_available){
			dmx_command_available = 0;
			
			switch (dmx_command[COMMAND_TYPE]){
				case 0x01:																					// Command to change DMX address
					temp_start_addr = (dmx_command[START_ADDR_HI] << 8) | dmx_command[START_ADDR_LO];		// Extract DMX address from received command data slots
			
					if ((temp_start_addr >= DMX_MIN_ADDRESS) && (temp_start_addr <= DMX_MAX_ADDRESS)){		// Validation received DMX address
						dmx_start_addr = temp_start_addr;													// Apply new DMX address
						eeprom_update_word(&eep_dmx_start_addr, temp_start_addr);							// Store in EEPROM
					}				
				break;
			}

		}	
    }
}

