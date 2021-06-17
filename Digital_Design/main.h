/*
 * main.h
 *
 * Created: 07-06-2021 13:28:22
 *  Author: Jan
 */ 

#ifndef DEVEL
#define DEVEL 1
#endif

#define F_CPU 16000000UL
#define BAUD_EFFECT 11520UL

#ifndef MAIN_H_
#define MAIN_H_

// ================================================
// Includes
// ================================================
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>
#include <string.h>
#include "USART.h"
#include "ADC.h"
#include "Timer.h"


#include "I2C.h"
#include "ssd1306.h"

#include "SPI.h"
// ================================================
// Defines/macros
// ================================================
#define SETBIT(ADDR, BIT)(ADDR |= (1<<BIT))
#define CLRBIT(ADDR, BIT)(ADDR &= ~(1<<BIT))
#define CHKBIT(ADDR, BIT)(ADDR & (1<<BIT))
#define TOGGLEBIT(ADDR,BIT)(ADDR ^= (1<<BIT))

#define HEADER_SIZE	5
#define FOOTER_SIZE 2
#define PADDING_SIZE	HEADER_SIZE+FOOTER_SIZE

//Telemetry types
#define BTN_TYPE	0x01
#define SEND_TYPE	0x02
#define START_TYPE	0x03

//SPI data

#define RESET_SPI 0x01
#define SPI_DATA_SIZE 4
#define SPI_RESET 0x01
#define SPI_START 0x02
#define SPI_STOP 0x03
#define SPI_SHAPE 0x04
#define SPI_AMP 0x05
#define SPI_FREQ 0x07

//Telecommand types
#define GENERATOR_TYPE	0x01
#define SCOPE_TYPE	0x02
#define BODE_TYPE	0x03
#define GEN_PKG 4
#define BODE_PKG 255

//BTN Type-values
#define ENTER 0X00
#define SELECT 0x01
#define START_STOP 0x02
#define RESET 0x03

//ADC
#define ADC_CHANNEL	0
#define ADC_TRIG_SRC_PS	64
#define MIN_RECORD_LENGTH	47
#define SAMPLE_BUF	1010 

//Checksum
#define CKSUM_TYPE	1	//ZERO16=0  ,  LRC8=1

#define datasize 15

// ================================================
// Functions
// ================================================
void setup();
void transmitUARTPackage(char * data, unsigned char type, unsigned int dataSize);
void transmitADCSample(char * data, unsigned char type, unsigned int dataSize);


void setSampleRate(unsigned int sampleRate);
void readBuffer();

enum tilstande handle_type(char input);
void debug_print(char input, int value);
void handle_generator();
void evaluate_recieve();
void resetLabview();


//Utils
unsigned int calcCheckSum(char * data, unsigned int pkgSize);
unsigned int sampleRate_comp(unsigned int record_length);
void debug_print_char(char input);
void debug_print_int(int input);

#endif /* MAIN_H_ */