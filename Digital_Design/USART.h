/*
 * USART.h
 *
 * Created: 11-03-2021 17:06:13
 *  Author: Jan
 */ 


#ifndef USART_H_
#define USART_H_
// ================================================
// Includes
// ================================================
#include <avr/io.h>


// ================================================
// Defines/macros
// ================================================
#define BAUD	115200
#define UBBR_S	F_CPU/16/BAUD-1		//Half Duplex single speed
#define UBBR_D	F_CPU/8/BAUD-1		//Half Duplex double speed


// ================================================
// Functions
// ================================================
extern void init_uart(unsigned int  ubrr);
extern void init_uart_interrupt(unsigned int  ubrr);
extern void init_uart_interrupt1(unsigned int  ubrr);
extern char getCharUSART(void);
extern void putCharUSART(char tx);
extern void transmitStrUSART(char * ptr);
extern int receiveStrUSART(char * buffer);
extern void USART_Flush();

#endif /* USART_H_ */