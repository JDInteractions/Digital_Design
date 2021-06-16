/*
 * SPI.c
 *
 * Created: 25-02-2021 16:45:00
 *  Author: BJUS
 */ 

#include "SPI.h"


void init_spi_master(){
	DDRB |= (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<SPR1); //SPI-MODE 0, sample rising setup falling.
}


void init_spi_slave(){
	DDRB |= (1<<DDB3);
	SPCR |= (1<<SPE) | (1<<CPOL);
	PORTB |= (1<<PB0);							//Enable pull-up
}


void transmit_spi_master(unsigned char cx){
	PORTB &= ~(1<<PB0);							//Select slave
	SPDR = cx;									//Write char to data register
	while(!(SPSR & (1<<SPIF)));					//Wait for data to be shifted out
	PORTB |= (1<<PB0);							//De-select slave
}


unsigned char receive_spi_slave(void){
	while(!(SPSR & (1<<SPIF)));					//Wait for a byte to shift in
	return SPDR;
}


void transmit_Spi_pkg(char *data, char size){	//Transmit full SPI-datapackage
	for(int i =0; i<size;i++){
		transmit_spi_master(data[i]);
	}
}
