/*
 * SPI.h
 *
 * Created: 25-02-2021 16:45:29
 *  Author: Jan & Lars
 */ 

#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>


extern void init_spi_master();
extern void init_spi_slave();
extern void transmit_spi_master(unsigned char cx);
extern unsigned char receive_spi_slave(void);
extern void transmit_Spi_pkg(char *data, char size);


#endif /* SPI_H_ */