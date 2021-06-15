/*
 * ADC.h
 *
 * Created: 21-04-2021 13:59:06
 *  Author: Jan
 */ 


#ifndef ADC_H_
#define ADC_H_


#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

// ================================================
// Functions
// ================================================
void init_adc(char interrupt);
unsigned int get_sample(char channel);
void startADCSampling(char channel);
void formatADCSample(int sample, char * buffer);

#endif /* ADC_H_ */