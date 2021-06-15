/*
 * ADC.c
 *
 * Created: 21-04-2021 13:58:47
 *  Author: Jan
 */ 

#include "ADC.h"

//Initialize ADC with 125kHz input clock
void init_adc(char interrupt){
	ADCSRA |= (1<<ADPS2);						//1MHz input clock to ADC (div factor 16)
	ADCSRB |= (1<<ADTS2) | (1<<ADTS0);			//Auto Trigger source: (Timer/Counter1 Compare Match B)
	ADCSRA |= (1<<ADATE);						//Enable auto trigger mode
	ADCSRA |= (1<<ADEN);						//Enable ADC
	if(interrupt) ADCSRA |= (1<<ADIE);			//Enable interrupt
}

//Returns 10-bit sampled value from ADC.
unsigned int get_sample(char channel){
	ADMUX |= channel;		//Select ADC channel
	ADMUX |= (1<<REFS0);	//Select AVCC as voltage reference
	
	DIDR0 = (1<<channel);
	DIDR0 = ~DIDR0;
	DIDR1 = 0xFF;			//Disable DIDR1
	
	ADCSRA |= (1<<ADSC);	//Start ADC sampling
	
	while(!(ADCSRA & (1<<ADIF)));
	
	return (ADCL + (ADCH<<8));
}

//Initiates ADC on specified channel and starts sampling. Used with interrupt
void startADCSampling(char channel){
	ADMUX = channel;		//Select ADC channel
	//Using external 3.3V reference on AREF	
	ADMUX |= (1<<ADLAR);	//Left adjust ADC conversion result, since only ADCH is read because only 8-bit precision is needed	 
	DIDR0 = (1<<channel);	//Disabling unused inputs to save power
	DIDR0 = ~DIDR0;
	DIDR1 = 0xFF;			//Disable DIDR1
	DIDR2 = 0xFF;			//Disable DIDR2
	//ADCSRA |= (1<<ADSC);	//Start ADC sampling. (Not needed when using Auto Trigger mode)
	
}


//Format ADC 10-bit sample to decimal value with 2 decimal places.	
void formatADCSample(int sample, char * buffer){
	
	char Vref = 10;		//3,3V * 3 =9,9. Searching for whole number. Divider should be mult. by 3 as well. Side 280 i datablad
	int div = 1024*3;
	
	unsigned int voltage = (Vref*sample/div); 
	buffer[0]=voltage+0x30;					//ascii for the integer
	unsigned int frac =(Vref*sample)%div;
	buffer[1]='.';
	buffer[2]=Vref*frac/div+0x30;
	frac = (Vref*frac)%div;
	buffer[3]= (Vref*frac)/div +0x30;
	frac = (Vref*frac)%div;
	if(frac>=1023)
	buffer[3]++;
	if(buffer[3]==(0x30+10)) {
		buffer[3]=0x30;
		buffer[2]++;
		if(buffer[2]==(0x30+10)) {
			buffer[2]=0x30;
			buffer[0]++;
		}
	}
}