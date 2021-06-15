/*
 * Timer.c
 *
 * Created: 25-03-2021 22:43:18
 *  Author: Jan
 */ 



#include "Timer.h"


//Initialize hardware 16-bit timer 1
void init_timer1(){
	TCCR1B |= (1<<CS11) | (1<<CS10);	//Prescaler: 64
	TCCR1B |= (1<<WGM12);				//Mode: CTC
	OCR1A =	12;							//Compare value 16-bit
	OCR1B =	12;							//Compare value 16-bit
	TIMSK1 |= (1<<OCIE1B);				//Interrupt when TCNT1 = OCR3B
}
	
//Initialize hardware timer 3
void init_timer3(){
	TCCR3B |= (1<<CS31) | (1<<CS30);	//Prescaler /64
	TCCR3B |= (1<<WGM32);				//Mode: CTC
	OCR3A =	249;						//Compare value
	TIMSK3 |= (1<<OCIE3A);				//Interrupt when TCNT3 = OCR3A
}

//Setup Fast PWM on 8-bit timer 0
void initFastPWM(){
	DDRB |= (1<<DDB7);	//set OC0A pin as output
	TCCR0A |= (1<<COM0A1) | (1<<WGM01) | (1<<WGM00);	//select fast PWM non-inverting mode
	TCCR0B |= (1<<CS01);	//pre-scaler set to 8	7,84kHz
	TCNT0 = 0;	//force TCNT0 to count from 0
	OCR0A = 0;	//0% duty cycle
}

//Setup Phase Correct PWM on 8-bit timer 0
void initPhaseCorrPWM(){
	DDRB |= (1<<DDB7);	//set OC0A pin as output
	TCCR0A |= (1<<COM0A1) | (1<<WGM00);	//select Phase correct PWM mode
	TCCR0B |= (1<<CS01);	//pre-scaler set to 8	(3,92kHz)
	TCNT0 = 0;	//force TCNT0 to count from 0
	OCR0A = 0;	//start with 0% duty cycle
}

//Setup Phase Frequency Correct PWM on 16-bit timer 5. (using timer 5 to avoid conflict with timer1 used by ADC "Auto trigger mode")
void initPhaseFreqCorrPWM(){
	DDRL |= (1<<DDL3);					//SET OC5A PIN AS OUTPUT (PIN46)
	TCCR5A |= (1<<COM5A1);				//CLEAR ON UPCOUNTING SET ON DOWNCOUNTING
	TCCR5B |= (1<<CS51) | (1<<WGM53);	//PRE-SCALER SET TO 8 (3,92kHz) AND PHASE FREQ MODE SELECT
	ICR5 = 255;							//ICR1 HOLDS TOP VALUE SO OC1A PIN CAN BE USED.
	OCR5A = 0;							//start with 0% duty cycle
}

//Reset timer control registers for PWM. Fx when selecting new pwm timer
void resetPWMTimers(){
	TCCR0A = 0;
	TCCR0B = 0;
	TCCR5A = 0;
	TCCR5B= 0;
}