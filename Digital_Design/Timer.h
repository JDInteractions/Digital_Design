/*
 * Timer.h
 *
 * Created: 25-03-2021 22:43:29
 *  Author: Jan
 */ 


#ifndef TIMER_H_
#define TIMER_H_
// ================================================
// Includes
// ================================================
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

// ================================================
// Defines/macros
// ================================================
#define SETBIT(ADDR, BIT)(ADDR |= (1<<BIT))
#define CLRBIT(ADDR, BIT)(ADDR &= ~(1<<BIT))
#define CHKB IT(ADDR, BIT)(ADDR & (1<<BIT))
// ================================================
// Functions
// ================================================

void init_timer1();
void init_timer3();
void initFastPWM();
void initPhaseCorrPWM();
void initPhaseFreqCorrPWM();
void resetPWMTimers();


#endif /* TIMER_H_ */