// ClockSystemTestMain.c
// Runs on MSP432
// Test the Clock System initialization to verify that the
// system clock is running at the expected rate.  Use the
// debugger if possible or an oscilloscope connected to P2.2.
// When using an oscilloscope to look at LED2, it should be
// clear to see that the LED flashes about 4 (12/3) times
// faster with a 12 MHz clock than with the default 3 MHz
// clock.
// The operation of the Clock System can be tested even more
// precisely by using an oscilloscope to measure P4.3, which
// is configured to output the main clock signal.  The main
// clock is used by the CPU and peripheral module interfaces
// and can be used directly by some peripheral modules.  In
// this case and by default, this is the DCO frequency
// divided by 1.  P4.2 is configured to output the auxiliary
// clock signal.  The auxiliary clock is used by individual
// peripheral modules that select it.  In this case, this is
// the internal, low-power low-frequency oscillator REFOCLK,
// which is 32,768 Hz in this case and by default.  REFOCLK
// can be 128,000 Hz by setting bit 15 of CSCLKEN register.
// In this case and by default, the auxiliary clock is
// divided by 1.  P4.4 is configured to output the subsystem
// master clock signal.  The subsystem master clock (HSMCLK)
// and low-speed subsystem master clock (SMCLK) get their
// clocks from the same source.  By default, this is the DCO
// frequency divided by 1 for both (although both the HSMCLK
// and SMCLK can be programmed with different dividers).
// Both subsystem master clocks are used by individual
// peripheral modules that select them.
// Daniel Valvano
// June 30, 2015

/* This example accompanies the book
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
   ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2015
   Program 4.6

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// auxiliary clock output connected to P4.2
// main clock output connected to P4.3
// subsystem master clock output connected to P4.4

#include <stdint.h>
#include <math.h>

#include "ClockSystem.h"
#include "msp.h"

#define CALIBRATION	0

// delay function for testing
// which delays about 8.1*ulCount cycles
#ifdef __TI_COMPILER_VERSION__
  //Code Composer Studio Code
  void Delay(unsigned long ulCount){
  __asm (  "dloop:   subs    r0, #1\n"
      "    bne     dloop\n");
}

#else
  //Keil uVision Code
  __asm void
  Delay(unsigned long ulCount)
  {
    subs    r0, #1
    bne     Delay
    bx      lr
  }

#endif
  
  
#define SCALE 16
	
void ResistanceMeasure_Init(void);
double ResistanceMeasure(void);
double calculateDistance(uint32_t);
	
// test
// DCO1_5MHz  SCALE=1/2 blue LED flashs 1Hz
// DCO3MHz    SCALE=1   blue LED flashs 1Hz
// DCO6MHz    SCALE=2   blue LED flashs 1Hz
// DCO12MHz   SCALE=4   blue LED flashs 1Hz
// Init48MHz  SCALE=16  blue LED flashs 1Hz
int main(void){
//  Clock_Init(DCO12MHz);        // configure for 12 MHz clock
  Clock_Init48MHz();           // configure for 48 MHz clock
//  Clock_Init32kHz();           // configure for 32 kHz clock
	ResistanceMeasure_Init();
	
	double distance;
	while (1) {
		distance = ResistanceMeasure();
		double bar = distance;
	}
}

void ResistanceMeasure_Init(void) {
	P2SEL0 &= ~0x10; // configure P2.4 as GPIO 	TODO change me
	P2SEL1 &= ~0x10;
	P2DIR |= 0x10;
	P2OUT &= ~0x10; 	// P2.4 init LOW
//	P7SEL0 |= 0x08;
//	P7SEL1 &= ~0x08;
//	P7DIR &= ~0x08;
//	TA0CTL &= ~0x0030;
//	TA0CTL = 0x0200;
//	TA0EX0 &= ~0x0007;
	P2SEL0 |= 0x20;
	P2SEL1 &= ~0x20;
	P2DIR &= ~0x20;
	TA0CTL &= ~0x0030;
	TA0CTL = 0x0200;
	TA0EX0 &= ~0x0007; 
}

double ResistanceMeasure(void) {
	uint16_t rising;
	TA0CTL &= ~0x0030;
	TA0CCTL2 = 0x4900;
	TA0CTL |= 0x0024;
	P2OUT |= 0x10;  // reset P2.4 HIGH
	Delay(55);
	P2OUT &= ~0x10; 	// P2.4 LOW
	while ((TA0CCTL2 & 0x0001) == 0);
	rising = TA0CCR2;
	TA0CCTL2 = 0x8900;
	while ((TA0CCTL2 & 0x0001) == 0);
	
	uint32_t bar = TA0CCR2 - rising - CALIBRATION;
	double baz = calculateDistance(bar);
	double box = baz;
	
	return baz;
}

double calculateDistance(uint32_t x) {
	double ns = x * pow(10, -9);
	
	return (340 * ns) * 50;
}


























