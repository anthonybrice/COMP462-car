// main.c
// Runs on MSP432
// Squarewave on P7.3 using TimerA0 TA0.CCR0
// PWM on P2.4 using TimerA0 TA0.CCR1
// PWM on P2.5 using TimerA0 TA0.CCR2
// MCLK = SMCLK = 3MHz DCO; ACLK = 32.768kHz
// TACCR0 generates a square wave of freq ACLK/1024 =32Hz
// Derived from msp432p401_portmap_01.c in MSPware
// Jonathan Valvano
// July 24, 2015

/* This example accompanies the book
   "Embedded Systems: Interfacing to the MSP432 Microcontroller",
   ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2015
   Volume 2, Program 9.8

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

#include <stdint.h>
#include <stdlib.h>
#include "msp432p401r.h"
#include "ClockSystem.h"
#include "SysTickInts.h"
#include "InputCapture.h"
#include "PWM.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void logic_init(void);
void encoder_init(void);
void period_measure_init(void);
void period_measure(uint16_t time);

// 50Hz squarewave on P7.3
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// P2.5=1 when timer equals TA0CCR2 on way down, P2.5=0 when timer equals TA0CCR2 on way up
//uint32_t leftCount = 0;
//uint32_t rightCount = 0;
uint16_t leftPeriod;
uint16_t first;
int done;

void period_measure(uint16_t time) {
	P2OUT = P2OUT^0x02;              // toggle P2.1
  leftPeriod = (time - first)&0xFFFF;  // 16 bits, 83.3 ns resolution
  first = time;                    // setup for next
  done = 1;
}

int main(void){int i; uint16_t duty;
	//SysTick_Init(10);
	//Clock_Init48MHz();
  PWM_Init(15000,11250,3750); //while(1){};
  // Squarewave Period/4 (20ms)
  // P2.4 CCR1 75% PWM duty cycle, 10ms period
  // P2.5 CCR2 25% PWM duty cycle, 10ms period
  //PWM_Init(10,7,2);//   while(1){};
  // Squarewave Period/4 (20us)
  // P2.4 CCR1 66% PWM duty cycle, 13.3us period
  // P2.5 CCR2 33% PWM duty cycle, 10us period
	
	logic_init();
	encoder_init();
//	
//	period_measure_init();
//	
//	P2SEL0 &= ~0x07;
//  P2SEL1 &= ~0x07;                 // configure built-in RGB LEDs as GPIO
//  P2DS |= 0x07;                    // make built-in RGB LEDs high drive strength
//  P2DIR |= 0x07;                   // make built-in RGB LEDs out
//  P2OUT &= ~0x07;                  // RGB = off
	
	TimerCapture_Init(&period_measure);
//	
//	PWM_Duty1(10000);
//	PWM_Duty2(10000);
  while (1){
		WaitForInterrupt();
    for (duty = 50; duty<15000; duty = duty+1000) {
      PWM_Duty1(duty);
      PWM_Duty2(duty);
			
			// read encoders
//			if (P4IN & 0x01) rightCount++;
//			if (P4IN & 0x02) leftCount++;
			
      for (i = 1000000; i ;i--);
    }
  }
}

void logic_init() {
	// configure p3.2-3 & p3.6-7 as GPIO
	P3SEL0 &= ~0xcc;
	P3SEL1 &= ~0xcc;
	
	// make p3.2-3 & P3.6-7 output
	P3DIR |= 0xcc;
	
	// turn on 3.2 & 3.6 and turn off 3.3 & 3.7
	P3OUT |= 0x44;
	P3OUT &= ~0x88;
}

void period_measure_init() {
	first = 0;
	done = 0;
}

void encoder_init() {
	// configure  4.0-1 as GPIO
	P4SEL0 &= ~0x03;
	P4SEL1 &= ~0x03;
	
	// make P4.0-1 input
	P4DIR &= ~0x03;
}




































