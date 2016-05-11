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

//#include <stdint.h>
//#include <stdlib.h>
#include <math.h>


#include "msp432p401r.h"
//#include "UART.h"
#include "ClockSystem.h"
#include "SysTickInts.h"
#include "InputCapture.h"
#include "PWM.h"

#include "Nokia5110.h"

#define CALIBRATION	0
#define DELTA_T 1

#define SIZE 200
uint16_t leftWheelBuf[SIZE];
uint16_t rightWheelBuf[SIZE];
uint32_t count = 0;

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

void logic_init(void);
void encoder_init(void);
void period_measure_init(void);
void left_period_measure(uint16_t time);
void right_period_measure(uint16_t time);
uint16_t left_pid_controller(uint16_t y, uint16_t r);
uint16_t right_pid_controller(uint16_t y, uint16_t r);
void print_debug(void);
void save_feedback(uint16_t left, uint16_t right);

void sonar1_init(void);
void sonar2_init(void);
double sonar1_measure(void);
double sonar2_measure(void);
double calculateDistance(uint32_t x);

// delay function for testing from sysctl.c
// which delays 3*ulCount cycles
#ifdef __TI_COMPILER_VERSION__
  //Code Composer Studio Code
  void Delay(uint32_t ulCount){
  __asm (  "dloop:   subs    r0, #1\n"
      "    bne     dloop\n");
}

#else
  //Keil uVision Code
  __asm void
  Delay(uint32_t ulCount)
  {
    subs    r0, #1
    bne     Delay
    bx      lr
  }
#endif

// 50Hz squarewave on P7.3
// P2.4=1 when timer equals TA0CCR1 on way down, P2.4=0 when timer equals TA0CCR1 on way up
// P2.5=1 when timer equals TA0CCR2 on way down, P2.5=0 when timer equals TA0CCR2 on way up
//uint32_t leftCount = 0;
//uint32_t rightCount = 0;
uint16_t leftPeriod;
uint16_t leftFirst;
int leftDone;
uint16_t leftCount = 0;

uint16_t rightPeriod;
uint16_t rightFirst;
int rightDone;
uint16_t rightCount = 0;
	
	
// VELOCITY CONSTANT
int v = 1.0;
	
const int D = 10; // desired distance from right wall during wall following
const int R = 3; // wheel radius in cm
const int L = 13; // wheel base in cm
const int KPe = 1.0; // distance error constant

void left_period_measure(uint16_t time) {
	P2OUT = P2OUT^0x02;              // toggle P2.1
  leftPeriod = (time - leftFirst)&0xFFFF;  // 16 bits, 83.3 ns resolution
  leftFirst = time;                    // setup for next
  leftDone = 1;
	
//	if (P2OUT & 0x02) 
//	{
//		leftCount++;
//	}
}

void right_period_measure(uint16_t time) {
	P3OUT = P3OUT^0x02;              // toggle P3.1
  rightPeriod = (time - rightFirst)&0xFFFF;  // 16 bits, 83.3 ns resolution
  rightFirst = time;                    // setup for next
  rightDone = 1;
	
//	if (P3OUT & 0x02) 
//	{
//		rightCount++;
//	}
}



int main(void) {
	//SysTick_Init(10);
	Clock_Init48MHz();
  PWM_Init(15000,7500,7500); //while(1){};
  // Squarewave Period/4 (20ms)
  // P2.4 CCR1 75% PWM duty cycle, 10ms period
  // P2.5 CCR2 25% PWM duty cycle, 10ms period
  //PWM_Init(10,7,2);//   while(1){};
  // Squarewave Period/4 (20us)
  // P2.4 CCR1 66% PWM duty cycle, 13.3us period
  // P2.5 CCR2 33% PWM duty cycle, 10us period
	
	Nokia5110_Init();
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
	
	TimerCapture1_Init(&left_period_measure, &right_period_measure);
	//TimerCapture2_Init(&right_period_measure);
	EnableInterrupts();
//	
//	PWM_Duty1(10000);
//	PWM_Duty2(10000);
  while (1){

		uint16_t left_u = left_pid_controller(leftPeriod, 10000);
		uint16_t right_u = right_pid_controller(rightPeriod,10000);
		save_feedback(left_u, right_u);
		
		
		// TODO
		// pingValue = ping(right_sensor)
		
		// pingValCentimeters = microsecondsToCentimeters(pingValue);
		
		// error_d = D - pingValCentimeters;
		
		
		// w = KPe * error_d
		
		// DESIRED VELOCITIES
		// vr = (2v + wL) / 2R
		// vl = (2v - wL) / 2R
		
		//
		
		PWM_Duty1(left_u);
		PWM_Duty2(right_u);
		
		Delay(DELTA_T);
		//print_debug();
		
		if (count == SIZE) break;
  }
	
//	for (int i = 0; i < SIZE; i++) {
//		printf("%d,", leftWheelBuf[i]);
//	}
//	printf("\n");
//	for (int i = 0; i < SIZE; i++) {
//		printf("%d, ", rightWheelBuf[i]);
//	}
//	printf("\n");
}

void print_debug() {
	Nokia5110_Clear();
	Nokia5110_OutString("LPrd:  ");
	Nokia5110_OutUDec(leftPeriod);
	//Nokia5110_OutString("            ");
	Nokia5110_OutString("RPrd:  ");
	Nokia5110_OutUDec(rightPeriod);
	
	Nokia5110_OutString("            ");
	
	Nokia5110_OutString("LCnt:  ");
	Nokia5110_OutUDec(leftCount);
	Nokia5110_OutString("RCnt:  ");
	Nokia5110_OutUDec(rightCount);
}

uint16_t right_e_old = 0;
uint16_t right_E = 0;
uint16_t right_pid_controller(uint16_t y, uint16_t r) {
	uint16_t e = r - y;
	uint16_t e_dot = (e - right_e_old) / DELTA_T;
	
	right_E += e;
	uint16_t u = e + right_E + e_dot;
	right_e_old = e;
	
	return u;
}

uint16_t left_e_old = 0;
uint16_t left_E = 0;
uint16_t left_pid_controller(uint16_t y, uint16_t r) {
	uint16_t e = r - y;
	uint16_t e_dot = (e - left_e_old) / DELTA_T;
	
	left_E += e;
	uint16_t u = e + left_E + e_dot;
	left_e_old = e;
	
	return u;
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
	leftFirst = 0;
	leftDone = 0;
	rightFirst = 0;
	rightDone = 0;
	
	leftPeriod = 0;
	rightPeriod = 0;
}

void encoder_init() {
	// configure  4.0-1 as GPIO
	P4SEL0 &= ~0x03;
	P4SEL1 &= ~0x03;
	
	// make P4.0-1 input
	P4DIR &= ~0x03;
}

void save_feedback(uint16_t left, uint16_t right) {
	if (count < SIZE) {
		leftWheelBuf[count] = left;
		rightWheelBuf[count] = right;
		count++;
	}
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void sonar1_init(void) {
	P5SEL0 &= ~0x40;
	P5SEL1 &= ~0x40;
	P5DIR |= 0x40;
	P5OUT |= 0x40; // make P5.6 out and initially high
	
	// TODO init input line
	
	TA2CTL &= ~0x0030;
	TA2CTL = 0x0200;
	TA2EX0 &= ~0x0007;
}

void sonar2_init(void) {
	P10SEL0 &= ~0x10;
	P10SEL1 &= ~0x10;
	P10DIR |= 0x10;
	P10OUT |= 0x10;  // make P10.4 out and initially high

	// TODO init input line
	
	TA3CTL &= ~0x0030;
	TA3CTL = 0x0200;
	TA3EX0 &= ~0x0007;
}

double calculateDistance(uint32_t x) {
	double ns = x * pow(10, -9);
	
	return (340 * ns) * 50;
}

double sonar1_measure(void) {
	uint16_t rising;
	TA2CTL &= ~0x0030;
	TA2CCTL1 = 0x4900;
	TA2CTL |= 0x0024;
	P2OUT |= 0x10;  // reset P2.4 HIGH
	Delay(55);
	P2OUT &= ~0x10; 	// P2.4 LOW
	while ((TA2CCTL1 & 0x0001) == 0);
	rising = TA2CCR1;
	TA2CCTL1 = 0x8900;
	while ((TA2CCTL1 & 0x0001) == 0);
	
	uint32_t bar = TA2CCR1 - rising - CALIBRATION;
	double baz = calculateDistance(bar);
	double box = baz;
	
	return baz;
}

double sonar2_measure(void) {
	uint16_t rising;
	TA3CTL &= ~0x0030;
	TA3CCTL0 = 0x4900;
	TA3CTL |= 0x0024;
	P2OUT |= 0x10;  // reset P2.4 HIGH
	Delay(55);
	P2OUT &= ~0x10; 	// P2.4 LOW
	while ((TA3CCTL0 & 0x0001) == 0);
	rising = TA3CCR0;
	TA3CCTL0 = 0x8900;
	while ((TA3CCTL0 & 0x0001) == 0);
	
	uint32_t bar = TA3CCR0 - rising - CALIBRATION;
	double baz = calculateDistance(bar);
	double box = baz;
	
	return baz;
}

































