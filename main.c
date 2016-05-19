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
#include <stdio.h>
 
#include "msp432p401r.h"
#include "ClockSystem.h"
#include "InputCapture.h"
#include "PWM.h"

#include "Nokia5110.h"
#include "SysTick.h"

#define TRUE (!FALSE)
#define FALSE (0)

#define CALIBRATION	0
#define DELTA_T 360000
//#define DELTA_T 30000

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#define K_P 1
#define K_I 0
#define K_D 0

#define K_PB1 1
#define K_PB2 1
#define K_DB 0
#define K_IB 0

#define K_SCALE 1

#define V_DESIRED 7000
#define TURN_CALBR 7000

#define R 3.0
#define L 13.0

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
int32_t left_pid_controller(uint16_t y, uint16_t r);
int32_t right_pid_controller(uint16_t y, uint16_t r);
void print_debug(void);
void save_feedback(uint16_t left, uint16_t right);

void sonar1_init(void);
void sonar2_init(void);
int32_t sonar1_measure(void);
int32_t sonar2_measure(void);
double calculateDistance(uint32_t x);
void print_sonars(int32_t son1, int32_t son2, int32_t vld, int32_t vrd);
void print_udecs(uint32_t ns1, uint32_t ns2);
void print_strings(char* ns1, char* ns2);
int32_t average(int32_t* arr, uint16_t length);
int32_t average_front(int32_t x);
int32_t average_right(int32_t x);
int32_t pid_controller(int32_t y, int32_t r);
uint16_t scale(uint16_t period);
uint32_t cycles_to_delay_units(uint32_t time);

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
	
typedef struct _Car Car;
	
struct _Car {
	const uint16_t left_velocity;
	const uint16_t right_velocity;
	const uint16_t distance;
};

Car states[3] = {
	{ // straight
		.left_velocity = V_DESIRED,
		.right_velocity = V_DESIRED,
		.distance = 3676
	},
	{ // turn right
		.left_velocity = 11000,
		.right_velocity = 7143,
		.distance = 0.0
	}, 
	{ // turn left
		.left_velocity = 4200,
		.right_velocity = 14000,
		.distance = 0.0
	}
};

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

void left_period_measure(uint16_t time) {
	P2OUT = P2OUT^0x02;              // toggle P2.1
  leftPeriod = (time - leftFirst)&0xFFFF;  // 16 bits, 83.3 ns resolution
  leftFirst = time;                    // setup for next
  leftDone = 1;
}

void right_period_measure(uint16_t time) {
	P3OUT = P3OUT^0x02;              // toggle P3.1
  rightPeriod = (time - rightFirst)&0xFFFF;  // 16 bits, 83.3 ns resolution
  rightFirst = time;                    // setup for next
  rightDone = 1;
}

int main(void) {
	Clock_Init48MHz();
  PWM_Init(15000,0,0); //while(1){};
  // Squarewave Period/4 (20ms)
  // P2.4 CCR1 75% PWM duty cycle, 10ms period
  // P2.5 CCR2 25% PWM duty cycle, 10ms period
  //PWM_Init(10,7,2);//   while(1){};
  // Squarewave Period/4 (20us)
  // P2.4 CCR1 66% PWM duty cycle, 13.3us period
  // P2.5 CCR2 33% PWM duty cycle, 10us period
	
	Nokia5110_Init();
	Nokia5110_Clear();
	logic_init();
	encoder_init();
	sonar1_init();
	sonar2_init();

	period_measure_init();
	
	TimerCapture1_Init(&left_period_measure, &right_period_measure);
	EnableInterrupts();
	
	Car* car = &states[0];
	
  while (TRUE) {
		SysTick_Init(); 
		
//		double sonar1 = average_right(sonar1_measure());
//		double sonar2 = average_front(sonar2_measure());
//		int32_t sonar1 = average_right(sonar1_measure());
//		int32_t sonar2 = average_front(sonar2_measure());
		int32_t sonar1 = sonar1_measure();
		int32_t sonar2 = sonar2_measure();
		
		if (sonar1 == -1) continue;
		
		int32_t ed;
		if (sonar2 >= 0 && sonar2 < 5147) {
			// turn left
			car = &states[2];
			ed = 0;
		} else if (sonar1 >= 7353) {
			// turn right
			car = &states[1]; 
			ed = 0; 
		} else {
			// head straight
			car = &states[0];
			ed = pid_controller(sonar1, car->distance);
		}
		
		int32_t vld = car->left_velocity - ed * TURN_CALBR;
		int32_t vrd = car->right_velocity + ed * TURN_CALBR;
		
		print_sonars(sonar1, sonar2, vld, vrd);
				
		uint16_t lm = scale(leftPeriod);
		uint16_t rm = scale(rightPeriod);
			
		int32_t left_u = left_pid_controller(lm, vld);
		int32_t right_u = right_pid_controller(rm, vrd);
		
		PWM_Duty1(CLAMP((int32_t) vrd + right_u, 0, 14999)); // right
		PWM_Duty2(CLAMP((int32_t) vld + left_u, 0, 14999)); // left
		
		uint32_t delay_time;
		if (0x00010000 & SYSTICK_STCSR) {
			delay_time = 1;
		} else {
			uint32_t run_time = cycles_to_delay_units(0x00ffffff - SYSTICK_STCVR);
			delay_time = DELTA_T - run_time;
		}
		
//		Delay(delay_time);
  }
}

/**
 * 48MHz means 1 cycle is 2.08E-8 seconds.
 * Then multiply by 1E6 to get microseconds.
 * Then multiply by 6 to convert to delay units.
 * Delay(60) == 10 microseconds, since the function is linear, 
 * 60x == 10 ==> x == 1/6 microseconds.
 */
uint32_t cycles_to_delay_units(uint32_t time) {
	return (uint32_t) ((time * 12) / 100); // time * 2.08E-8 * 1E6 * 6
}

uint16_t scale(uint16_t period) {
	return (15000 * ((uint32_t) period)) / 65535;
}

int32_t average(int32_t* arr, uint16_t length) {
	int32_t sum = 0;
	int divisor = length;
	for (int i = 0; i < length; i++) {
		if (arr[i] != -1) {
			if (arr[i] == 0) divisor--;
			sum += arr[i];
		} else {
			return -1;
		}
	}
	
	return sum / divisor;
}

#define AVG_LENGTH 10

int32_t average_right(int32_t x) {
	static int i = 0;
	static int32_t sonars[AVG_LENGTH] = { 0 };
	
	sonars[i++] = x;
	if (i == AVG_LENGTH) i = 0;
	
	return average(sonars, AVG_LENGTH);
}

int32_t average_front(int32_t x) {
	static int i = 0;
	static int32_t sonars[AVG_LENGTH] = { 0 };
	
	sonars[i++] = x;
	if (i == AVG_LENGTH) i = 0;
	
	return average(sonars, AVG_LENGTH);
}

int32_t pid_controller(int32_t y, int32_t r) {
	static int32_t e_old = 0;
	static int32_t E = 0;
	
	int32_t e = r - y;
	int32_t e_dot = (e - e_old) / DELTA_T;
	
	// if we are too far from wall, use K_PB1. Else use K_PB2
	int32_t k = e < 0 ? K_PB1 : K_PB2;
	
	E = CLAMP(E + e, -1, 1);
	
	int32_t u = (k * e + K_IB * E + K_DB * e_dot) / K_SCALE;
	e_old = e;
	
	return u;
}

int32_t right_pid_controller(uint16_t y, uint16_t r) {
	static int32_t e_old = 0;
	static int32_t E = 0;
	
	int32_t e = (int32_t) r - (int32_t) y;
	int32_t e_dot = (e - e_old) / DELTA_T;
	
	E = CLAMP(E + e, 0, 1000);

	int32_t u = K_P * e + K_I * E + K_D * e_dot;
	e_old = e;
	
	return u;
}

int32_t left_pid_controller(uint16_t y, uint16_t r) {
	static int32_t e_old = 0;
	static int32_t E = 0;
	
	int32_t e = (int32_t) r - (int32_t) y;
	int32_t e_dot = (e - e_old) / DELTA_T;

	E = CLAMP(E + e, 0, 1000);

	int32_t u = K_P * e + K_I * E + K_D * e_dot;
	e_old = e;
	
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

void sonar1_init(void) { // right
	// make P4.0 out and initially high
	P4SEL0 &= ~0x01;
	P4SEL1 &= ~0x01;
	P4DIR |= 0x01;
	P4OUT |= 0x01;

	// configure P5.6 as TA2CCP1
	P5SEL0 |= 0x40;
	P5SEL1 &= ~0x40;
	P5DIR &= ~0x40;
	
	TA2CTL &= ~0x0030;
	TA2CTL = 0x02c0;
	TA2EX0 &= ~0x0007;
}

void sonar2_init(void) {	// front
	// make P4.1 out and initially high
	P4SEL0 &= ~0x02;
	P4SEL1 &= ~0x02;
	P4DIR |= 0x02;
	P4OUT |= 0x02;

	// configure P10.4 as TA3CCP0
	P10SEL0 |= 0x10;
	P10SEL1 &= ~0x10;
	P10DIR &= ~0x10;
	
	TA3CTL &= ~0x0030;
	TA3CTL = 0x02c0;
	TA3EX0 &= ~0x0007;
}

double calculateDistance(uint32_t x) {
	double ns = x * pow(10, -9);
	
	return (340 * ns) * 400;
}

int32_t sonar1_measure(void) {
	uint16_t rising;
	TA2CTL &= ~0x0030;
	
	TA2CCTL1 = 0x4900;
	TA2CTL |= 0x0024;
	P4OUT &= ~0x01;  // reset P4.0 low
	Delay(55);
	P4OUT |= 0x01; 	// P4.0 high
	while ((TA2CCTL1 & 0x0001) == 0);
	rising = TA2CCR1;
	TA2CCTL1 = 0x8900;
	while ((TA2CCTL1 & 0x0001) == 0) {
		//if ((TA2CCR1 - rising - CALIBRATION) >= 16384) return -1.0;
	}
	
	uint32_t ns = TA2CCR1 - rising - CALIBRATION;
//	double dist = calculateDistance(ns);
	
	return ns;
}

int32_t sonar2_measure(void) {
	uint16_t rising;
	TA3CTL &= ~0x0030;
	TA3CCTL0 = 0x4900;
	TA3CTL |= 0x0024;
	P4OUT &= ~0x02;  // reset P4.1 low
	Delay(55);
	P4OUT |= 0x02; 	// P4.1 high
	while ((TA3CCTL0 & 0x0001) == 0);
	rising = TA3CCR0;
	TA3CCTL0 = 0x8900;
	while ((TA3CCTL0 & 0x0001) == 0) {
		//if ((TA3CCR0 - rising - CALIBRATION) >= 16384) return -1.0;
	}
	
	uint32_t ns = TA3CCR0 - rising - CALIBRATION;
//	double dist = calculateDistance(ns);
	
	return ns;
}


// ********* PRINTING METHODS ***************
void print_sonars(int32_t son1, int32_t son2, int32_t vld, int32_t vrd) {
//	char str1[80];
//	char str2[80];
//	
//	sprintf(str1, "%f", son1);
//	sprintf(str2, "%f", son2);
//	str1[6] = '\0';
//	str2[6] = '\0';
	
	Nokia5110_Clear();
	Nokia5110_OutString("Son1: ");
	Nokia5110_OutUDec(son1);
//	Nokia5110_OutString("            ");
	Nokia5110_OutString("Son2: ");
	Nokia5110_OutUDec(son2);
	Nokia5110_OutString("vld:   ");
	Nokia5110_OutUDec(vld);
	Nokia5110_OutString("vrd:   ");
	Nokia5110_OutUDec(vrd);
//	Nokia5110_OutString("Cnt: ");
//	Nokia5110_OutUDec(count++);
}

void print_udecs(uint32_t ns1, uint32_t ns2) {
	Nokia5110_Clear();
	Nokia5110_OutString("S1:           ");
	Nokia5110_OutUDec(ns1);
	Nokia5110_OutString("            ");
	Nokia5110_OutString("S2:           ");
	Nokia5110_OutUDec(ns2);
	Nokia5110_OutString("            ");
}

void print_strings(char* ns1, char* ns2) {
	Nokia5110_Clear();
	Nokia5110_OutString("Ude1:           ");
	Nokia5110_OutString(ns1);
	Nokia5110_OutString("            ");
	Nokia5110_OutString("Ude2:           ");
	Nokia5110_OutString(ns2);
	Nokia5110_OutString("            ");
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






























