#ifndef __Spen_74hc_H
#define __Spen_74hc_H

#include "MCP23017.h"

#define U1 				1
#define U2 				2
#define U3 				3
#define U4 				4
#define U5 				5
#define U6 				6
#define U7 				7
#define U8 				8
#define U9 				9
#define U10 			10
#define U11 			11
#define U12 			12
#define U13 			13
#define U14 			14
#define U15 			15
#define U16 			16
#define U17 			17
#define U18 			18
#define U19 			19
#define U20 			20
#define HC74_PIN1 1
#define HC74_PIN2 2
#define HC74_PIN3 3
#define HC74_PIN4 4
#define HC74_PIN5 5
#define HC74_PIN6 6
#define HC74_PIN7 7
#define HC74_PIN8 8

void WriteByte_74HC (uint8_t u ,uint8_t data);
void WriteBit_74HC (uint8_t u, uint8_t PIN, uint8_t bit);
extern void (*test_con)(uint8_t, uint8_t, uint8_t);

#endif


