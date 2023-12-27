#ifndef PINMAP_H_
#define PINMAP_H_

#include "main.h"
#include "Spen_74hc.h"

#define SCALE_20R                       20
#define SCALE_200R                      200
#define SCALE_2K                        2000
#define SCALE_20K                       20000
#define SCALE_200K                      200000
#define SCALE_2M                        2000000



#define CON1_SPEC               test_con (U1, HC74_PIN2, LOW) // chan con1 dau vao 
#define CON1_GND                test_con (U1, HC74_PIN2, HIGH) // chan con1 noi gnd

#define CON2_SPEC               test_con (U1, HC74_PIN1, LOW) // chan con2 dau vao
#define CON2_GND                test_con (U1, HC74_PIN1, HIGH) // chan con2 dau vao

#define CON3_SPEC               test_con (U1, HC74_PIN4, LOW) // chan con2 dau vao
#define CON3_GND                test_con (U1, HC74_PIN4, HIGH) // chan con2 dau vao

#define CON4_SPEC               test_con (U1, HC74_PIN6, LOW) // chan con2 dau vao
#define CON4_GND                test_con (U1, HC74_PIN6, HIGH) // chan con2 dau vao

#define CON5_SPEC               test_con (U1, HC74_PIN6, LOW) // chan con5 dau vao
#define CON5_GND                test_con (U1, HC74_PIN6, HIGH) // chan con5 noi gnd

#define CON6_SPEC               test_con (U1, HC74_PIN5, LOW) // chan con6 dau vao
#define CON6_GND                test_con (U1, HC74_PIN5, HIGH) // chan con6 noi gnd

#define CON7_SPEC               test_con (U1, HC74_PIN8, LOW) // chan con7 dau vao
#define CON7_GND                test_con (U1, HC74_PIN8, HIGH) // chan con7 noi gnd

#define CON8_SPEC               test_con (U1, HC74_PIN8, LOW) // chan con8 dau vao
#define CON8_GND                test_con (U1, HC74_PIN8, HIGH) // chan con8 dau vao

#define CON9_SPEC               test_con (U1, HC74_PIN2, LOW) // chan con9 dau vao
#define CON9_GND                test_con (U1, HC74_PIN2, HIGH) // chan con9 moi gnd

#define CON10_SPEC              test_con (U1, HC74_PIN1, LOW) // chan con10 dau vao
#define CON10_GND               test_con (U1, HC74_PIN1, HIGH) // chan con10 noi gnd

#define CON11_CON12_SPEC        test_con (U2, HC74_PIN2, LOW) // chan con11 con12 dau vao
#define CON11_CON12_GND         test_con (U2, HC74_PIN2, HIGH) // chan con11 con12 noi gnd

#define CON13_CON14_SPEC        test_con (U2, HC74_PIN1, LOW) // chan con13 con14 dau vao
#define CON13_CON14_GND         test_con (U2, HC74_PIN1, HIGH) // chan con13 con14 dau vao

extern uint16_t adc_value;
//extern float R_spec;

float read_res_fpcb(uint16_t *adc, uint32_t scale_value, uint8_t time_test);
void GND_INIT();

#endif 





