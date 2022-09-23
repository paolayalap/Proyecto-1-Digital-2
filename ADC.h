/* 
 * File:   ADC.h
 * Author: paola
 *
 * Created on 18 de septiembre de 2022, 7:12
 */

#ifndef __ADC__H_
#define	__ADC__H_

#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)


#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include "ADC.h"

void adc_init (uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus);
void adc_start(uint8_t channel);
//uint16_t adc_read(void);

#endif	/* __ADC__H_*/

