/* 
 * File:   USART.h
 * Author: paola
 *
 * Created on 18 de septiembre de 2022, 7:14
 */


// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __USART_H
#define	__USART_H

#include <xc.h> // include processor files - each processor file is guarded. 
#include <pic16f887.h>
#include <stdint.h>
#include "USART.h"

void initUART(void);

#endif	/* __USART_H */
