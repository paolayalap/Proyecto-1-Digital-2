/* 
 * File:   USART.c
 * Author: paola
 *
 * Created on 18 de septiembre de 2022, 7:07
 */

#include <xc.h> // include processor files - each processor file is guarded. 
#include <pic16f887.h>
#include <stdint.h>
#include "USART.h"

/*
 Inicialización para TX y RX
 */
void initUART(void){
    //ENVIO
    TXSTAbits.TX9 = 0;
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    BAUDCTLbits.BRG16 = 0;
    SPBRG = 25;
    SPBRGH = 0;
    TXSTAbits.TXEN = 1;
    //**************************************************************************
    //RECEPCION
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;    
}
