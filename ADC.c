/* 
 * File:   ADC.c
 * Author: paola
 *
 * Created on 18 de septiembre de 2022, 7:09
 */


#include <xc.h>
#include <stdint.h>
#include <pic16f887.h>
#include "ADC.h"

//VARIABLES


//FUNCIONES
void adc_init (uint8_t adc_cs, uint8_t vref_plus, uint8_t vref_minus)
{
    ADCON1bits.ADFM = 1;    //justificado a la derecha
    
    switch(adc_cs)
    {
        case 2:
            ADCON0bits.ADCS = 0b00; // Fosc/2
            break;
            
        case 8:
            ADCON0bits.ADCS = 0b01; // Fosc/8
            break;
            
        case 32:
            ADCON0bits.ADCS = 0b10; // Fosc/32
            break;
            
        case 0:
            ADCON0bits.ADCS = 0b11; // Fosc/32
            break;
            
        default:
            ADCON0bits.ADCS = 0b11; // Frc
            break;
    }
    

    
    ADCON1bits.VCFG1 = vref_plus;
    ADCON1bits.VCFG0 = vref_minus;
    
}

void adc_start(uint8_t channel)
{
        ADCON0bits.CHS = channel;

        switch(channel)
        {
            case 0:
                ADCON0bits.CHS = 0b0000; //AN0
                break;
            case 1:
                ADCON0bits.CHS = 0b0001; //AN1
                break;
            case 2:
                ADCON0bits.CHS = 0b0010; //AN2
                break;
            case 3:
                ADCON0bits.CHS = 0b0011; //AN3
                break;
            case 4:
                ADCON0bits.CHS = 0b0100; //AN4
                break;
            case 5:
                ADCON0bits.CHS = 0b0101; //AN5
                break;
            
            case 6:
                ADCON0bits.CHS = 0b0001; //AN6
                break;
            case 7:
                ADCON0bits.CHS = 0b0010; //AN7
                break;
            case 8:
                ADCON0bits.CHS = 0b0011; //AN8
                break;
            case 9:
                ADCON0bits.CHS = 0b0100; //AN9
                break;
            case 10:
                ADCON0bits.CHS = 0b0101; //AN10
                break;

            
            
            case 11:
                ADCON0bits.CHS = 0b1011; //AN11
                break;
            case 12:
                ADCON0bits.CHS = 0b1100; //AN12
                break;
            case 13:
                ADCON0bits.CHS = 0b1101; //AN13
                break;
            case 14:
                ADCON0bits.CHS = 0b1110; //AN14
                break;
            default:
                ADCON0bits.CHS = 0b1111; //Fixed ref
                break;    
                
        ADCON0bits.ADON = 1;
        ADCON0bits.GO_DONE = 1;
        //ADIF = 0;
    }
}
