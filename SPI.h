/* 
 * File:   SPI.h
 * Author: paola
 *
 * Created on 18 de septiembre de 2022, 7:14
 */

/*
 * Archivo          : spi.h
 * Autor            : Ligo George
 * Compania         : electroSome
 * Proyecto         : SPI Library for MPLAB XC8
 * Microcontrolador : PIC 16F877A
 * Creado en Abril 15, 2017, 5:59 PM
 * Modificado       : Paola Ayala Pineda
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef __SPI_H
#define	__SPI_H

#include <xc.h> 
#include <pic16f887.h>
//FRECUENCIA DEL RELOJ
typedef enum 
{
    SPI_MASTER_OSC_DIV4  = 0b00100000,
    SPI_MASTER_OSC_DIV16 = 0b00100001,
    SPI_MASTER_OSC_DIV64 = 0b00100010,
    SPI_MASTER_TMR2      = 0b00100011,
    SPI_SLAVE_SS_EN      = 0b00100100,  //cuando estamos utilizando al microcontrolador como esclavo
    SPI_SLAVE_SS_DIS     = 0b00100101   //es esclavo, pero no está habilitado el selector slave sino cualquier otro pin
}Spi_Type;


//SMP
typedef enum
{
    SPI_DATA_SAMPLE_MIDDLE   = 0b00000000,  //Cuando transmite el dato a la mitad 
    SPI_DATA_SAMPLE_END      = 0b10000000   //cuando transmite el dato al final
}Spi_Data_Sample;


//MODO DE RELOJ
typedef enum
{
    SPI_CLOCK_IDLE_HIGH  = 0b00010000,
    SPI_CLOCK_IDLE_LOW   = 0b00000000        
}Spi_Clock_Idle;

typedef enum
{
    SPI_IDLE_2_ACTIVE    = 0b00000000,
    SPI_ACTIVE_2_IDLE    = 0b01000000
}Spi_Transmit_Edge;


void spiInit(Spi_Type, Spi_Data_Sample, Spi_Clock_Idle, Spi_Transmit_Edge);
void spiWrite(char cont_m);
unsigned spiDataReady();
char spiRead();

#endif /* SPI_H */

