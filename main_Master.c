//MODIFICADO

/* 
 * File:   M_main.c
 * Author: paola
 *
 * Created on 21 de septiembre de 2022, 1:56
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


//LIBRERIAS
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pic16f887.h>
#include "ADC.h"
#include "I2C.h"
#include "LCD.h"
#include "OSC.h"
#include "SPI.h"
#include "TMR0.h"


//DEFINICIONES
#define _XTAL_FREQ 4000000
#define RS RC0
#define EN RC1
#define D0 RD0
#define D1 RD1
#define D2 RD2
#define D3 RD3
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7

//DECLARACION DE FUNCIONES
void configuracion_1(void);
uint8_t BCD_a_Decimal (uint8_t numero);
uint8_t Decimal_a_BCD (uint8_t numero);    // Función que convierte un número decimal a BCD.
unsigned short map(uint8_t val, uint8_t in_min, uint8_t in_max, //Función del mapeo
            unsigned short out_min, unsigned short out_max);

//VARIABLES
//Seccion varias
    uint8_t enviar = 0;
    uint8_t entrada = 0;
    uint8_t flag2 = 0;
    uint8_t flag3 = 0;
    uint8_t a;
    uint8_t dato;
             
//Seccion sensores
    uint8_t hours = 12;                             
    uint8_t minutes = 59;                           
    uint8_t seconds = 59;
    uint8_t hours2 = 1;
    uint8_t temp = 0;

//Seccion motores
    uint8_t pot = 25;
    uint8_t change = 0;
    uint8_t front = 0;
    uint8_t move = 0;
    uint8_t state = 0;
    uint8_t direccion = 0;
    

//MAIN
void __interrupt() isr(void){
    if(INTCONbits.RBIF){ //Interrupción del puerto B
        if(!PORTBbits.RB1){//cambiamos dirección del motor DC
            direccion = 1;
        }
        else if(!PORTBbits.RB2){//cambiamos dirección del motor DC
            direccion = 0;
        }
        INTCONbits.RBIF = 0;
    }
    
    if(PIR1bits.RCIF){//Interrupción de recepción USART
        entrada = RCREG;
        if(entrada == 0){//Verificiamos que nos enviaron para enviar de regreso la variable que pidieron 
            enviar = seconds;
        }
        else if(entrada == 1){
            enviar = minutes;
        }
        else if(entrada == 2){
            enviar = hours;
        }
        
        else if(entrada == 3){
            enviar = temp;
        }
        TXREG = enviar;//Enviamos el valor segun la bandera
    }
    if(INTCONbits.T0IF){ //Interrupción TMR0?
        if(move == 1){
            if(flag2 == 1){ //Movemos el motor SERVO según la variable POTENCIOMETRO 2
                flag2 = 0;
                if (flag2 == 0){
                    PORTDbits.RD0 = 0;
                }
                TMR0 = pot; //cargamos valor
            }
            else if(flag2 == 0){
                flag2 = 1;
                if (flag2 == 1){
                    PORTDbits.RD0 = 1;
                }
                TMR0 = 255-pot; //cargamos valor
            }
        }
        INTCONbits.T0IF = 0; //limpiamos bandera
    }
    if(PIR1bits.TMR1IF)//INTERRUPCION TMR1
        if (move == 1){
            change++;
            if (change == 10){//Cada cierto tiempo movemos el STEPPER de dirección
                front = !front;
                change = 0;
            }
            if (flag3 == 0){ //Movemos SERVO para adelante y para atrás
                pot++;
                if (pot >= 35){
                    flag3 = 1;
                }
            }
            else if (flag3 == 1){
                pot--;
                if (pot <= 8){
                    flag3 = 0;
                }
            }
        }
        PIR1bits.TMR1IF = 0;       // Poner a 0 la bandera de bit del TMR1IF
    
    if(PIR1bits.TMR2IF){// INTERRUPCION TMR2?
        if (move == 1){
            if (front == 1){//Secuencia STEPPER para adelante
                if (state == 0){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0;
                    state = 1;
                }
                else if (state == 1){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0;
                    state = 2;
                }
                else if (state == 2){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 0;
                    state = 3;
                }
                else if (state == 3){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 0;
                    state = 4;
                }
                else if (state == 4){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 1;
                    state = 5;
                }
                else if (state == 5){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 1;
                    state = 6;
                }
                else if (state == 6){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 1;
                    state = 7;
                }
                else if (state == 7){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0 ;
                    state = 0;
                }
            }
            else if (front == 0){ //Secuencia stepper hacia atrás
                if (state == 0){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0;
                    state = 7;
                }
                else if (state == 1){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0;
                    state = 0;
                }
                else if (state == 2){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 1;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 0;
                    state = 1;
                }
                else if (state == 3){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 0;
                    state = 2;
                }
                else if (state == 4){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 1;
                    PORTDbits.RD6 = 1;
                    state = 3;
                }
                else if (state == 5){
                    PORTDbits.RD3 = 0;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 1;
                    state = 4;
                }
                else if (state == 6){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 1;
                    state = 5;
                }
                else if (state == 7){
                    PORTDbits.RD3 = 1;
                    PORTDbits.RD4 = 0;
                    PORTDbits.RD5 = 0;
                    PORTDbits.RD6 = 0 ;
                    state = 6;
                }
            }
        }
        PIR1bits.TMR2IF = 0;
    }
    return;
}

void main(void) {
    configuracion_1();
    seconds = Decimal_a_BCD(seconds);
    minutes = Decimal_a_BCD(minutes);
    hours = Decimal_a_BCD(hours);

    I2C_Start();                // Llamamos a la función Start.
    I2C_Write(0xD0);            // Escribimos en SSPBUF la dirección de DS1307 1101000 + 0 de escritura.
    I2C_Write(0x00);            // Dirección de segundos.
    I2C_Write(seconds);            // Reiniciamos los segundos.
    I2C_Write(minutes);         // Cargamos el valor de minutos en la dirección de minutos.
    I2C_Write(hours);           // Cargamos el valor de horas en la dirección de horas.
    I2C_Stop();                 // Llamamos a la función Stop.
    __delay_ms(200);            // Retardo de 200 ms.
    
    while(1)
    {
        I2C_Start();                        // Llamamos a la función Start.
        I2C_Write(0xD0);                    // Escribimos en SSPBUF la dirección de DS1307 1101000 + 0 de escritura.
        I2C_Write(0);                       // Dirección de segundos.
        I2C_ReStart();                      // Llamamos a la función ReStart.
        I2C_Write(0xD1);                    // Escribimos en SSPBUF la dirección de DS1307 1101000 +1 de lectura.
        
        seconds=I2C_Read();                 // Cargamos la variable "seconds" con el valor de SSPBUF.
        I2C_Ack();                          // ACK.
        
        minutes=I2C_Read();                 // Cargamos la variable "minutes" con el valor de SSPBUF.
        I2C_Ack();                          // ACK.
        
        hours=I2C_Read();                   // Cargamos la variable "hours" con el valor de SSPBUF.
        I2C_NO_Ack();                       // NO ACK.
       
        I2C_Stop();                         // Llamamos a la función Stop.
        __delay_ms(50);                     // Retardo de 50 ms. 

        //adc_start(12);

        I2C_Start();                        // Llamamos a la función Start.
        I2C_Write(0x90);                    // Escribimos en SSPBUF la dirección del sensor de TEMPERATURA LM75 + 0 de escritura.
        I2C_Write(0);                       // Dirección de temperatura.
        I2C_ReStart();                      // Llamamos a la función ReStart.
        I2C_Write(0x91);                    // Escribimos en SSPBUF la dirección del sensor de TEMPERATURA LM75 +1 de lectura.
        temp=I2C_Read();                 // Cargamos la variable "temperatura" con el valor de SSPBUF.
        I2C_NO_Ack();                       // NO ACK.
        I2C_Stop();                         // Llamamos a la función Stop.
        __delay_ms(50); 
        
      
        
        if (move == 1){
            PIE1bits.TMR1IE = 1;       // Interrupción habilitada por desbordamiento
            T1CONbits.TMR1ON = 1;            // Encender el temporizador Timer1
            INTCONbits.T0IE = 1;        // Habilitamos interrupcion TMR0
            T2CONbits.TMR2ON = 1;       // Encendemos TMR2
            PIE1bits.TMR2IE = 1;       // Interrupción habilitada por desbordamiento
            if (direccion == 1){
                PORTDbits.RD1 = 1;
                //PORTDbits.RD2 = 0;
            }
           else if (direccion == 0){
                PORTDbits.RD1 = 0;
               // PORTDbits.RD2 = 1;
            }
        }
        else if (move == 0){
            PORTDbits.RD0 = 0;
            PIE1bits.TMR1IE = 0;       // Interrupción deshabilitada por desbordamiento
            T1CONbits.TMR1ON = 0;            // Apagamos el temporizador Timer1
            INTCONbits.T0IE = 0;        // Des habilitamos interrupcion TMR0
            T2CONbits.TMR2ON = 0;       // Apagamos TMR2
            PIE1bits.TMR2IE = 0;       // Interrupción des habilitada por desbordamiento
            PORTDbits.RD1 = 0;
            PORTDbits.RD2 = 0;
        }
        
        if (temp >= 15){ 
                hours2 = BCD_a_Decimal(hours); //Establecemos ciertos horarios de riego
                if (hours2 == 7){
                    move = 1;
                }
                else if (hours2 == 8){
                    move = 1;
                }
                else if (hours2 == 16){
                    move = 1;
                }
                else if (hours2 == 20){
                    move = 1;
                }
                else{
                    move = 0;
                }
            
        }
        else {
            move = 0;
        }
    }
    return;
}
//FUNCIONES
void configuracion_1(void){
    ANSEL = 0;
    ANSELH = 0;
    TRISB = 0b00001111;
    TRISC = 0b10000001;
    TRISA = 0;
    PORTA = 0;
    TRISD = 0;
    PORTB = 0;
    PORTD = 0;
    init_osc_MHz(2);
    I2C_Init(100000);        // Inicializar Comuncación I2C
    INTCONbits.PEIE = 1;        // Habilitamos int. de perifericos
    INTCONbits.GIE = 1;         // Habilitamos int. globales
    PIR1bits.ADIF = 0;          // Limpiamos bandera de ADC
    PIE1bits.ADIE = 1;          // Habilitamos interrupcion de ADC
    PIE1bits.RCIE = 1;          // Habilitamos Interrupciones de recepción
    
    //TMR0
    INTCONbits.T0IE = 1;        // Habilitamos interrupcion TMR0
    tmr0_init(64);  //configuración prescaler 256 TMR0 
    INTCONbits.T0IF = 0; //limpiamos bandera
    TMR0 = 0;
    
    //TMR1
    PIR1bits.TMR1IF = 0;       // Poner a 0 la bandera de bit del TMR1IF
    TMR1H = 0x22;          // Poner el valor inicial para el temporizador Timer1
    TMR1L = 0x00;
    TMR1CS = 0;            // Temporizador1 cuenta los pulsos del oscilador interno
    T1CKPS1 =1 ; // El valor del pre-escalador asignada es 1:8
    T1CKPS0 = 1;
    PIE1bits.TMR1IE = 1;       // Interrupción habilitada por desbordamiento
    T1CONbits.TMR1ON = 1;            // Encender el temporizador Timer1
    
    //TMR2
    PIR1bits.TMR2IF = 0;        // Limpiamos bandera de interrupcion del TMR2
    PR2 = 20;                  // periodo de 2ms
    TMR2 = 0;
    T2CONbits.T2CKPS = 0b11;    // prescaler 1:16
    T2CONbits.TOUTPS = 0b1111; //postscaler 16
    T2CONbits.TMR2ON = 1;       // Encendemos TMR2
    PIE1bits.TMR2IE = 1;       // Interrupción habilitada por desbordamiento
    
    //PORTB
    INTCONbits.RBIE = 1;   
    IOCB = 0b00000110;         
    INTCONbits.RBIF = 0;     
    
    
    // Configuraciones de comunicacion serial
    //SYNC = 0, BRGH = 1, BRG16 = 1, SPBRG=25 <- Valores de tabla 12-5
    TXSTAbits.SYNC = 0;         // Comunicación ascincrona (full-duplex)
    TXSTAbits.BRGH = 1;         // Baud rate de alta velocidad 
    BAUDCTLbits.BRG16 = 1;      // 16-bits para generar el baud rate
    
    SPBRG = 25;
    SPBRGH = 0;                 // Baud rate ~9600, error -> 0.16%
    
    RCSTAbits.SPEN = 1;         // Habilitamos comunicación
    TXSTAbits.TX9 = 0;          // Utilizamos solo 8 bits
    TXSTAbits.TXEN = 1;         // Habilitamos transmisor
    RCSTAbits.CREN = 1;         // Habilitamos receptor
}

unsigned short map(uint8_t x, uint8_t x0, uint8_t x1,
            unsigned short y0, unsigned short y1){
    return (unsigned short)(y0+((float)(y1-y0)/(x1-x0))*(x-x0));
}
uint8_t BCD_a_Decimal (uint8_t numero)            // Función que convierte un número BCD a decimal.
{
  return ((numero >> 4) * 10 + (numero & 0x0F));  // Retornamos la variable "numero" desplazado 4 posiciones a la izquierda, multiplicado por 10 más "numero" &&  1111.
}
uint8_t Decimal_a_BCD (uint8_t numero)            // Función que convierte un número decimal a BCD. 
{
    return (((numero / 10) << 4) + (numero % 10));// Retornamos la decena de la variable "numero" desplazado 4 bits a la derecha y las unidades de la variable "numero" en el nibble menos significativo
}
