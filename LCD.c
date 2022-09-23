/* 
 * File:   LCD.c
 * Author: paola
 *
 * Created on 18 de septiembre de 2022, 7:07
 */


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include "LCD.h"


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

void Lcd_Port(char a){
	if (a & 1)
        D0 = 1;
    else
        D0 = 0;
    
    if (a & 2)
        D1 = 1;
    else
        D1 = 0;
    
    if (a & 3)
        D2 = 1;
    else
        D2 = 0;
    
    if (a & 4)
        D3 = 1;
    else
        D3 = 0;
    
    if (a & 1)
        D4 = 1;
    else
        D4 = 0;

    if (a & 2)
        D5 = 1;
    else
        D5 = 0;

    if (a & 4)
        D6 = 1;
    else
        D6 = 0;

    if (a & 8)
        D7 = 1;
    else
        D7 = 0;
}

void Lcd_Cmd(char a){
	RS = 0;	// => RS = 0 // Dato en el puerto lo va a interpretar como comando
	Lcd_Port(a);
	EN = 1;	// => E = 1
	__delay_ms(4);
	EN = 0; // => E = 0

}

void Lcd_Clear(void){
	Lcd_Cmd(0);
	Lcd_Cmd(1);
}


void Lcd_Set_Cursor(char a, char b){
	char temp, z, y;
    if (a == 1) {
        temp = 0x80 + b - 1;
        z = temp >> 4;
        y = temp & 0x0F;
        Lcd_Cmd(z);
        Lcd_Cmd(y);
	}
	else if (a == 2){
		temp = 0XC0 + b - 1;
		temp = 0xC0 + b - 1;
        z = temp >> 4;
        y = temp & 0x0F;
        Lcd_Cmd(z);
        Lcd_Cmd(y);
	}
}


void Lcd_Init(void){
	Lcd_Port(0X00);
	__delay_ms(20);
	Lcd_Port(0X30);
	__delay_ms(5);
	Lcd_Port(0X03); 
	__delay_ms(11);
	Lcd_Cmd(0X03); 

    Lcd_Cmd(0x02);
    Lcd_Cmd(0x02);
    Lcd_Cmd(0x08);
    Lcd_Cmd(0x00);
    Lcd_Cmd(0x0C);
    Lcd_Cmd(0x00);
    Lcd_Cmd(0x06);

}


void Lcd_Write_Char(char a){
	char temp, y;
    temp = a & 0x0F;
    y = a & 0xF0;
    RS = 1;
    Lcd_Port(y >> 4);
    EN = 1;
    __delay_us(40);
    EN = 0;
    Lcd_Port(temp);
    EN = 1;
    __delay_us(40);
    EN = 0;

}

void Lcd_Write_String(char *a){
	int i;
	for (i = 0; a[i] != '\0'; i++)
		Lcd_Write_Char(a[i]);
}


void Lcd_Shift_Right(void){
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left(void){
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x08);
}



