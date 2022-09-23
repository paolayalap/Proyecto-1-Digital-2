/* 
 * File:   LCD.h
 * Author: paola
 *
 * Created on 18 de septiembre de 2022, 7:14
 */

#ifndef __LCD_H
#define	__LCD_H
#define _XTAL_FREQ 4000000

void Lcd_Port(char a);
void Lcd_Cmd(char a);
void Lcd_Clear(void);

void Lcd_Set_Cursor(char a, char b);
void Lcd_Init(void);
void Lcd_Write_Char(char a);
void Lcd_Write_String(char *a);
void Lcd_Shift_Right(void);
void Lcd_Shift_Left(void);


#endif	/* LCD_POST_H */


