/*
 *  Module: lcd
 *  Author: M. J. Cree
 *
 *  (C) 2009-2014 The University of Waikato
 *
 *  Routines to interface to the 16x2 text LCD display.
 */

#ifndef _LIBRARY_LCD_H_
#define _LIBRARY_LCD_H_

#include <stdio.h>
#include <avr/io.h>


extern void lcd_init(char ctrl_port);
extern void lcd_clear();
extern void lcd_display(char const *str, int tr);


#define lcd_port PORTD
#define LCD_EN 0x08
#define LCD_RW 0x10
#define LCD_RS 0x20


#endif
