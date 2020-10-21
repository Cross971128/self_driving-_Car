/*
 * lcd.h
 *
 *  Created on: Sep 4, 2019
 *      Author: Joseph Hanna
 */

#ifndef LCD_H_
#define LCD_H_


#include <avr/io.h>
#include <util/delay.h>

/* Set a certain bit in any register */
#define SET_BIT(REG,BIT) (REG|=(1<<BIT))

/* Clear a certain bit in any register */
#define CLEAR_BIT(REG,BIT) (REG&=(~(1<<BIT)))

/* Toggle a certain bit in any register */
#define TOGGLE_BIT(REG,BIT) (REG^=(1<<BIT))

/* Rotate right the register value with specific number of rotates */
#define ROR(REG,num) ( REG= (REG>>num) | (REG<<(8-num)) )

/* Rotate left the register value with specific number of rotates */
#define ROL(REG,num) ( REG= (REG<<num) | (REG>>(8-num)) )

/* Check if a specific bit is set in any register and return true if yes */
#define BIT_IS_SET(REG,BIT) ( REG & (1<<BIT) )

/* Check if a specific bit is cleared in any register and return true if yes */
#define BIT_IS_CLEAR(REG,BIT) ( !(REG & (1<<BIT)) )


/* LCD HW Pins */
#define RS PB4
#define RW PB5
#define E  PB6
#define LCD_CTRL_PORT PORTB
#define LCD_CTRL_PORT_DIR DDRB
#define LCD_DATA_PORT PORTA
#define LCD_DATA_PORT_DIR DDRA

/* LCD Commands */
#define CLEAR_COMMAND 0x01
#define HOME 0x02
#define TWO_LINE_LCD_Eight_BIT_MODE 0x38
#define CURSOR_OFF 0x0C
#define CURSOR_ON 0x0E
#define SET_CURSOR_LOCATION 0x80

void LCD_sendCommand(unsigned char command);
void LCD_displayCharacter(unsigned char data);
void LCD_displayString(const char *Str);
void LCD_init(void);
void LCD_clearScreen(void);
void LCD_displayStringRowColumn(unsigned char row,unsigned char col,const char *Str);
void LCD_goToRowColumn(unsigned char row,unsigned char col);
void LCD_intgerToString(int data);

#endif /* LCD_H_ */
