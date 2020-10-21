/*
 * Final_Project.c
 *
 * Created: 05-Oct-20 4:10:07 PM
 * Author : SHADY MEDHAT
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "lcd.h"
#define F_CPU 1000000UL
/*
	PD0 ==> left in the servo motor
	PD1 ==> right int the servo motor
*/

unsigned char speed=0;
unsigned char current_location;
int OverFlow_count=0;
 ISR(TIMER1_OVF_vect)
 {
	 OverFlow_count++;	/* Increment Timer Overflow count */
 }

void Ultrasonic_Sensor_Inti( unsigned char echopin ,unsigned char triggerpin)
{
	DDRC |= (1<<triggerpin);	//the trigger pin is output
	DDRC &= ~(1<<echopin);		//the echo pin is input
	PORTC &=~(1<<triggerpin);	//the output of the port is 0
	PORTC|=(1<<echopin);		//turn on pull-up for echopin
}
int distance_to_obstcal(unsigned char echopin ,unsigned char triggerpin)
{
	int distance=400;//max distance
	int duration=0;
	PORTC|=(1<<triggerpin); //start the pulse
	_delay_us(10);			
	PORTC &=~(1<<triggerpin);
	while(!(PINC & (1<<echopin)))
	{}
	Clear_timer1();
	while(PINC & (1<<echopin))
	{}
	//duration= ICR1 + (65535 * OverFlow_count); 
	// each clock count takes 1/10*10^6 sec
	distance=(ICR1 + (65535 * OverFlow_count))*(0.034/2.0)*(1/pow(10,7));
	return distance ;
}
void inti_timer1_Overflow_NormalMode(void)
{
	sei();			/* Enable global interrupt */
	TIMSK = (1 << TOIE1);	/* Enable Timer1 overflow interrupts */
	TCCR1A = 0;		/* Set all bit to zero Normal operation */
}
void Clear_timer1(void)
{
	TCNT1 = 0;	/* Clear Timer counter */
	TCCR1B = 0x01;	/* Capture on falling edge, No prescaler */
	TIFR = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
	TIFR = 1<<TOV1;	/* Clear Timer Overflow flag */
	OverFlow_count = 0;/* Clear Timer overflow count */
}
void inti_timer2_PWM(void)
{
	sei();
	TCCR2|=(1<<WGM20)|(1<<WGM21);	// fast PWM
	TCCR2|=(1<<COM21);				//compare output mode # clear OC2 on compare match and set at the top#
	TCCR2|=(1<<CS21)|(1<<CS20);		// pre-scalar 32
	TIMSK|=(1<<OCIE2);				// compare interrupt enabled
	
}
void Car_velocity(unsigned char new_speed)
{
	DDRD|=(1<<7);
	while(speed!=new_speed)// acceleration
	{
		if (speed > new_speed)
		{
			speed--;
		}
		else if (new_speed>speed)
		{
			speed++;
		}
		OCR2= (int)(255*((float)speed/100.0));
	}
	
}
// regarding we have a servo motor
void Inti_direction_PINs(void)
{
	DDRD|=(1<<PD0)|(1<<PD1);
}
void Car_current_location(int left_distance, int right_distance)
{
	if(left_distance>right_distance)
		{current_location="R";}
	else if (right_distance>left_distance)
		{current_location="L";}
	else
	{
		Car_velocity(50);// the speed of car drops to 50%
		//turn left
		PORTD |= (1<<PD0);
		PORTD &= ~(1<<PD1);
		_delay_ms(1500);
		PORTD |= (1<<PD0)|(1<<PD0);
		Car_current_location(distance_to_obstcal(4,5),distance_to_obstcal(2,3));
	}
	
}
void Car_turn(unsigned char direction)
{
	Car_velocity(50);// the speed of car drops to 50%
	switch(direction)
	{
		case 'R':
			PORTD |= (1<<PD1);
			PORTD &= ~(1<<PD0);
			_delay_ms(2000);
			PORTD |= (1<<PD0);
			PORTD &= ~(1<<PD1);
			break;
		case 'L':
			PORTD |= (1<<PD0);
			PORTD &= ~(1<<PD1);
			_delay_ms(2000);
			PORTD |= (1<<PD1);
			PORTD &= ~(1<<PD0);
			break;
	}
	_delay_ms(1500);
	PORTD |= (1<<PD0)|(1<<PD0);
	Car_current_location(distance_to_obstcal(4,5),distance_to_obstcal(2,3));
}

int main(void)
{
    /* Replace with your application code */
	inti_timer1_Overflow_NormalMode();	// timer for the sensor
	inti_timer2_PWM();					// timer for PWM for the main motor
	Ultrasonic_Sensor_Inti(0,1);		// front sensor
	Ultrasonic_Sensor_Inti(2,3);		// right sensor
	Ultrasonic_Sensor_Inti(4,5);		// left sensor
	LCD_init();							//to display the action taken
	Inti_direction_PINs();				// to initialize the output pins for the servo motor
	Car_current_location(distance_to_obstcal(4,5),distance_to_obstcal(2,3)); // to initialize the position of the car
	Car_velocity(80);					// set the speed to 80% of the power
	char str[]="the distance = ";
	char str_L[]="turning Left";
	char str_R[]="turning Right";
    while (1) 
    {
		//displaying the distance to the nearest obstacle
		LCD_displayString(str);
		LCD_intgerToString(distance_to_obstcal(0 ,1));
		LCD_goToRowColumn(0,0);
		
		if(distance_to_obstcal(0,1) < 15)
		{
			switch(current_location)
			{
				case 'R':
					LCD_clearScreen();
					LCD_displayString(str_L);
					Car_turn('L');// slows down => turn left => y3dl el 3gal 
					break;
				case 'L':
					LCD_clearScreen();
					LCD_displayString(str_R);
					Car_turn('R');// slows down => turn right => y3dl el 3gal 
					break;
			}
			Car_velocity(80);// return the velocity to 80%
			_delay_ms(1000);
			LCD_clearScreen();
		}
		
    }
}

