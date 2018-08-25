/*
 * brushless motor.c
 *
 * Created: 8/20/2018 3:45:30 AM
 * Author : Mohamed Hassanin
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL;

void init_timers(void){
	GTCCR = 1 << TSM | 1 <<	PSRSYNC;
	
  TCCR1B |= 1 << WGM13 | 1 << WGM12 | 1 << CS10| 1 << CS11;
  TCCR3B |= 1 << WGM33 | 1 << WGM32 | 1 << CS30| 1 << CS31;
  TCCR4B |= 1 << WGM43 | 1 << WGM42 | 1 << CS40| 1 << CS41; //prescaler N = 64
  
  TCCR1A |= 1 << WGM11;
  TCCR3A |= 1 << WGM31;
  TCCR4A |=	1 << WGM41;
  
  ICR1 =1250;
  ICR3 = ICR1;
  ICR4 = ICR1;
  
  TIMSK1 =  1 << OCIE1B | 1 << TOIE1;
  TIMSK3 =  1 << OCIE3B | 1 << TOIE3;
  TIMSK4 =  1 << OCIE4B | 1 << TOIE4;
  
  OCR1B = ICR1 / 2;
  OCR3B = ICR1 / 2;
  OCR4B = ICR1 / 2;
  
  TCNT1 = ICR1/6;     //shift left
  TCNT3 = 0;    //stationary
  TCNT4 = -1 * TCNT1;    //shift right the same amount
 
  GTCCR = 0;
  sei();
}

ISR(TIMER1_OVF_vect){
	//put your if here
	PORTC |= 0b00000010;
	PORTC &= 0b11011111;
}

ISR(TIMER1_COMPB_vect){
	PORTC |= 0b00010000;
	PORTC &= 0b11111011;
}

ISR(TIMER3_OVF_vect){
	PORTC |= 0b00000100;
	PORTC &= 0b10111111;
}

ISR(TIMER3_COMPB_vect){
	PORTC |= 0b00100000;
	PORTC &= 0b11110111;
}

ISR(TIMER4_OVF_vect){
	PORTC |= 0b00001000;
	PORTC &= 0b11111101;
}

ISR(TIMER4_COMPB_vect){
	PORTC |= 0b01000000;
	PORTC &= 0b11101111;
}



int main(void)
{
	DDRC = 0b01111110;
	init_timers();
    while (1) 
    {
    }
}

