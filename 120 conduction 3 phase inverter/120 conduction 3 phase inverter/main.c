/*
 * 3_phase_inverter.c
 *
 * Created: 8/12/2018 9:30:35 PM
 * Author : Mohamed Hassanin
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL;

volatile float adc10;

void init_timers(void){
  GTCCR |= 1 << TSM | 1 << PSRSYNC;
  
  TCCR1B |= 1 << WGM13 | 1 << CS10| 1 << CS11;
  TCCR3B |= 1 << WGM33 | 1 << CS30| 1 << CS31;
  TCCR4B |= 1 << WGM43 | 1 << CS40| 1 << CS41; //prescaler N = 64

  ICR1 = 125;  //TOP=F_CPU/2/N/frequency
  ICR3 = ICR1;
  ICR4 = ICR1;
    
  TCCR1A |= 1 << COM1A1 | 1 << COM1A0 | 1 << COM1B1;
  TCCR3A |= 1 << COM3A1 | 1 << COM3A0 | 1 << COM3B1;
  TCCR4A |= 1 << COM4A1 | 1 << COM4A0 | 1 << COM4B1;
  
  OCR1A = ICR1 * 2 /3 ;
  OCR3A = OCR1A; // +1 and -1 to add some dead time between switches
  OCR4A = OCR1A;
  OCR1B = ICR1/3;
  OCR3B = ICR1/3;
  OCR4B = ICR1/3;
  
  TCNT1 = ICR1/3;     //shift left 
  TCNT3 = 0;    //stationary 
  TCNT4 = -1 * TCNT1;     //shift right the same amount
  
  GTCCR = 0;
}



int main(void)
{ //switch >> OCnX >> PXN >> arduino mega pin
  //1 >> OC1A >> PB5 >> 11; 2 >> OC3A >> PE3 >> 5;  3 >> OC4A >> PH3 >> 6;
  //4 >> OC1B >> PB6 >> 12; 5 >> OC3B >> PE4 >> 2;  6 >> OC4B >> PH4 >> 7;
  DDRB |= 1 << PINB5 | 1 << PINB6;
  DDRE |= 1 << PINE3 | 1 << PINE4;
  DDRH |= 1 << PINH3 | 1 << PINH4;

  init_timers();

    while (1) 
    {
    }
}


