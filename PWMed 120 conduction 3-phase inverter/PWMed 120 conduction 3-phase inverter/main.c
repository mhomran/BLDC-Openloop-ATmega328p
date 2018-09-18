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
  TCCR5B |= 1 << WGM53 | 1 << CS50| 1 << CS51; 

  ICR1 = 2500;  //50 Hz TOP=F_CPU/2/N/frequency
  ICR3 = ICR1;
  ICR4 = ICR1;
  
  ICR5 = 125;
  
  TCCR1A |= 1 << COM1A1 | 1 << COM1A0 | 1 << COM1B1;
  TCCR3A |= 1 << COM3A1 | 1 << COM3A0 | 1 << COM3B1;
  TCCR4A |= 1 << COM4A1 | 1 << COM4A0 | 1 << COM4B1;
  
  TCCR5A |= 1 << COM5B1;
  TIMSK5 |= 1 << TOIE1;
  
  OCR1A = ICR1 * 2 /3 ;
  OCR3A = OCR1A; // +1 and -1 to add some dead time between switches
  OCR4A = OCR1A;
  OCR1B = ICR1/3;
  OCR3B = ICR1/3;
  OCR4B = ICR1/3;
  
  OCR5B = ICR5/2;  //arduino mega >> 45
  
  TCNT1 = ICR1/3;     //shift left 
  TCNT3 = 0;    //stationary 
  TCNT4 = -1 * TCNT1;     //shift right the same amount
  
  
  GTCCR = 0;
}
volatile int curADC, prevADC;
void init_adc(void){   //A0
  ADMUX |= (1 << REFS0) ;
  ADCSRA |= (1 << ADEN) | (1 << ADPS1) | (1 << ADPS2);
}


ISR(TIMER5_OVF_vect){ 
	//ADSC
	ADCSRA |= (1<<ADSC);
	while( ADCSRA & (1<<ADSC) );
	curADC = ADC;
	if (!((curADC < (prevADC + 5)) && (curADC > (prevADC - 5)))){
	prevADC = curADC;
	
	GTCCR = (1<<TSM)|(1<<PSRSYNC); // halt all timers
    TCNT1 = ICR1/3;     //shift left //initial value 50% shift
    TCNT3 = 0;    //stationary
    TCNT4 = -1 * TCNT1;     //shift right the same amount

    OCR5B = (ICR5 - 1) * (prevADC / 1023.0);
  
    GTCCR=0;
  }
} 

int main(void)
{ //switch >> OCnX >> PXN >> arduino mega pin
  //1 >> OC1A >> PB5 >> 11; 2 >> OC3A >> PE3 >> 5;  3 >> OC4A >> PH3 >> 6;
  //4 >> OC1B >> PB6 >> 12; 5 >> OC3B >> PE4 >> 2;  6 >> OC4B >> PH4 >> 7;
  DDRB |= 1 << PINB5 | 1 << PINB6;
  DDRE |= 1 << PINE3 | 1 << PINE4;
  DDRH |= 1 << PINH3 | 1 << PINH4;
  DDRL |= 1 << PINL4;
  init_adc();
  ADCSRA |= (1<<ADSC);
  while( ADCSRA & (1<<ADSC) );
  prevADC = ADC;
  init_timers();
  sei();
    while (1) 
    {
    }
}