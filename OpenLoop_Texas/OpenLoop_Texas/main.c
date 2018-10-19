/*
 * OpenLoop_Texas.c
 *
 * Created: 9/16/2018 2:43:15 PM
 * Created by : Mohamed Hassanin
 */ 
//AND gate is used
#include <avr/io.h>
#include "main_open_loop.h"
#include "hall_sensor.h"
#include <avr/interrupt.h>
// Configure System Clock = DCO = 16Mhz
#define F_CPU 16000000UL

/* Relation between the hall sequence 000-111 and the HS/LS pedriver 
   o/p - THIS IS FIXED for 120deg 3-ph BLDC Commutation
*/
#ifdef DIRECTION_CCW                                        
  // Direction = 0x01 
unsigned char Hall_DIR_sequence[] = { 0x00,            
                                      HS_W|LS_V,       // Hall position 001
                                      HS_V|LS_U,       // Hall position 010
                                      HS_W|LS_U,       // Hall position 011
                                      HS_U|LS_W,       // Hall position 100
                                      HS_U|LS_V,       // Hall position 101
                                      HS_V|LS_W,       // Hall position 110
                                        0x00   };
#endif

#ifdef DIRECTION_CW
  // Direction = 0x00 
unsigned char Hall_DIR_sequence[] = {  0x00,          
                                      HS_V|LS_W,       // Hall position 001
                                      HS_U|LS_V,       // Hall position 010
                                      HS_U|LS_W,       // Hall position 011
                                      HS_W|LS_U,       // Hall position 100
                                      HS_V|LS_U,       // Hall position 101
                                      HS_W|LS_V,       // Hall position 110
                                        0x00   };
#endif

// Motor and Commutation Variables
unsigned int Desired_PWM_DutyCycle, Current_PWM_DutyCycle, PWM_BucketStep, PWM_Update_Counter, ADC_Sample_Counter;
unsigned char PreDriver_Sequence, Hall_IN, Motor_Status, Hall_State_Unknown;
unsigned char Motor_status;

// ADC Variables
volatile int curADC, prevADC;
int adc10;

// Function definition
void PWM_update (unsigned char Next_Hall_Sequence);
void Start_Motor(void);
void Stop_Motor(void);

int main(void)
{
	unsigned int Temp_DutyCycle;    
	// and Hall Sensor Inputs
	DDRD = 0b00000000;     // Input DIR >mega 20 , 19 , 18 = PD1 , PD2 , PD3
	EICRA |= 1 << ISC10 | 1 << ISC20 | 1 << ISC30; // Trigger on any edge
	EIMSK |= 1 << INT1 | 1 << INT2 | 1 << INT3;
	// INT1 >> yellow || INT2 << green || INT3 blue
  
	//you can put INT0 for fault Input
  
	//Configure Timer1 PWM pin and the enable pins
  
	//Configure Port I/O as Timer PWM output pin    
	//PWM HS> (pin 11 in Arduino mega) OCR1A PB5 
	DDRB |= 1 << PINB5;
	//HS en PL 2,4,6  LS en PL 1,3,5 ||mega HS >> 47, 45, 43 || LS >> 48, 46, 44
	DDRL = 0b01111110;

	//Timer initialization
	TCCR1A |= 1 << WGM11;
	TCCR1B |= 1 << WGM12 | 1 << WGM13; //fast mode ICR1 = PWM period 
	ICR1 = TIMER_PWM_PERIOD;
	TCCR1A |= 1 << COM1A1;  // reset/set for high side
	TIMSK1 |= 1 << TOIE1; // Timer1 overflow interrupt enabled 

	Current_PWM_DutyCycle = MIN_PWM_DUTYCYCLE; // Initial Duty cycle
  
	//Initialize PWM outputs with initial duty cycle counts
	OCR1A = Current_PWM_DutyCycle;  
  
	//Init_ADC()
	ADMUX |= 1 << REFS0; //AVCC with external capacitor at AREF pin, MUX = 0000 >ADC0 >PF0>A0
	ADCSRA |= 1 << ADEN | 1 << ADPS2;  //N = 16
	ADCSRA |= (1<<ADSC);
	while( ADCSRA & (1<<ADSC) );
	prevADC = ADC;
  
	//Variable Initializations
	PWM_Update_Counter = 0x0;
  

	//Start Motor
	Start_Motor();
  
	sei();                                 // enable interrupts
  
while (1) 
{
    
	// Trigger ADC Sampling
      
	ADCSRA |= 1 << ADSC;
	while(ADCSRA & (1 << ADSC));
	curADC = ADC;
	if(curADC > 1000){
		Desired_PWM_DutyCycle = TIMER_PWM_PERIOD - 1;
	}
	else if ((curADC > (prevADC + 10)) || (curADC < (prevADC - 10)))
		prevADC = curADC;
		Temp_DutyCycle = (prevADC/1024.0) * (int)(TIMER_PWM_PERIOD * 0.99); 
           
		if (Temp_DutyCycle < MIN_PWM_DUTYCYCLE)        
		Desired_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;  // < Min DutyCycle %age - latch to min value, 1023
		else
		Desired_PWM_DutyCycle = Temp_DutyCycle;     
  }
}   


void Start_Motor(void)
{
	//Read Speed Input and update duty cycle variable
	//Start_ADC_Conversion();
	ADCSRA |= 1 << ADSC;
	while(ADCSRA & (1 << ADSC));
	adc10 =ADC;
	Desired_PWM_DutyCycle = (adc10/1024.0) * (int)(TIMER_PWM_PERIOD * 0.99);

	//Read Hall inputs
	Hall_IN = ((PIND & 0b00001110) >> 1);

	//Start PWM TimerB
	PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];
	PWM_update(PreDriver_Sequence);
  
	if(Current_PWM_DutyCycle < Desired_PWM_DutyCycle)   // 1023
		{
		//Initially PWM duty cycle set to min duty cycle. If desired duty cycle < min dutycycle, latch
		//at min duty cycle, else compute #steps required to reach input speed value in ~100ms
		PWM_BucketStep = (Desired_PWM_DutyCycle-Current_PWM_DutyCycle)/(STARTUP_STEPS);  
		if(PWM_BucketStep <= 0)
			{
			PWM_BucketStep = 1;
    }
    Motor_Status = StartUp;
  }
	else
	{
    Motor_Status = Running;
	}
  
	TCCR1B |= 1 << CS10; //N =1 enable timer
}

void Stop_Motor(void)
{
	cli(); //disable global interrupt
	TCCR1B &= ~(1 << CS11);
	Motor_Status = Stopped;
}

ISR(TIMER1_OVF_vect){
	// heart beat signal = PWM period
	//In computer science, a heartbeat is a periodic signal generated by hardware or software to indicate normal operation or to synchronize other parts of a computer system.[1] #wiki pedia
	PWM_Update_Counter++;
	ADC_Sample_Counter++;
	if (Motor_Status == StartUp)
	{
		if((PWM_Update_Counter > DUTYCYCLE_CHANGE_PERIODS)&&(Current_PWM_DutyCycle < Desired_PWM_DutyCycle))  // 1023
		{
		Current_PWM_DutyCycle = Current_PWM_DutyCycle + PWM_BucketStep;  
		if (Current_PWM_DutyCycle > (TIMER_PWM_PERIOD -1)) OCR1A = (TIMER_PWM_PERIOD -1);
		else OCR1A = Current_PWM_DutyCycle;
      
		PWM_Update_Counter = 0x0;
		}
		else if(Current_PWM_DutyCycle >= Desired_PWM_DutyCycle)     
		{
		Motor_Status = Running;
		}
	}
	else if (Motor_Status == Running)
	{
	// Control Loop Duty cycle Update
		
		if (Desired_PWM_DutyCycle > Current_PWM_DutyCycle)
		{
		// Increment duty cycle or change duty cycle in +VE direction
		Current_PWM_DutyCycle = Current_PWM_DutyCycle + MAIN_PWM_BUCKET_DC;
		}
		else if (Desired_PWM_DutyCycle < Current_PWM_DutyCycle)
		{
		// Decrement duty cycle or change duty cycle in -VE direction
		Current_PWM_DutyCycle = Current_PWM_DutyCycle - MAIN_PWM_BUCKET_DC;
		}
		//if Desired_DutyCyle == Current_DutyCycle, dont change dutycycle value
      
		// Update PWM duty cycle values
		OCR1A = Current_PWM_DutyCycle;
	} 
}

ISR(INT1_vect){
	// Hall input interrupts
	//P1IE &= ~(BIT1+BIT2+BIT3);              // Disable Port ISRs -- is it required????????
     
	//Read Hall inputs
	Hall_IN = ((PIND & 0b00001110) >> 1);
     
	PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];
     
	// Start PWM TimerB
	PWM_update(PreDriver_Sequence);
	//P1IFG = 0x0;                             // Clear flags - added........
	//P1IE |= BIT1+BIT2+BIT3;                  // Enable Port ISRs  -- is it required???????
}
ISR(INT2_vect){
	//Hall input interrupts
	//P1IE &= ~(BIT1+BIT2+BIT3);              // Disable Port ISRs -- is it required????????
  
	//Read Hall inputs
	Hall_IN = ((PIND & 0b00001110) >> 1);
  
	PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];
  
	//Start PWM TimerB
	PWM_update(PreDriver_Sequence);
	//P1IFG = 0x0;                             // Clear flags - added........
	//P1IE |= BIT1+BIT2+BIT3;                  // Enable Port ISRs  -- is it required???????

}
ISR(INT3_vect){
	//Hall input interrupts
	//P1IE &= ~(BIT1+BIT2+BIT3);              // Disable Port ISRs -- is it required????????
  
	//Read Hall inputs
	Hall_IN = ((PIND & 0b00001110) >> 1);
  
	PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];
  
	//Start PWM TimerB
	PWM_update(PreDriver_Sequence);
	//P1IFG = 0x0;                             // Clear flags - added........
	//P1IE |= BIT1+BIT2+BIT3;                  // Enable Port ISRs  -- is it required???????

}
void PWM_update (unsigned char Next_Hall_Sequence)
{
	//Hall_U=> P1.3; Hall_V=> P1.2; Hall_W=> P1.1
	//HS3=> P1.3; HS2=> P1.2; HS1=> P1.1
	switch(Next_Hall_Sequence)
	{
	case HS_W|LS_V:            // Hall_IN DIR1_001 DIR0_110
	PORTL = 0b01001000;
	break;
    
	case HS_V|LS_U:           // Hall_IN DIR1_010 DIR0_101
	PORTL = 0b00010010;
	break;
    
	case HS_W|LS_U:            // Hall_IN DIR1_011 DIR0_100
	PORTL = 0b01000010;
	break;
    
	case HS_U|LS_W:            // Hall_IN CCW_100 CW_011
	PORTL = 0b00100100;
	break;
    
	case HS_U|LS_V:            // Hall_IN CCW_101 CW_010
	PORTL = 0b00001100;
	break;
    
	case HS_V|LS_W:            // Hall_IN CCW_110 CW_001
	PORTL = 0b00110000;
	break;
    
	default:
	break;
}
	//TBR = TIMER_PWM_PERIOD-1;
	//TBCCR0 = TIMER_PWM_PERIOD-1;
}
