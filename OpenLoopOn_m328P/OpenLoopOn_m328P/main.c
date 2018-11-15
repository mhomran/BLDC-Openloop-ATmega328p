/*
 * OpenLoop_Texas.c
 *
 * Created: 9/16/2018 2:43:15 PM
 * Created by : Mohamed Hassanin
 */ 
#include <avr/io.h>
#include "main_open_loop.h"
#include "hall_sensor.h"
#include <avr/interrupt.h>
#define F_CPU 16000000UL			// Configure System Clock = DCO = 16Mhz

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
unsigned int Desired_PWM_DutyCycle, Current_PWM_DutyCycle, PWM_BucketStep, PWM_Update_Counter,ADC_Sample_Counter , Temp_DutyCycle;
unsigned char PreDriver_Sequence, Hall_IN, Motor_Status;
unsigned char Motor_status;

// ADC Variables
volatile int curADC, prevADC = 0;
volatile unsigned char SampleADC = false;

// Function definition
void PWM_update (unsigned char Next_Hall_Sequence);
void Start_Motor(void);
void Stop_Motor(void);
void Start_ADC_Conversation(void);

//peripherals
void PCINT1_init(void);
void ADC_init(void);
void Timer1_config(void);


int main(void)
{
	//Variable Initializations
	PWM_Update_Counter = 0x0;
		
	//====DDR
  
	//PWM HS> (pin 9 in Arduino mega) OCR1A PB1 
	DDRB |= 1 << PINB1;
	//HS en PD 3,5,7  LS en PL 2,4,6 
	DDRD = 0b11111100;
  	
	//====mocrocontroller_configuration
	ADC_init();
	PCINT1_init();
	Timer1_config();
	
	//start the motor
	Start_Motor();
	sei();        
  
while (1) 
{
	if(SampleADC == true)
	{
		Start_ADC_Conversation();
		SampleADC = false;
	}
}
}   


void Start_Motor(void)
{
	//trigger_ADC
	ADCSRA |= 1 << ADSC;
	while(ADCSRA & (1 << ADSC));
	prevADC =ADC;
	Temp_DutyCycle = (prevADC/1023.0) * TIMER_PWM_PERIOD ;
	if (Temp_DutyCycle < MIN_PWM_DUTYCYCLE)
	{
			Desired_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;  // < Min DutyCycle %age - latch to min value, 1023
	}
	else if (prevADC >= 1000){  //to prevent closing the switches fast
	TCCR1A &= ~(1 << COM1A1);
	PORTB |= 1 << PINB1;
	}
	else
	{
	Desired_PWM_DutyCycle = Temp_DutyCycle;
	}
	
	
	//big push
	for(int i =0;i < 30; i++)
	{
	//Read Hall inputs
	Hall_IN = (PINC & 0b00000111) ;
	
	//send values 
	PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];
	PWM_update(PreDriver_Sequence);
	}
  
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

void PWM_update (unsigned char Next_Hall_Sequence)
{
	switch(Next_Hall_Sequence)
	{
	case HS_W|LS_V:            // Hall_IN DIR1_001 DIR0_110
	PORTD = 0b10010000;
	break;
    
	case HS_V|LS_U:           // Hall_IN DIR1_010 DIR0_101
	PORTD = 0b00100100;
	break;
    
	case HS_W|LS_U:            // Hall_IN DIR1_011 DIR0_100
	PORTD = 0b10000100;
	break;
    
	case HS_U|LS_W:            // Hall_IN CCW_100 CW_011
	PORTD= 0b01001000;
	break;
    
	case HS_U|LS_V:            // Hall_IN CCW_101 CW_010
	PORTD = 0b00011000;
	break;
    
	case HS_V|LS_W:            // Hall_IN CCW_110 CW_001
	PORTD = 0b0110000;
	break;
    
	default:
	break;
}
}
//========Microcontroller_Configuration======//
void PCINT1_init(void){
		//interrupt
		PCICR |= 1 << PCIE1;
		PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10); //UNO (A0 PC0,A1 PC1,A2 PC2)
		//========================================================// A0 >> yellow || A1 << green || A2 blue

}
void ADC_init(void){
	ADMUX |= 1 << REFS0 | (1 << MUX0) | (1 << MUX1); //AVCC with external capacitor at AREF pin, MUX = 0011, ArduinoUno "A3"
	ADCSRA |= 1 << ADEN | 1 << ADPS2;  //N = 16
}
void Timer1_config(void){
	TCCR1A |= 1 << WGM11;
	TCCR1B |= 1 << WGM12 | 1 << WGM13; //fast mode ICR1 = PWM period
	ICR1 = TIMER_PWM_PERIOD;
	Current_PWM_DutyCycle = MIN_PWM_DUTYCYCLE; // Initial Duty cycle
	OCR1A = Current_PWM_DutyCycle;
	TCCR1A |= 1 << COM1A1;  // clear at compare 
	TIMSK1 |= 1 << TOIE1; // Timer1 overflow interrupt enabled
}

//========ISRs==========//

ISR(PCINT1_vect){
	Hall_IN = (PINC & 0b00000111);
	PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];
	PWM_update(PreDriver_Sequence);	
}

ISR(TIMER1_OVF_vect){
	// heart beat signal = PWM period
	//In computer science, a heartbeat is a periodic signal generated by hardware or software to indicate normal operation or to synchronize other parts of a computer system.[1] #wiki pedia
    ADC_Sample_Counter++;	
	PWM_Update_Counter++;
	if (Motor_Status == StartUp)
	{
		if((PWM_Update_Counter > DUTYCYCLE_CHANGE_PERIODS)&&(Current_PWM_DutyCycle < Desired_PWM_DutyCycle))  // 1023
		{
			Current_PWM_DutyCycle = Current_PWM_DutyCycle + PWM_BucketStep;
			
			//to prevent overflow
			if (Current_PWM_DutyCycle > (TIMER_PWM_PERIOD -1)) 
			{
				OCR1A = (TIMER_PWM_PERIOD -1);
			
			}
			else 
			{
				OCR1A = Current_PWM_DutyCycle;
			}
			
			PWM_Update_Counter = 0x0;
		}
		
		else if(Current_PWM_DutyCycle >= Desired_PWM_DutyCycle)
		{
			Motor_Status = Running;
		}
	}
	else if (Motor_Status == Running)
	{
        if(ADC_Sample_Counter > ADC_SAMPLING_PWM_PERIODS)
        {
	        ADC_Sample_Counter = 0x0;
	        SampleADC = true;
        }		
		
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
		
		// Update PWM duty cycle values
		OCR1A = Current_PWM_DutyCycle;
	}
}
void Start_ADC_Conversation(void){
	//Trigger ADC Sampling
	
	ADCSRA |= 1 << ADSC;
	while(ADCSRA & (1 << ADSC));
	curADC = ADC;
	if ((curADC > (prevADC + 10)) || (curADC < (prevADC - 10))){
		prevADC = curADC;
		Temp_DutyCycle = (prevADC/1025.0) * TIMER_PWM_PERIOD; //1025 to not get to the top value

		TCCR1A |= 1 << COM1A1;
		
		if (Temp_DutyCycle < MIN_PWM_DUTYCYCLE)
		{
			Desired_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;  // < Min DutyCycle %age - latch to min value, 1023
		}
		else if (prevADC >= 1000){  //to prevent closing the switches fast
			TCCR1A &= ~(1 << COM1A1);
			PORTB |= 1 << PINB1;
		}
		else
		{
			Desired_PWM_DutyCycle = Temp_DutyCycle;
		}
	}
	
}