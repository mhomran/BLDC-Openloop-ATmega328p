/*
 * main.h
 *
 * Created: 2/24/2020 10:59:33 PM
 *  Author: Mohamed_Hassanin
 */ 


#ifndef MAIN_H_
#define MAIN_H_

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
void PCINT0_init(void);
void ADC_init(void);
void Timer1_config(void);

#endif /* MAIN_H_ */