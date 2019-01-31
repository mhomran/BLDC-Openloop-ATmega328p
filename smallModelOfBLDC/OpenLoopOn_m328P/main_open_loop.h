/*******************************************************************************
 *  Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *  main_open_loop Header File.
 *  Description: This header file includes user configurable definitions for 
 *    open loop motor control. 
 *
 * Bhargavi Nisarga 
 * Texas Instruments Inc. 
 * Dec 2010
 * Built with IAR Version:4.2

********************************************************************************/

//****************************************************************************//
// Clock Timing Definitions
//****************************************************************************//

#define SYSTEM_FREQ 16000000UL          // (in Hz) System Freq, Timer PWM Time Base
#define PWM_FREQ 5000                  // (in Hz) PWM Freq for switching the H-Bridge FETs 
#define TIMER_PWM_PERIOD  (SYSTEM_FREQ)/PWM_FREQ
                                        // Timer PWM Counts = System Freq/(2xPWM Freq)

#define SPEEDIN_PWM_FACTOR  4           // Calculate this value and define the constant 
                                        // = 2^12/TIMER_PWM_PERIOD, (as ADC12 is being used) rounded off to the closest whole digit

#define DUTYCYCLE_MIN 5                 // (in %age) Min Dutycycle percentage - latching percentage used initially
#define MIN_PWM_DUTYCYCLE		((TIMER_PWM_PERIOD)*(DUTYCYCLE_MIN))/100
                                        // Min PWM Dutycycle; also used for configuring initial dutycycle 

//****************************************************************************//
// Motor Speed Control - Open Loop Definitions
//****************************************************************************//

#define MOTOR_STARTUP_TIME  100         // (in ms) Motor Start-Up time 
#define DUTYCYCLE_CHANGE_PERIODS  10    // #PWM periods dutycycle change is expected, used for start-up routine only
#define STARTUP_STEPS ((unsigned long)PWM_FREQ*MOTOR_STARTUP_TIME)/(DUTYCYCLE_CHANGE_PERIODS*1000)
                                        // For e.g: PWM freq = 15kHz, Startup time=100ms, 
                                        // then #steps for ramping = (15*100)/10 = 150 steps

#define ADC_SAMPLING_PWM_PERIODS 1000   // Every #PWM periods when ADC is sampled
#define MAIN_PWM_BUCKET_PERCENT 2/10    // (in %age) Resolution by which the PWM dutycycle is 
                                        // incremented/decremented after motor start-up
#define MAIN_PWM_BUCKET_DC  (TIMER_PWM_PERIOD*MAIN_PWM_BUCKET_PERCENT)/100
                                        // PWM dutycycle change resolution in timer counts

//****************************************************************************//
// Motor Speed Input Defintions
//****************************************************************************//

#define ANALOG_SPEEDIN
//#define PWM_SPEEDIN

//****************************************************************************//
// Motor Status Definitions
//****************************************************************************//

#define Stopped 0x00
#define StartUp 0x01
#define Running 0x02

//#define DIRECTION_CCW 
#define DIRECTION_CW

#define true 0x01
#define false 0x0


