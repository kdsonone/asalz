/**********************************************************************
* FileName:        ADC.c
* Dependencies:    Header (.h) files if applicable, see below
* Processor:       dsPIC30Fxxxx
* Compiler:        MPLAB® C30 v1.33.00 or higher
*

* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Hrushikesh Vasuki 07/29/05  First release of source file
* George Adamidis   05/29/07  Modified for the Audio Spectrum Analyzer Project 
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
* This file contains two functions - ADC_Init() and _ADCInterrupt().
* Optimized for speed (g-wall-3 compile mode)
**********************************************************************/

#include "p30fxxxx.h"
#include "common.h"
#include "dsp.h"

 extern fractcomplex inputSignal[NUMSAMP] 						/* The inputSignal is defined */
__attribute__ ((section (".ydata, data, ymemory"), 				/* as a complex array containing samples */
aligned (NUMSAMP * 2 *2)));      					        	/* of the real input signal. */

volatile extern int SamplesReadyFlag;

volatile extern fractcomplex * p_inputSignal;

//Functions and Variables with Global Scope:
void ADC_Init(char mode);
void __attribute__((__interrupt__)) _ADCInterrupt(void);

volatile unsigned int * adcPtr;

//Functions:
//ADC_Init() is used to configure A/D to convert 16 samples of 1 input
//channel per interrupt. The A/D is set up for a sampling rate of 7998.6981 Hz (mode=1) or 87252.071 Hz (mode=2)
//Timer3 is used to provide sampling time delay.
//The acquired and converted input pins are AN8 (for mode=1)and AN9 (for mode=2).
void ADC_Init(char mode)
{
        //ADCON1 Register
        //Set up A/D for Automatic Sampling
        //Use Timer3 to provide sampling time
        //Set up A/D conversrion results to be read in 1.15 fractional
        //number format.
        //All other bits to their default state
        ADCON1bits.FORM = 3;
        ADCON1bits.SSRC = 2;
        ADCON1bits.ASAM = 1;

        //ADCON2 Register
         ADCON2bits.VCFG = 3;         //External Vref+, External Vref-
		 ADCON2bits.SMPI = 15;		  //Set up A/D for interrupting after 16 samples get filled in the buffer	

        //ADCON3 Register
        //We would like to set up a sampling rate of 7998.6981Hz or 87252.071 Hz
        //Total Conversion Time= 1/Sampling Rate = 125.02 microseconds or 11.461 microseconds
        //For fosc=117.968MHz, Tcy = 33.91 ns = Instruction Cycle Time
        //We will set up Sampling Time using Timer3 & Tad using ADCS<5:0> bits
        //All other bits to their default state
        //Let's set up ADCS arbitrarily to 38
        //So Tad = Tcy*(ADCS+1)/2 = 661.2 nanoseconds
        //So, the A/D converter will take 14*Tad periods to convert each sample
        ADCON3bits.ADCS = 38;

        //Next, we will to set up Timer 3 to time-out every X=125.02 microseconds (for mode 1) or X=11.461 microseconds (for mode 2)
        //As a result, the module will stop sampling and trigger a conversion
        //on every Timer3 time-out, i.e., 125.02 microseconds or 11.461 microseconds. At that time,
        //the conversion process starts and completes 14*Tad periods later.
        TMR3 = 0x0000;      
	    if (mode==1) PR3 = SAMPCOUNT1;
		if (mode==2) PR3 = SAMPCOUNT2;
		IFS0bits.T3IF = 0;
        IEC0bits.T3IE = 0;
		T3CONbits.TCS = 0;
		T3CONbits.TCKPS=0;

        //ADCHS Register
        //Set up A/D Channel Select Register to convert AN8 (mode=1) or AN9 (mode=2) on Mux A input
        if (mode==1) ADCHS = 0x0008;
		if (mode==2) ADCHS = 0x0009;

	    //ADCSSL Register
        //Channel Scanning is disabled. All bits left to their default state
        ADCSSL = 0x0000;

        //ADPCFG Register
        //Set up channels AN8 and AN9 as analog inputs and configure rest as digital
        //Recall that we configured all A/D pins as digital when code execution
        //entered main() out of reset
        ADPCFG = 0xFFFF;
        ADPCFGbits.PCFG8 = 0;
		ADPCFGbits.PCFG9 = 0;

        //Clear the A/D interrupt flag bit
        IFS0bits.ADIF = 0;

        //Set the A/D interrupt enable bit
        IEC0bits.ADIE = 1;

        //Turn on the A/D converter
        //This is typically done after configuring other registers
        ADCON1bits.ADON = 1;

        //Start Timer 3
        T3CONbits.TON = 1;
}

//_ADCInterrupt() is the A/D interrupt service routine (ISR).
//The routine must have global scope in order to be an ISR.
//The ISR name is chosen from the device linker script.
void __attribute__((__interrupt__)) _ADCInterrupt(void)
{
        //Clear the Timer3 Interrupt Flag
        IFS0bits.T3IF = 0;

        int i = 0;

        //Clear the A/D Interrupt flag bit or else the CPU will
        //keep vectoring back to the ISR
        IFS0bits.ADIF = 0;
        
        //Put the A/D conversion results to inputSignal.real
        adcPtr = &ADCBUF0 ;
        
		for (i=0;i<16;i++)
        	{
               (*p_inputSignal++).real = *adcPtr++;  			 
			}
        
		if (p_inputSignal > &inputSignal[NUMSAMP-1])
			{
			 SamplesReadyFlag++ ; 	// Sampling completed
		     IEC0bits.ADIE = 0;		// Disable ADC interrupt
			}

}

