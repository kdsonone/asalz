/* FileName:        Audio Spectrum.c
 * Dependencies:    Header (.h) files if applicable, see below
 * Processor:       dsPIC30Fxxxx
 * Compiler:        MPLAB® C30 v3.00 or higher
 *
 * REVISION HISTORY:
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author            Date      Comments on this revision
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * George Adamidis  May 29, 2007  First release of source file
 * George Adamidis June 30, 2007  Version 1.0
 * George Adamidis July 01, 2007  Version 1.1
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * ADDITIONAL NOTES:
 * Optimized for speed (g-wall-3 compile mode)
 *
 **********************************************************************/

#include "p30fxxxx.h"
#include "common.h"
#include "dsp.h"

//Macros for Configuration Fuse Registers:
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers, BOR characteristics etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.

_FOSC(CSW_FSCM_OFF & HS2_PLL16);  	    //Run this project using an external crystal
                                		//routed via the PLL in HS2 16x multiplier mode
                                		//For the 14.7456 MHz crystal we will derive a
                                		//throughput of (14.7456/2)e+6*16/4 = 29.49 MIPS(Fcy)
                                		//,~33.9 nanoseconds instruction cycle time(Tcy).
_FWDT(WDT_OFF);                 		//Turn off the Watch-Dog Timer.
_FBORPOR(MCLR_EN & PWRT_OFF);   		//Enable MCLR reset pin and turn off the
                                		//power-up timers.
_FGS(CODE_PROT_OFF);            		//Disable Code Protection



///////////////////////////////////////////////////////////////////////////////
/*                     Variables-definitions                                 */
///////////////////////////////////////////////////////////////////////////////
fractcomplex inputSignal[NUMSAMP] 												/* The inputSignal is defined */
__attribute__ ((section ("ymemory"),far, 								/* as a complex array containing samples */
aligned (NUMSAMP * 2 *2 )));      			        							/* of the real input signal. */
														
fractcomplex twiddleFactors[NUMSAMP/2]  	               					 	/* Declare Twiddle Factor coefficients array in X-space*/
__attribute__ ((section (".xbss, bss, xmemory"),aligned (NUMSAMP*2)));      	/* Twiddle Factors are needed for the FFT computation  */  

// Contains a normalized to 0.5  Hamming window 
fractional HammingFactors[NUMSAMP*2] __attribute__ ((section (".xbss, bss, xmemory"), aligned (NUMSAMP*2)));

// Contains the discrete spectral-Power of the input signal
fractional Power_Vector[NUMSAMP] __attribute__ ((section (".xbss, bss, xmemory"), aligned (NUMSAMP)));

volatile fractcomplex * p_inputSignal = &inputSignal[0] ;	// Points to the inputSignal

fractional Bar[20];                                    	    // The 20 samples whitch will be displayed in a 20-bars, led display 
                                                	        // Each sample corresponds to the center frequency of 31, 63, 94, 126, 170, 230, 310, 420, 563, 760 Hz and
															// 1.02, 1.37, 1.87, 2.55, 3.4, 4.6, 6.15, 8.36, 11.2 and 15KHz respectively

fractional PeakHold[20];    								// The 20 Peak Hold values (they are used for the Bars+Peaks display mode.
unsigned int Rain[20];										// The 20 "Rain" values (they are used for the Climping Bars+Rain display mode.
unsigned int SnapToPeak_Bar[20];							// The 20 Snap-to-Peak values (they are used for the Climping-Bars display mode.
long counter;                                   			// Time counter - >Counts display's frames (it is useful for the Bars+Peaks, and the Climping-Bars display mode. 						 
long counter2;                                   			// Time counter -> Counts display's frames (it is useful for the "2-Climbing Bars +Rain" display mode. 
	
volatile unsigned int SamplesReadyFlag;						// A/D converter's Samples-Ready flag




/////////////////////////////////////////////////////////////////////////
/*                  Functions - definitions                            */
/////////////////////////////////////////////////////////////////////////
void ConvertToComplex(void);										/* This function used to convert the real input sampling sequence to a Complex array */	
																	/* by simply zero out the imaginary part of each data sample          */ 
int main (void);
extern void ADC_Init(char mode);									//ADC_Init() is used to configure A/D to convert 16 samples of 1 input
																	//channel per interrupt. The A/D is set up for a sampling rate of 7998.6981 Hz (mode=1) or 87252.071 Hz (mode=2)
																	//Timer3 is used to provide sampling time delay.
																	//The acquired and converted input pins are AN8 (for mode=1)and AN9 (for mode=2).

void Filter_Bank(unsigned int Samplesflag);							// Applies a Filter-Bank after the FFT and   
																	// Finds the Bar samples which will be displayed in a 20-bars led display
																	// SamplesFlag=1 OR 3 for 7998.6981Hz or 87252.071 Hz sampling rate

void FindPeakHold(void);											// Finds the PeakHold samples which will be displayed in the "Bars+Peaks" mode 
void FindSnapToPeak(void);											// Finds the Snap-to-peak samples which will be displayed in the 0-Climbing-Bars display mode 	
void initializeIO(void);											// Initializes IO pins
void initialize_rest(void);											// Initializes arrays
void FindClimbingBars_Rain(void);									// Finds the Snap-to-peak[] and Rain[] samples which will be displayed in the "2-Climbing-Bars+Rain" display mode

unsigned int display_mode(void);                                    // Returns display mode (0-3) according to S1 switch 
																	// There are 4 display modes: 0-Bars, 1-Climbing-Bars+Rain, 2-Bars+Peaks, 3-Climbing-Bars

void display (unsigned int dis_mode);                               // Display routine

void Set_Y (unsigned int x);									    // Sets Y for the x bargraph-Bar according to Bar[x] (for the 1-Bars display mode)
void Set_Y_Peak (unsigned int z);									// Sets Y for the z bargraph-Bar according to Bar[z] and PeakHold[z] (for the 3-Bars+Peaks display mode)
void Set_Y_SnapTP (unsigned int n);									// Sets Y for the n bargraph-Bar according to SnapToPeak_Bar[] (for the 0-Climping Bars)
void Set_Y_Rain(unsigned int j);			                        // Sets Y for the j bargraph-Bar according to SnapToPeak_Bar[] and Rain[] (for the 2-Climping Bars+Rain display mode)

fractional add_Elements(int num, fractional* point);				//  Returns the sum of num subsequent real elements of a fractional vector (point, points to vector)   						



/////////////////////////////////////////////////////////////////////////
/*                      main routine starts here                       */
/////////////////////////////////////////////////////////////////////////
int main (void)
{

	// Initialization
	initializeIO();
    initialize_rest();
 	
        while (1)               									//Loop Endlessly 
        {                      					    				//from this point on. (interrupt driven state-machine)
           	    
				if (SamplesReadyFlag==1)							// NUMSAMP samples have been acquired (sampling rate:7998.6981 Hz)
                 {
 
  			      /*Apply HammingFactors to the input sample array and store the resulting windowed vector in place*/
				   VectorMultiply (NUMSAMP*2,&inputSignal[0].real,&inputSignal[0].real,&HammingFactors[0]);
				 	 				 
       			  /* Perform FFT operation */	
				  FFTComplexIP (LOG2_BLOCK_LENGTH, &inputSignal[0], &twiddleFactors[0], COEFFS_IN_DATA);  
				
				  /* Store output samples in bit-reversed order of their addresses */
				  BitReverseComplex (LOG2_BLOCK_LENGTH, &inputSignal[0]);

	              /* Compute the square magnitude of the complex FFT output array so we have a Real output vector */
	              SquareMagnitudeCplx(NUMSAMP, &inputSignal[0], &Power_Vector[0]);
				  				  		
				  Filter_Bank(SamplesReadyFlag);	        // Find the Bar[0-9] samples which will be displayed in a 20-bars led display
				 	  
				  p_inputSignal = &inputSignal[0];          //Initialize pointer for the A/D convertion
        		  SamplesReadyFlag = 2;                     //Initialize A/D converter's Samples-Ready flag for 87252.071Hz sampling rate  
                 		  
				  ADC_Init(2);            			        //Initialize the A/D converter for 87252.071 Hz sampling rate
                

				 }

				if (SamplesReadyFlag==3)					// NUMSAMP samples have been acquired (sampling rate:87252.071 Hz)
                  {
				  
			      /*Apply HammingFactors to the input sample array and store the resulting windowed vector in place*/
	        	  VectorMultiply (NUMSAMP*2,&inputSignal[0].real,&inputSignal[0].real,&HammingFactors[0]);
				 		  
       			  /* Perform FFT operation */	
				  FFTComplexIP (LOG2_BLOCK_LENGTH, &inputSignal[0], &twiddleFactors[0], COEFFS_IN_DATA);  
				
				  /* Store output samples in bit-reversed order of their addresses */
				  BitReverseComplex (LOG2_BLOCK_LENGTH, &inputSignal[0]);

	              /* Compute the square magnitude of the complex FFT output array so we have a Real output vector */
	              SquareMagnitudeCplx(NUMSAMP, &inputSignal[0], &Power_Vector[0]);
				    
				  Filter_Bank(SamplesReadyFlag);	          // Find the Bar[10-19] samples which will be displayed in a 20-bars led display	
				  
				  p_inputSignal = &inputSignal[0];            //Initialize pointer for the A/D convertion
        		  SamplesReadyFlag = 0;                       //Initialize A/D converter's Samples-Ready flag for 7998.6981 Hz sampling rate  
                  
				 			
				  ADC_Init(1);            			          //Initialize the A/D converter for 7998.6981 Hz sampling rate
				  
				  }
        
		
      		display (display_mode());	  					// Dislay routine (for display_mode 0, 1, 2, 3)  

        }
        return 0;

}



/////////////////////////////////////////////////////////////////////////////////////////
/*  This function used to convert the real input sampling sequence to a Complex array  */
/////////////////////////////////////////////////////////////////////////////////////////
void ConvertToComplex()
{
	int i;
	p_inputSignal=&inputSignal[NUMSAMP-1];
	for ( i = NUMSAMP; i > 0; i-- ) 									
	{																
	     /* Zero out the imaginary part of each data sample */
		(*p_inputSignal--).imag = 0x0000;			      
	}
}


//////////////////////////////////////////////////////////////////////////////
/* Finds the Bar samples which will be displayed in a 20-bars led display   */
/* Each sample corresponds to the center frequency of 31, 63, 94, 126, 170, */
/* 230, 310, 420, 563, 760 Hz and 1.02, 1.37, 1.87, 2.55, 3.4, 4.6, 6.15,   */
/* 8.36, 11.2 and 15KHz respectively                                        */ 
//////////////////////////////////////////////////////////////////////////////
void Filter_Bank(unsigned int Samplesflag)
{

	if (Samplesflag==1)										// 7998.6981 Hz Sampling rate
	{	
	   
		Bar[0]=Power_Vector[1];	 							//at 31Hz
		Bar[1]=Power_Vector[2];								//at 63 Hz
		Bar[2]=Power_Vector[3];	 							//at 94Hz
		Bar[3]=Power_Vector[4];								//at 125 Hz
		Bar[4]=add_Elements(2, &Power_Vector[5]); 			//at 170 Hz
		Bar[5]=add_Elements(2, &Power_Vector[7]); 			//at 230 Hz
		Bar[6]=add_Elements(3, &Power_Vector[9]); 			//at 310 Hz
		Bar[7]=add_Elements(4, &Power_Vector[12]); 			//at 420 Hz
		Bar[8]=add_Elements(5, &Power_Vector[16]); 			//at 563 Hz
		Bar[9]=add_Elements(8, &Power_Vector[21]); 			//at 760 Hz
		
	}



	if (Samplesflag==3)										// 87252.071 Hz Sampling rate
	{	
		
		Bar[10]=Power_Vector[3];		 					//at 1024 Hz
		Bar[11]=Power_Vector[4];							//at 1370 Hz
		Bar[12]=add_Elements(2, &Power_Vector[5]); 			//at 1870 Hz
		Bar[13]=add_Elements(2, &Power_Vector[7]); 			//at 2550 Hz
		Bar[14]=add_Elements(3, &Power_Vector[9]); 			//at 3400 Hz
		Bar[15]=add_Elements(4, &Power_Vector[12]); 		//at 4600 Hz
		Bar[16]=add_Elements(5, &Power_Vector[16]); 		//at 6150 Hz
		Bar[17]=add_Elements(8, &Power_Vector[21]); 		//at 8360 Hz
		Bar[18]=add_Elements(9, &Power_Vector[29]); 		//at 11200 Hz
		Bar[19]=add_Elements(13,&Power_Vector[38]); 		//at 15000 Hz
			
	}

}



//////////////////////////////////////////////////////////////////////////
/*  					  Initiliazes arrays  					  	    */
/* 						  Runs at Start-up								*/
//////////////////////////////////////////////////////////////////////////
void initialize_rest(void)
{
	int i;
		
		/* Generate TwiddleFactor Coefficients */
		TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactors[0], 0);	/* We need to do this only once at start-up */
	
		/* Generate a Hamming Window*/
		HammingInit(NUMSAMP*2,&HammingFactors[0]);
		
		/* Scale by 0.5 the Hamming Window because the FFT function requires input data to be in the fractional fixed-point range [-0.5, +0.5]*/
		VectorScale (NUMSAMP*2,&HammingFactors[0],&HammingFactors[0],0x4000);
		
		/* Zero the Complex part of the input sample array*/
		 ConvertToComplex();

		//Initialize Peak Hold[], Bar[], SnapToPeak[] and Rain[]  arrays 
		for (i=0; i<20; i++)
		{
			PeakHold[i]=0;
			Bar[i]=0;
			SnapToPeak_Bar[i]=0;
			Rain[i]=0;
		}
		
		//Initialize counter variables
		counter=0;
		counter2=0;

		p_inputSignal = &inputSignal[0];              				//Initialize pointer for the A/D convertion
        SamplesReadyFlag = 0;                       				//Initialize A/D converter's Samples-Ready flag for 7998.6981 Hz sampling rate 
        ADC_Init(1);            									//Initialize the A/D converter for 7998.6981 Hz sampling rate - A/D conversion is interrupt driven from this point on 
																	// until SamplesReadyFlag

}											



//////////////////////////////////////////////////////////////////////////
/*  					  Initiliazes IO pins   						*/
/* 						  Runs at Start-up								*/
//////////////////////////////////////////////////////////////////////////
void initializeIO(void)		
{
	TRISBbits.TRISB2=0;
	TRISBbits.TRISB3=0;
	TRISBbits.TRISB4=0;
	TRISBbits.TRISB5=0;
	TRISBbits.TRISB6=0;
	TRISBbits.TRISB7=0;
	TRISBbits.TRISB10=0;
	TRISBbits.TRISB11=0;
	TRISBbits.TRISB12=0;
	TRISBbits.TRISB13=0;
	TRISBbits.TRISB14=0;
	TRISBbits.TRISB15=0;

	TRISCbits.TRISC13=1;
	TRISCbits.TRISC14=1;

	TRISDbits.TRISD0=0;
	TRISDbits.TRISD1=0;
	TRISDbits.TRISD2=0;
	TRISDbits.TRISD3=0;
	TRISDbits.TRISD4=0;
	TRISDbits.TRISD5=0;
	TRISDbits.TRISD6=0;
	TRISDbits.TRISD7=0;
	TRISDbits.TRISD8=0;
	TRISDbits.TRISD9=0;
	TRISDbits.TRISD10=0;
	TRISDbits.TRISD11=0;

	TRISFbits.TRISF0=0;
	TRISFbits.TRISF1=0;
	TRISFbits.TRISF2=0;
	TRISFbits.TRISF3=0;
	TRISFbits.TRISF4=0;
	TRISFbits.TRISF5=0;
	TRISFbits.TRISF6=0;

	TRISGbits.TRISG0=0;
	TRISGbits.TRISG1=0;
	TRISGbits.TRISG2=0;
	TRISGbits.TRISG3=0;
	TRISGbits.TRISG6=0;
	TRISGbits.TRISG12=0;
	TRISGbits.TRISG13=0;
	TRISGbits.TRISG14=0;
	TRISGbits.TRISG15=0;

	
   // Turns off all X
	X1=off;
	X2=off;
	X3=off;
	X4=off;
	X5=off;
	X6=off;
	X7=off;
	X8=off;
	X9=off;
	X10=off;
	X11=off;
	X12=off;
	X13=off;
	X14=off;
	X15=off;
	X16=off;
	X17=off;
	X18=off;
	X19=off;
	X20=off;

    // Turns off all Y
	Y1=off;
	Y2=off;
	Y3=off;
	Y4=off;
	Y5=off;
	Y6=off;
	Y7=off;
	Y8=off;
	Y9=off;
	Y10=off;
	Y11=off;
	Y12=off;
	Y13=off;
	Y14=off;
	Y15=off;
	Y16=off;
	Y17=off;
	Y18=off;
	Y19=off;
	Y20=off;

}



////////////////////////////////////////////////////////////////////////////////////////////
/*  		Returns display mode (1-4) according to S1 switch 							  */
/* There are 4 display modes: 1-Bars, 2-Climbing Bars+Rain, 3-Bars+Peaks, 4 Climping-Bars */
////////////////////////////////////////////////////////////////////////////////////////////
unsigned int display_mode(void)
{

unsigned int a,b;

a=0;
b=0;

if (ControlBit1) a++;
if (ControlBit2) b++;

a=a+(2*b);
return a;
}


////////////////////////////////////////////////////////////////////////////////////////////
/*  		 			 	Display routine											      */
/* There are 4 display modes: 1-Bars, 2-Climbing Bars+Rain, 3-Bars+Peaks, 4 Climping-Bars */
////////////////////////////////////////////////////////////////////////////////////////////
void display (unsigned int dis_mode)
{

int i;

		switch (dis_mode)
			{
           case 1:						// Display Bar-graph

									
		 	Set_Y(0);				
			X1=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X1=off;
			Set_Y(1);
			X2=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X2=off;
			Set_Y(2);
			X3=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X3=off;
			Set_Y(3);				
			X4=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X4=off;
			Set_Y(4);			
			X5=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X5=off;
			Set_Y(5);				
			X6=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X6=off;
			Set_Y(6);				
			X7=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X7=off;
			Set_Y(7);				
			X8=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X8=off;
			Set_Y(8);				
			X9=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X9=off;
			Set_Y(9);				
			X10=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X10=off;
			Set_Y(10);				
			X11=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X11=off;
			Set_Y(11);				
			X12=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X12=off;
			Set_Y(12);				
			X13=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X13=off;
			Set_Y(13);				
			X14=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X14=off;
			Set_Y(14);				
			X15=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X15=off;
			Set_Y(15);				
			X16=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X16=off;
			Set_Y(16);
			X17=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X17=off;
			Set_Y(17);
			X18=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X18=off;
			Set_Y(18);
			X19=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X19=off;
			Set_Y(19);
			X20=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X20=off;
			Set_Y(0);
			
			

		 break;
		
		
 		 case 2:						// Display Climbing Bars+Rain graph

   			Set_Y_Rain(0);				
			X1=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X1=off;
			Set_Y_Rain(1);
			X2=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X2=off;
			Set_Y_Rain(2);
			X3=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X3=off;
			Set_Y_Rain(3);				
			X4=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X4=off;
			Set_Y_Rain(4);			
			X5=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X5=off;
			Set_Y_Rain(5);				
			X6=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X6=off;
			Set_Y_Rain(6);				
			X7=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X7=off;
			Set_Y_Rain(7);				
			X8=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X8=off;
			Set_Y_Rain(8);				
			X9=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X9=off;
			Set_Y_Rain(9);				
			X10=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X10=off;
			Set_Y_Rain(10);				
			X11=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X11=off;
			Set_Y_Rain(11);				
			X12=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X12=off;
			Set_Y_Rain(12);				
			X13=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X13=off;
			Set_Y_Rain(13);				
			X14=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X14=off;
			Set_Y_Rain(14);				
			X15=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X15=off;
			Set_Y_Rain(15);				
			X16=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X16=off;
			Set_Y_Rain(16);
			X17=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X17=off;
			Set_Y_Rain(17);
			X18=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X18=off;
			Set_Y_Rain(18);
			X19=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X19=off;
			Set_Y_Rain(19);
			X20=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X20=off;
			FindClimbingBars_Rain();
			Set_Y_Rain(0);   
            counter++; 
			counter2++;                                                     
		 break;

 		
		 case 3:						// Display Bar-graph + Peaks
         	 	Set_Y_Peak(0);				
			X1=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X1=off;
			Set_Y_Peak(1);
			X2=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X2=off;
			Set_Y_Peak(2);
			X3=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X3=off;
			Set_Y_Peak(3);				
			X4=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X4=off;
			Set_Y_Peak(4);			
			X5=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X5=off;
			Set_Y_Peak(5);				
			X6=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X6=off;
			Set_Y_Peak(6);				
			X7=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X7=off;
			Set_Y_Peak(7);				
			X8=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X8=off;
			Set_Y_Peak(8);				
			X9=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X9=off;
			Set_Y_Peak(9);				
			X10=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X10=off;
			Set_Y_Peak(10);				
			X11=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X11=off;
			Set_Y_Peak(11);				
			X12=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X12=off;
			Set_Y_Peak(12);				
			X13=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X13=off;
			Set_Y_Peak(13);				
			X14=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X14=off;
			Set_Y_Peak(14);				
			X15=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X15=off;
			Set_Y_Peak(15);				
			X16=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X16=off;
			Set_Y_Peak(16);
			X17=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X17=off;
			Set_Y_Peak(17);
			X18=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X18=off;
			Set_Y_Peak(18);
			X19=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X19=off;
			Set_Y_Peak(19);
			X20=on;
					for (i=0;i<BarDelay2;i++)
					{
						Nop();
					}
		  	X20=off;
			Set_Y_Peak(0);   
            counter++;                    
                              
		 break;


		 case 0:						// Display Climping-Bars graph
		                    	   
  			Set_Y_SnapTP(0);				
			X1=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X1=off;
			Set_Y_SnapTP(1);
			X2=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X2=off;
			Set_Y_SnapTP(2);
			X3=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X3=off;
			Set_Y_SnapTP(3);				
			X4=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X4=off;
			Set_Y_SnapTP(4);			
			X5=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X5=off;
			Set_Y_SnapTP(5);				
			X6=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X6=off;
			Set_Y_SnapTP(6);				
			X7=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X7=off;
			Set_Y_SnapTP(7);				
			X8=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X8=off;
			Set_Y_SnapTP(8);				
			X9=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X9=off;
			Set_Y_SnapTP(9);				
			X10=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X10=off;
			Set_Y_SnapTP(10);				
			X11=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X11=off;
			Set_Y_SnapTP(11);				
			X12=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X12=off;
			Set_Y_SnapTP(12);				
			X13=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X13=off;
			Set_Y_SnapTP(13);				
			X14=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X14=off;
			Set_Y_SnapTP(14);				
			X15=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X15=off;
			Set_Y_SnapTP(15);				
			X16=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X16=off;
			Set_Y_SnapTP(16);
			X17=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X17=off;
			Set_Y_SnapTP(17);
			X18=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X18=off;
			Set_Y_SnapTP(18);
			X19=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X19=off;
			Set_Y_SnapTP(19);
			X20=on;
					for (i=0;i<BarDelay;i++)
					{
						Nop();
					}
		  	X20=off;
			FindSnapToPeak();
			Set_Y_SnapTP(0);   
            counter++;                             
		 break;
	
	}
}


//////////////////////////////////////////////////////////////////////////
/*       	Sets Y for the x bargraph-Bar according to Bar[x]	        */
//////////////////////////////////////////////////////////////////////////
void Set_Y (unsigned int x)	
{
		 
		PORTD=0;
		PORTG=0;
		PORTF=0;
		PORTB=0;
		
			
		if (Bar[x]>Threshold1-1)  Y1=on;
		if (Bar[x]>Threshold2-1)  Y2=on;
	    if (Bar[x]>Threshold3-1)  Y3=on;
		if (Bar[x]>Threshold4-1)  Y4=on;
		if (Bar[x]>Threshold5-1)  Y5=on;
		if (Bar[x]>Threshold6-1)  Y6=on;
		if (Bar[x]>Threshold7-1)  Y7=on;
	    if (Bar[x]>Threshold8-1)  Y8=on;
		if (Bar[x]>Threshold9-1)  Y9=on;
		if (Bar[x]>Threshold10-1) Y10=on;
		if (Bar[x]>Threshold11-1) Y11=on;
		if (Bar[x]>Threshold12-1) Y12=on;
	    if (Bar[x]>Threshold13-1) Y13=on;
		if (Bar[x]>Threshold14-1) Y14=on;
		if (Bar[x]>Threshold15-1) Y15=on;
		if (Bar[x]>Threshold16-1) Y16=on;
		if (Bar[x]>Threshold17-1) Y17=on;
	    if (Bar[x]>Threshold18-1) Y18=on;
		if (Bar[x]>Threshold19-1) Y19=on;
		if (Bar[x]>Threshold20-1) Y20=on;
		  
}


////////////////////////////////////////////////////////////////////////////////
/*Finds the PeakHold samples which will be displayed in the "Bars+Peaks" mode */
////////////////////////////////////////////////////////////////////////////////

void FindPeakHold(void)
{
	int i;
	
		for (i=0;i<20;i++)						// Max - hold function (stores Peaks at PeakHold[] array)  
			{
				if (Bar[i]>PeakHold[i]) PeakHold[i]=Bar[i];
			}	 

		if (counter>PeakHold_timeout)   		// If counter exceeds timeout then refresh PeakHold[]
			for (i=0;i<20;i++)					// Stores current Peaks at PeakHold[] array  
				{
					PeakHold[i]=Bar[i];
					counter=0;
				}	 	 

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*Sets Y for the z bargraph-Bar according to Bar[z] and PeakHold[z] (for the 3-Bars+Peaks display mode)*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Set_Y_Peak (unsigned int z)
{

		PORTD=0;
		PORTG=0;
		PORTF=0;
		PORTB=0;
		
		//Sets Bar
		if (Bar[z]>Threshold1-1)  Y1=on;
		if (Bar[z]>Threshold2-1)  Y2=on;
	    if (Bar[z]>Threshold3-1)  Y3=on;
		if (Bar[z]>Threshold4-1)  Y4=on;
		if (Bar[z]>Threshold5-1)  Y5=on;
		if (Bar[z]>Threshold6-1)  Y6=on;
		if (Bar[z]>Threshold7-1)  Y7=on;
	    if (Bar[z]>Threshold8-1)  Y8=on;
		if (Bar[z]>Threshold9-1)  Y9=on;
		if (Bar[z]>Threshold10-1) Y10=on;
		if (Bar[z]>Threshold11-1) Y11=on;
		if (Bar[z]>Threshold12-1) Y12=on;
	    if (Bar[z]>Threshold13-1) Y13=on;
		if (Bar[z]>Threshold14-1) Y14=on;
		if (Bar[z]>Threshold15-1) Y15=on;
		if (Bar[z]>Threshold16-1) Y16=on;
		if (Bar[z]>Threshold17-1) Y17=on;
	    if (Bar[z]>Threshold18-1) Y18=on;
		if (Bar[z]>Threshold19-1) Y19=on;
		if (Bar[z]>Threshold20-1) Y20=on;

		//Sets Peak
		FindPeakHold();
		if (PeakHold[z]>Threshold20-1)  
			{
			 Y20=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold19-1)  
			{
			 Y19=on;
			 return;
			}							

		if (PeakHold[z]>Threshold18-1)  
			{
			 Y18=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold17-1)  
			{
			 Y17=on;
			 return;
			}					

		if (PeakHold[z]>Threshold16-1)  
			{
			 Y16=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold15-1)  
			{
			 Y15=on;
			 return;
			}							

		if (PeakHold[z]>Threshold14-1)  
			{
			 Y14=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold13-1)  
			{
			 Y13=on;
			 return;
			}				


		if (PeakHold[z]>Threshold12-1)  
			{
			 Y12=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold11-1)  
			{
			 Y11=on;
			 return;
			}							

		if (PeakHold[z]>Threshold10-1)  
			{
			 Y10=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold9-1)  
			{
			 Y9=on;
			 return;
			}					

		if (PeakHold[z]>Threshold8-1)  
			{
			 Y8=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold7-1)  
			{
			 Y7=on;
			 return;
			}							

		if (PeakHold[z]>Threshold6-1)  
			{
			 Y6=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold5-1)  
			{
			 Y5=on;
			 return;
			}				
		if (PeakHold[z]>Threshold4-1)  
			{
			 Y4=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold3-1)  
			{
			 Y3=on;
			 return;
			}							

		if (PeakHold[z]>Threshold2-1)  
			{
			 Y2=on;
			 return;
			}
		
		
		if (PeakHold[z]>Threshold1-1)  
			{
			 Y1=on;
			 return;
			}				
}


/////////////////////////////////////////////////////////////////////////////////////////////////
// Returns the sum of num subsequent elements of a fractional vector (point, points to vector) // 	
/////////////////////////////////////////////////////////////////////////////////////////////////
fractional add_Elements(int num, fractional* point)
{
	int i;
	fractional sum=0;

	for(i=0; i<num; i++)
		{
			sum += (*point++); 

		}
	return sum;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
//   Finds the Snap-to-peak samples which will be displayed in the Climbing-Bars dispaly mode  // 	
/////////////////////////////////////////////////////////////////////////////////////////////////
void FindSnapToPeak(void)
{
unsigned int slow[20];
int i;
	for(i=0;i<20;i++)
		{
			if (Bar[i]<=Threshold1-1)  slow[i]=0;
			if (Bar[i]>Threshold1-1)  slow[i]=1;
			if (Bar[i]>Threshold2-1)  slow[i]=2;
			if (Bar[i]>Threshold3-1)  slow[i]=3;
			if (Bar[i]>Threshold4-1)  slow[i]=4;
			if (Bar[i]>Threshold5-1)  slow[i]=5;
			if (Bar[i]>Threshold6-1)  slow[i]=6;
			if (Bar[i]>Threshold7-1)  slow[i]=7;
			if (Bar[i]>Threshold8-1)  slow[i]=8;
			if (Bar[i]>Threshold9-1)  slow[i]=9;
			if (Bar[i]>Threshold10-1)  slow[i]=10;
			if (Bar[i]>Threshold11-1)  slow[i]=11;
			if (Bar[i]>Threshold12-1)  slow[i]=12;
			if (Bar[i]>Threshold13-1)  slow[i]=13;
			if (Bar[i]>Threshold14-1)  slow[i]=14;
			if (Bar[i]>Threshold15-1)  slow[i]=15;
			if (Bar[i]>Threshold16-1)  slow[i]=16;
			if (Bar[i]>Threshold17-1)  slow[i]=17;
			if (Bar[i]>Threshold18-1)  slow[i]=18;
			if (Bar[i]>Threshold19-1)  slow[i]=19;
			if (Bar[i]>Threshold20-1)  slow[i]=20;
		}
			
			if (counter>Snap_To_Peak_Timeout)
				{
					for(i=0;i<20;i++)
						{
				  			if (slow[i]>SnapToPeak_Bar[i])SnapToPeak_Bar[i]=slow[i];
				  			if (slow[i]<SnapToPeak_Bar[i])SnapToPeak_Bar[i]--;
				 		}	  	
				 	counter=0;
				}


}											


////////////////////////////////////////////////////////////////////////////////////////////////////
// 		 Sets Y for the n bargraph-Bar according to SnapToPeak_Bar (for the 0-Climbing Bars mode) // 	
////////////////////////////////////////////////////////////////////////////////////////////////////
void Set_Y_SnapTP (unsigned int n)
{

		PORTD=0;
		PORTG=0;
		PORTF=0;
		PORTB=0;
		
			
		if (SnapToPeak_Bar[n]>0) Y1=on;
		if (SnapToPeak_Bar[n]>1) Y2=on;
		if (SnapToPeak_Bar[n]>2) Y3=on;
		if (SnapToPeak_Bar[n]>3) Y4=on;
		if (SnapToPeak_Bar[n]>4) Y5=on;
		if (SnapToPeak_Bar[n]>5) Y6=on;
		if (SnapToPeak_Bar[n]>6) Y7=on;
		if (SnapToPeak_Bar[n]>7) Y8=on;
		if (SnapToPeak_Bar[n]>8) Y9=on;
		if (SnapToPeak_Bar[n]>9) Y10=on;
		if (SnapToPeak_Bar[n]>10) Y11=on;
		if (SnapToPeak_Bar[n]>11) Y12=on;
		if (SnapToPeak_Bar[n]>12) Y13=on;
		if (SnapToPeak_Bar[n]>13) Y14=on;
		if (SnapToPeak_Bar[n]>14) Y15=on;
		if (SnapToPeak_Bar[n]>15) Y16=on;
		if (SnapToPeak_Bar[n]>16) Y17=on;
		if (SnapToPeak_Bar[n]>17) Y18=on;
		if (SnapToPeak_Bar[n]>18) Y19=on;
		if (SnapToPeak_Bar[n]>19) Y20=on;


}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Sets Y for the j bargraph-Bar according to SnapToPeak_Bar[] and Rain[] (for the 2-Climping Bars+Rain display mode)//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Set_Y_Rain(unsigned int j)
{

		PORTD=0;
		PORTG=0;
		PORTF=0;
		PORTB=0;
		
		if (SnapToPeak_Bar[j]>0) Y1=on;
		if (SnapToPeak_Bar[j]>1) Y2=on;
		if (SnapToPeak_Bar[j]>2) Y3=on;
		if (SnapToPeak_Bar[j]>3) Y4=on;
		if (SnapToPeak_Bar[j]>4) Y5=on;
		if (SnapToPeak_Bar[j]>5) Y6=on;
		if (SnapToPeak_Bar[j]>6) Y7=on;
		if (SnapToPeak_Bar[j]>7) Y8=on;
		if (SnapToPeak_Bar[j]>8) Y9=on;
		if (SnapToPeak_Bar[j]>9) Y10=on;
		if (SnapToPeak_Bar[j]>10) Y11=on;
		if (SnapToPeak_Bar[j]>11) Y12=on;
		if (SnapToPeak_Bar[j]>12) Y13=on;
		if (SnapToPeak_Bar[j]>13) Y14=on;
		if (SnapToPeak_Bar[j]>14) Y15=on;
		if (SnapToPeak_Bar[j]>15) Y16=on;
		if (SnapToPeak_Bar[j]>16) Y17=on;
		if (SnapToPeak_Bar[j]>17) Y18=on;
		if (SnapToPeak_Bar[j]>18) Y19=on;
		if (SnapToPeak_Bar[j]>19) Y20=on;


		if (Rain[j]==1) Y1=on;
		if (Rain[j]==2) Y2=on;
		if (Rain[j]==3) Y3=on;
		if (Rain[j]==4) Y4=on;
		if (Rain[j]==5) Y5=on;
		if (Rain[j]==6) Y6=on;
		if (Rain[j]==7) Y7=on;
		if (Rain[j]==8) Y8=on;
		if (Rain[j]==9) Y9=on;
		if (Rain[j]==10) Y10=on;
		if (Rain[j]==11) Y11=on;
		if (Rain[j]==12) Y12=on;
		if (Rain[j]==13) Y13=on;
		if (Rain[j]==14) Y14=on;
		if (Rain[j]==15) Y15=on;
		if (Rain[j]==16) Y16=on;
		if (Rain[j]==17) Y17=on;
		if (Rain[j]==18) Y18=on;
		if (Rain[j]==19) Y19=on;
		if (Rain[j]==20) Y20=on;


}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Finds the Snap-to-peak[] and Rain[] samples which will be displayed in the "2-Climbing-Bars+Rain" dispay mode  //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FindClimbingBars_Rain(void)
{

unsigned int slow[20];
int i;
	for(i=0;i<20;i++)
		{
			if (Bar[i]<=Threshold1-1)  slow[i]=0;
			if (Bar[i]>Threshold1-1)  slow[i]=1;
			if (Bar[i]>Threshold2-1)  slow[i]=2;
			if (Bar[i]>Threshold3-1)  slow[i]=3;
			if (Bar[i]>Threshold4-1)  slow[i]=4;
			if (Bar[i]>Threshold5-1)  slow[i]=5;
			if (Bar[i]>Threshold6-1)  slow[i]=6;
			if (Bar[i]>Threshold7-1)  slow[i]=7;
			if (Bar[i]>Threshold8-1)  slow[i]=8;
			if (Bar[i]>Threshold9-1)  slow[i]=9;
			if (Bar[i]>Threshold10-1)  slow[i]=10;
			if (Bar[i]>Threshold11-1)  slow[i]=11;
			if (Bar[i]>Threshold12-1)  slow[i]=12;
			if (Bar[i]>Threshold13-1)  slow[i]=13;
			if (Bar[i]>Threshold14-1)  slow[i]=14;
			if (Bar[i]>Threshold15-1)  slow[i]=15;
			if (Bar[i]>Threshold16-1)  slow[i]=16;
			if (Bar[i]>Threshold17-1)  slow[i]=17;
			if (Bar[i]>Threshold18-1)  slow[i]=18;
			if (Bar[i]>Threshold19-1)  slow[i]=19;
			if (Bar[i]>Threshold20-1)  slow[i]=20;
		}
			
			if (counter>Snap_To_Peak_Timeout)
				{
					for(i=0;i<20;i++)
						{
				  			if (slow[i]>SnapToPeak_Bar[i])SnapToPeak_Bar[i]=slow[i];
				  			if (slow[i]<SnapToPeak_Bar[i])SnapToPeak_Bar[i]--;
				 		}	  	
				 	counter=0;
				}


	if (counter2>Rain_Timeout)
				{
					for(i=0;i<20;i++)
						{
				  			if (SnapToPeak_Bar[i]>Rain[i])Rain[i]=SnapToPeak_Bar[i];
				  			if (SnapToPeak_Bar[i]<Rain[i])Rain[i]--;
				 		}	  	
				 	counter2=0;
				}

}


