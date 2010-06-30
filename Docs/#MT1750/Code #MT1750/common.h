
// ADC
// FOSC = 7372800 - From a 14745600 Ctal @ HS2 Osc.- mode
// PLL	x16
// FCY= FOSC*PLL/4
// SAMPLINGRATE1 = 7998.6981 Hz 
// SAMPLINGRATE2 =  87252.071  Hz 
// SAMPCOUNT1 = (FCY/SAMPLINGRATE1)+1
// SAMPCOUNT2 = (FCY/SAMPLINGRATE2)+1

#define SAMPCOUNT1  3687                 
#define SAMPCOUNT2  339     


#define BarDelay    200					// Sets the display's orizontal sweeping frequency for the 0,1 and 2 display  modes  
#define BarDelay2   400					// Sets the display's orizontal sweeping frequency for the 3rd display  mode 
#define PeakHold_timeout 500			// The Peaks are being hold for PeakHold_timeout display frames in the 3rd display mode
#define Snap_To_Peak_Timeout 80			// The Bars climb up to the peak after Snap_To_Peak_Timeout display frames in the 0, 2 display modes 
#define Rain_Timeout		500			// The Rain falls at a rate of 1.3db/Rain_Timeout display frames in the 2 display mode

//FFT
#define NUMSAMP 256                     /* = Number of samples & frequency points in the FFT */
#define LOG2_BLOCK_LENGTH 	8   	    /* = Number of "Butterfly" Stages in FFT processing */

//Display
#define	Threshold1   0x0001     // -27.0 db threshold 		
#define	Threshold2	 0x0002	    // -24.0 db threshold	
#define	Threshold3	 0x0003		// -22.1 db threshold
#define	Threshold4   0x0004		// -20.8 db threshold 
#define	Threshold5	 0x0005		// -19.5 db threshold
#define	Threshold6	 0x0007		// -18.2 db threshold
#define	Threshold7	 0x000A		// -16.9 db threshold	
#define	Threshold8	 0x000E		// -15.6 db threshold
#define	Threshold9   0x0012		// -14.3 db threshold
#define	Threshold10	 0x0019		// -13.0 db threshold	
#define	Threshold11  0x0021 	// -11.7 db threshold
#define	Threshold12  0x002D 	// -10.4 db threshold
#define	Threshold13	 0x003C		// -9.1 db threshold
#define	Threshold14	 0x0051		// -7.8 db threshold
#define	Threshold15	 0x006E		// -6.5 db threshold
#define	Threshold16	 0x0094		// -5.2 db threshold
#define	Threshold17	 0x00C8		// -3.9 db threshold
#define	Threshold18	 0x010E		// -2.6 db threshold	
#define	Threshold19	 0x016B		// -1.3 db threshold
#define	Threshold20	 0x01EA		//  0.0 db threshold

//IO - mapping
#define	X1  PORTDbits.RD8 
#define	X2  PORTGbits.RG2
#define	X3  PORTGbits.RG3
#define	X4  PORTFbits.RF6 
#define	X5  PORTFbits.RF2
#define	X6  PORTFbits.RF5
#define	X7  PORTFbits.RF3 
#define	X8  PORTFbits.RF4
#define	X9  PORTBbits.RB15
#define	X10 PORTBbits.RB14
#define	X11 PORTBbits.RB13 
#define	X12 PORTBbits.RB12
#define	X13 PORTBbits.RB11
#define	X14 PORTBbits.RB10 
#define	X15 PORTBbits.RB5
#define	X16 PORTBbits.RB4
#define	X17 PORTBbits.RB3 
#define	X18 PORTBbits.RB2
#define	X19 PORTBbits.RB7
#define	X20 PORTBbits.RB6

#define	Y1  PORTGbits.RG6 
#define	Y2  PORTGbits.RG15
#define	Y3  PORTGbits.RG13
#define	Y4  PORTGbits.RG12 
#define	Y5  PORTGbits.RG14
#define	Y6  PORTGbits.RG0
#define	Y7  PORTGbits.RG1 
#define	Y8  PORTFbits.RF1
#define	Y9  PORTFbits.RF0
#define	Y10 PORTDbits.RD7
#define	Y11 PORTDbits.RD6 
#define	Y12 PORTDbits.RD5
#define	Y13 PORTDbits.RD4
#define	Y14 PORTDbits.RD3 
#define	Y15 PORTDbits.RD2
#define	Y16 PORTDbits.RD1
#define	Y17 PORTDbits.RD0 
#define	Y18 PORTDbits.RD11
#define	Y19 PORTDbits.RD10
#define	Y20 PORTDbits.RD9

#define	ControlBit1 PORTCbits.RC13
#define	ControlBit2 PORTCbits.RC14

#define	on  1
#define	off 0



