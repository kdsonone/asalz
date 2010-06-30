# Exported from MPLAB IDE project `AudioSpectrum.mcp' on Sat Jun 30 13:08:21 2007.

AudioSpectrum.hex : AudioSpectrum.cof
	"C:\Program Files\Microchip\MPLAB C30\bin\pic30-bin2hex.exe" "AudioSpectrum.cof"

AudioSpectrum.cof : "ADC.o" "Audio Spectrum.o"
	"C:\Program Files\Microchip\MPLAB C30\bin\pic30-gcc.exe" -Wl,"D:\DSP Project\Code\ADC.o","D:\DSP Project\Code\Audio Spectrum.o",-L"C:\Program Files\Microchip\MPLAB C30\lib",--script="D:\DSP Project\Code\linkerscript.gld",-o"AudioSpectrum.cof",-Map="AudioSpectrum.map",--report-mem

ADC.o : "ADC.c" "C:\Program Files\Microchip\MPLAB C30\include\math.h" "C:\Program Files\Microchip\MPLAB C30\include\yvals.h" "C:\Program Files\Microchip\MPLAB C30\include\stdlib.h" "C:\Program Files\Microchip\MPLAB C30\support\h\dsp.h" "D:\DSP Project\Code\common.h" "C:\Program Files\Microchip\MPLAB C30\support\h\p30f6012A.h" "C:\Program Files\Microchip\MPLAB C30\support\h\p30fxxxx.h" "D:\DSP Project\Code\ADC.c" "D:\DSP Project\Code\ADC.c"
	"C:\Program Files\Microchip\MPLAB C30\bin\pic30-gcc.exe" -mcpu=30F6012A -c -x c "ADC.c" -o"ADC.o" -g -Wall -O3

Audio Spectrum.o : "Audio Spectrum.c" "C:\Program Files\Microchip\MPLAB C30\include\math.h" "C:\Program Files\Microchip\MPLAB C30\include\yvals.h" "C:\Program Files\Microchip\MPLAB C30\include\stdlib.h" "C:\Program Files\Microchip\MPLAB C30\support\h\dsp.h" "D:\DSP Project\Code\common.h" "C:\Program Files\Microchip\MPLAB C30\support\h\p30f6012A.h" "C:\Program Files\Microchip\MPLAB C30\support\h\p30fxxxx.h" "D:\DSP Project\Code\Audio Spectrum.c" "D:\DSP Project\Code\Audio Spectrum.c"
	"C:\Program Files\Microchip\MPLAB C30\bin\pic30-gcc.exe" -mcpu=30F6012A -c -x c "Audio Spectrum.c" -o"Audio Spectrum.o" -g -Wall -O3

clean : 
	del "ADC.o" "Audio Spectrum.o" "AudioSpectrum.cof" "AudioSpectrum.hex"

