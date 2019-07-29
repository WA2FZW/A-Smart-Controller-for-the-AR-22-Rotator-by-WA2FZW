/*
 * 	My_Rotator.h - Version 05.2
 *
 *	This source file is under General Public License version 3. Please feel free to
 *	distribute it, hack it, or do anything else you like to it. I would ask, however
 *	that is you make any cool improvements, you let me know via any of the websites
 *	on which I have published this.
 *	
 *	This header file is used by the AR22 Controller Version 4.0 software. There are a
 *	few variables that might need to be tweaked based on the characteristics of any
 *	specific rotator and/or hardware used to build the smart controller. Those variables
 *	and explanations of why and how to change them are included here.
 *
 */

#ifndef My_Rotator_h				// Prevent double #include
#define My_Rotator_h

/*
 *	Libraries needed:
 */

#include <Rotary.h>					// From https://github.com/brianlow/Rotary
#include <LiquidCrystal_I2C.h>		// From https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
#include <TimerOne.h>				// From https://github.com/PaulStoffregen/TimerOne
#include <SoftReset.h>				// From https://github.com/WickedDevice/SoftReset
#include <Wire.h> 					// Built-in
#include <String.h>					// Built-in
#include <EEPROM.h>					// Built-in


/*
 *	While in the process of doing some detailed analysis of why my rotator wasn't
 *	turning a full 360 degrees using the original clunk-clunk box, it turned out
 *	that the cam switch in my rotator was not putting out pulses every 6 degrees
 *	as advertised, but rather every 5.85 degrees. This causes the rotator to come
 *	up a little over 9 degrees short after 60 clicks of the clunk-clunk box.
 *
 *	This meant completely revising the azimuth calculations in the main AR-22
 *	program. The following definitions control the number of degrees we add to or
 *	subtract from the Current_Azimuth for each pulse (cam or timer) we receive.
 *	The numbers are all multiplied by 1000 to avoid having to convert the math to
 *	floating point. But we do have to use "long" integers for the math because the
 *	numbers will be much larger than 32K.
 *
 *	Whenever we read in a normal azimuth (from N1MM+ or the serial monitor), it will
 *	be multiplied by "timerDegrees". Conversely, whenever we need to display an
 *	azimuth, the internal number will be divided by "timerDegrees".
 *
 *	"timerDegrees" along with "maxAzimuth" and "minAzimuth" are calculated in "setup()"
 *	based on the value of "PULSE_DEGREES".
 */

#define	PULSE_DEGREES	5850L	// 5.85 (x 1000) degrees per cam pulse
#define	AZ_ROUNDING		 500L	// Used in the azimuth display functions

long	timerDegrees;			// Degrees of rotation for each pulse
long	maxAzimuth;				// Maximum allowed azimuth
long	minAzimuth;				// Minimum allowed azimuth


/*
 *	"timerInterval" determines the interval between interrupts from the TimerOne library.
 *	
 *	The rotator I used for testing and development took just over 45 seconds to complete
 *	a 360 degree rotation. Pulses from the cam operated switch were seen an average of
 *	every 813Ms. The value of "timerInterval" should be set to about 2 - 3% longer than
 *	1/6 of the interval between cam pulses. This ensures that no more and no less than
 *	5 timer pulses will be seen between cam switch pulses.
 *	
 *	The value that seems to work for both of my rotators was determined to be 137Ms.
 */

volatile unsigned long	timerInterval = 137000;	// Generate interrupts every 137mS


/*
 *	"CAL_ADJUSTMENT" is the number of degrees to back off the end stop whenever the rotator
 *	hits the end stop. I have it set to 5 degrees for my rotator, but the stop tit on
 *	mine is about twice the width of the stock one. Interestingly enough, the way the
 *	rotator is designed, there are actually more than 360 degrees stop to stop, thus
 *	it's never actually calibrated accurately.
 */

#define	CAL_ADJUSTMENT 5


/*
 *	These 2 variables are related to the cam switch pulses.
 *	
 *	The cam switch has a lot of contact bounce. On the rotator used for development and
 *	testing, the cam switch sent a series of pulses between 5 and 10Ms long over a period
 *	of about 50Ms. The setting of "PULSE_DEBOUNCE" masks all but the first state transition
 *	in the series of pulses, so that the software only sees one pulse for each operation
 *	of the cam switch.
 *
 *	Although the Version 05.2 hardware design almost completely eliminated the need to
 *	debounce the cam pulses, but I left in in the code in case it's really needed. On
 *	one of my rotators, I lowered it to 5mS based on the spreadsheet computations that
 *	are described in the documentation.
 *	
 *	Again, my rotator sent cam switch pulses approximately every 813Ms, so the setting of
 *	the "rotatorTimeout" is 850 milliseconds.
 */

#define	PULSE_DEBOUNCE	 15L						// Debounce limit = 15 mS
#define	ROTATOR_TIMEOUT	850L						// Timeout between cam pulses


/*
 * 	Define the characteristics for the I2C LCD Display. The pin definitions are standard
 * 	for the Arcuino Mega 2560, and should not need to be changed.
 * 	
 * 	The next two variables define the height and width of the display. If a 4x20 display
 * 	were to be used, those should be changed appropriately.
 * 	
 * 	The final variable is the I2C address of the display. 0x27 is the standard address,
 * 	however some of the ones I purchased on the internet came with an address of 0x3F.
 * 	There is a program available on GitHub (https://gist.github.com/tfeldmann/5411375)
 * 	that will scan for I2C devices and report the addresses of any found, if you're
 * 	not sure of the display's address.
 */

#define LCD_SDA		  20			// Data line
#define LCD_SCL 	  21			// Clock line
#define LCD_HEIGHT 	   2			// 2 Lines high
#define LCD_WIDTH 	  16			// 16 characters wide


/*
 *	The standard I2C address for the LCD is 0x27. One of my units has one
 *	whose address is 0x3F. There are programs on the internet that will
 *	scan for I2C devices and report the addresses of any found if you 
 *	suspect you're having a problem here.
 */

#define	LCD_ADDRESS 0x27			// Standard address


/*
 *	Define the variables associated with the rotary encoder.
 *
 *	Note that the "HALF_STEP" definition is currently disabled in the "Rotary.h" file, thus
 *	each click of the encoder is actually 1 encoder step. If "HALF_STEP" is defined, each
 *	click is actually 2 steps on the encoder.
 *
 *	On some encoders available on the internet, the "A" and "B" pins are reversed (as
 *	is the case with the ones I have). If your encoder seems to work backwards, swap
 *	the pin definitions.
 */

#define	ENCODER_PIN_B	2				// and Pin_B as CLK
#define	ENCODER_PIN_A	3				// My encoders work backwards, so we treat Pin_A as DT
#define CAL_SWITCH		4				// Switch is on digital pin 9 of the Arduino


/*
 *	If "AZ_SW_INSTALLED" is defined, the 3 position toggle switch that allows the operator 
 *	to select the number of degrees to change the target azimuth for each encoder click is
 *	installed. It the switch has not been installed, the definition should be commented out.
 *
 *	If the switch is NOT installed, you can change the value of "DEFAULT_INCR" to set the
 *	number of degrees by which to change the target azimuth for each encoder click.
 */

	#define AZ_SW_INSTALLED
	#define	DEFAULT_INCR	5L				// Feel free to change this!


/*
 * "EPROM_TIMEOUT" defines the number of milliseconds to wait afterthe rotator stops moving to
 * update the saved azimuth in the EEPROM. It is currently set to 10 seconds.
 */

	#define	EEPROM_TIMEOUT	10000			// 10 seconds in milliseconds
	
#endif
