/*
 * 	AR22_Controller - Version 5.4
 *
 *	This source file is under General Public License version 3. Please feel free to
 *	distribute it, hack it, or do anything else you like to it. I would ask, however
 *	that is you make any cool improvements, you let me know via any of the websites
 *	on which I have published this.
 *
 *	This program is designed to run on an Arduino Mega 2560. It emulates the interface
 *	between the N1MM Rotor control program and a Yaesu rotator using the GS-232
 *	protocol). Since the N1MM Rotor program does not recognize the AR-22 rotator,
 *	we have to make it think it is talking to something that it does support. A 2nd
 *	choice would have been to emulate the HyGain DCU-1 protocol, however, it does
 *	not have as complete a command set as the Yaesu protocol, and is harder to decipher.
 *	
 *	In Version 1.0, the rotator operated in the same manner as it did with the original
 *	clunk-clunk box; in other words, the azimuth resolution was based solely on the ~6
 *	degree cam operated switch in the rotator.
 *	
 *	In Version 2.0, we added a timer interrupt set to trigger approximately every 140mS,
 *	which is just slightly longer than the rotator takes to turn 1 degree. We treat
 *	the timer interrupt as if it were a regular index pulse (except for reseting the
 *	timeout). When a hard pulse is received, we will reset the timer and timeouts.
 *	
 *	For Version 2.1, we made a hardware change. Rather than the back panel reset button
 *	being connected to the Arduino's reset pin, we moved it to a digital pin (D18) and
 *	use a software reset along with a forced save of the "currentAzimuth" before restarting.
 *	Note that controllers wired for the hard reset can still use this software.
 *	
 *	In Version 3.0, I moved the definitions of variables that might need to be tweaked
 *	based on characteristics of a particular rotator to the "My_Rotator.h" header file.
 *	
 *	More significantly, in trying to determine why the rotator was not turning a full 360
 *	degrees with the clunk-clunk controller, it was discovered that the cam switch was
 *	actually sending pulses about every 5.85 degrees as opposed to the advertised 6 degrees.
 *	In Version 3.0, the azimuth math has been totally overhauled to deal with this issue.
 *	
 *	Part of the overhaul of the azimuth math involves maintaining all azimuth related values
 *	in the program multiplied by 1000 (defined in "My_Rotator.h" as "timerDegrees").
 *
 *	In Version 4.0, a number of hardware and software changes are made. In the hardware, we
 *	added a 3 position toggle switch which is used to select the size of the azimuth increment
 *	(or decrement) for each click of the encoder. After finally getting to use the controller
 *	during a contest where many azimuth changes were made only being to change the azimuth one
 *	degree at a time was a pain! The switch allows you to select 1, 5 or 10 degree increments.
 *
 *	Another hardware change is to add a LED that indicates the current azimuth has been saved
 *	in the EEPROM. This LED will also flash for about a second whenever a timeout occurrs.
 *
 *	Software changes in Version 4.0 include support for the above hardware changes and a
 *	minor change in the azimuth save logic. I discovered that when making large azimuth
 *	changes, sometimes you stop turning the knob briefly and the azimuth gets saved before the
 *	entire change has completed. We will now wait a few seconds after the rotator has stopped
 *	turning before saving the azimuth. The delay is currently set to 10 seconds in the
 *	"My_Rotator.h" file. It can be changed to whatever you like. We also eliminated the 1
 *	minute minimum between EEPROM saves.
 *
 *	I also changed the naming conventions of all defined things and variables to match the
 *	standard coding conventions.
 *
 *	In Version 4.1, I made a change in the calibration procedure. Previously, the "calibrate()"
 *	function used a comaparison between the current aximuth the maximum and minimum azimuths to
 *	take a guess as to which way to turn to accomplish the procedure. If the rotator got out of
 *	calibration due to a missed hard pulse, the current azimuth would be at either the maximum
 *	or minimum and would sometimes turn 3/4 or more of the way around to calibrate.
 *
 *	The fix is to add a new variable: "lastGoodAzimuth" which will be updated everytime a pulse
 *	is seen during normal rotation. This variable will also be set at the end of a calibration
 *	cycle, but not during calibration.
 *
 *	Version 5.0 made some changes in pin assignments for compatibility with the PCB that
 *	replaced the hand-wired controller board.
 *
 *	Version 5.1 just fixes a couple of bugs.
 *
 *	In Version 5.4, I changed a number of things (including the hardware) in an attempt
 *	to fix the problem of the USB connection between the controller dropping and a few
 *	other things:
 *	
 *		First, I changed the timeout logic which fixed a couple of problems when the
 *		motor is first started:
 *
 *			1.	With a large antenna, there is some startup inertia which needed to be
 *				compensated for.
 *			2.	If the rotator happened to have stopped with the cam switch in the
 *				closed position we were seeing an immediate pulse and an immediate
 *				timeout occurred.
 *
 *		Secondly, I added code to turn off the timer interrupts when a timeout occurs
 *		or when the motor is stopped.
 *
 *		The other major change was to eliminate the command translation table and to
 *		combine the original "Get_Command" and "Process_Command" functions into a
 *		single function.
 *
 *		Finally, I added all the hooks for the interface to an elevation controller
 *		for my satellite antenna.
 */

/*
 *	Setting "DEBUG" to "true" will turn on a lot of print statements that show you
 *	what the program is thinking. It should only be enabled if you are using the
 *	Arduino IDE's serial monitor (or some other terminal emulator) to talk to it as
 *	the debug outputs are likely to confuse other programs such as N1MM Rotor or
 *	SatPC32.
 */

#define	DEBUG	false				// Enables verbose output when set to "true"

#include "My_Rotator.h"				// Things that might have to be tweaked.


/*
 *	Summary of Arduino input/output pin usage:
 *
 *		D0		USB Interface Receive
 *		D1		USB Interface Transmit
 *		D2		ENCODER_PIN_B
 *		D3		ENCODER_PIN_A 
 *		D4		CAL_SWITCH (encoder pushbutton)
 *		D5	 	1 degree switch
 *		D6		10 degree switch
 *		D8		Azimuth saved LED (on when the "currentAzimuth" is saved in the EEPROM)
 *		D9		MOTOR_DIRECTION
 *		D10		MOTOR_RUN
 *		D11		-|
 *		D12		 |- Reserved for Timer1 PWM output signals
 *		D13		-|
 *		D16		Serial2 Transmit line (reserved for future use)
 *		D17		Serial2 Receive line (reserved for future use)
 *		D18		MOTOR_INDEX (Interrupt enabled pin )
 *		D19		Software Reset
 *		D20		LCD_SDA (Data)				
 *		D21		LCD_SCL (Clock)
 */


/*
 *	Define the pin assignments and other stuff for the rotator
 */

#define	MOTOR_DIRECTION	 9				// Relay (K2) to select direction of rotation
#define	MOTOR_RUN	 	10				// Relay (K1) to power the rotator
#define MOTOR_INDEX	    18				// 6 degree indicator (interrupt enabled)

#define	ONE_DEGREE	     5				// When LOW, each encoder click is 1 degree
#define TEN_DEGREE       6				// When LOW. each encoder click is 10 degrees
										// If neither is LOW, each click is "DEFAULT_INCR" degrees

#define	SAVED_LED		 8				// Operates LED to show azimuth is saved
#define	LED_ON	HIGH					// By defining the on and off conditions this
#define	LED_OFF	LOW						// way I don't have to remember what they are!

#define	STOP	LOW						// Stop the motor
#define	RUN		HIGH					// Run the motor

#define	CW		LOW						// Rotate clockwise
#define	CCW		HIGH					// Rotate counter-clockwise
#define	OFF		LOW						// Alternate definition to turn relays off

bool			directionRelay  = OFF;		// Direction relay is off
volatile bool	isRotating 		= false;	// True when rotation in progress


/*
 *	Define the pulse types. See "My_Rotator.h" for the definition of the timer pulse
 *	interval and degrees of rotation parameters.
 */
 
volatile uint8_t	pulseType;				// Real, fake or none

#define				NO_PULSE	0			// No pulse seen
#define				HARD_PULSE	1			// Pulse from cam switch
#define				SOFT_PULSE	2			// Timer generated pulse


/*
 *	There are a number of timeout conditions we need to deal with.
 *
 *	1).	When the rotator is turning, we expect to see periodic index pulses from the cam
 *		operated switch. The actual time for the rotator used for testing is about every
 *		784 mS). If that doesn't happen within the alloted time ("ROTATOR_TIMEOUT"), we
 *		assume that it was out of calibration, and it hit the mechanical end stop.
 *
 *	2).	Hello message delay time. When we have nothing of importance to tell the operator about,
 *		we put the default message on the top line of the display. However, if there is something
 *		of importance there, or while the rotator is turning, we don't want to overwrite it until
 *		the operator has had a few seconds to read it.
 *
 *	3).	In version 4.0, we dropped the 1 minute minimum time between EEPROM updates and instead
 *		wait 10 seconds after "rotatorStopTime" before doing the write. This keeps the EEPROM
 *		from being updated until we're pretty sure the operator has finished tweaking the
 *		azimuth.
 *
 *	4). As noted above, in Version 5.4 I added a longer timeout (2 seconds) when the motor first
 *		starts.
 *
 *	The following definitions set the parameters for all the timeouts:
 */

volatile uint32_t	systemTime;					// Since millis() doesn't work in ISRs

volatile uint32_t	pulseTime    = 0;			// Time of last index pulse for timeout function
volatile uint32_t	debounceTime = 0;			// Time of last index pulse for pulse processing logic
volatile uint32_t	rotatorTimeout;				// Cam pulse timeout is variable now
volatile uint16_t	hardPulseCount = 0;			// Counts cam switch pulses

uint32_t			notHelloTime    = 0;		// Last time something other than hello displayed
uint32_t			helloTime       = 0;		// Current time
uint32_t			helloTimeout    = 3000;		// 3 second refresh delay
bool				helloMsg        = false;	// True when the default message is on line 1 of the LCD

uint32_t			rotatorStopTime = 0;		// Last time rotator stopped turning
bool				azChanged       = false;	// True when currentAzimuth has changed
volatile bool		tgtChanged      = false;	// True when targetAzimuth has changed


/*
 *	These are computed based on the value assigned to the symbol "PULSE_DEGREES"
 *	in the header file:
 */

int32_t	timerDegrees;			// Degrees of rotation for each pulse
int32_t	maxAzimuth;				// Maximum allowed azimuth
int32_t	minAzimuth;				// Minimum allowed azimuth


/*
 *	The target and current azimuths keep track of where the rotator is and where we 
 *	want it to be. Anytime they are not the same, the rotator will start turning. Note
 *	the numbers are all stored internally as "long" integers as the numbers will be
 *	much larger than 32K. The math is all done as integer math; Arduinos suck at doing
 *	floating point accurately!
 */

volatile int32_t	targetAzimuth;				// Target azimuth
int32_t				currentAzimuth;				// Current azimuth
int32_t				lastGoodAzimuth;			// Last known good azimuth
int32_t				deltaAzimuth;				// Used in rotating logic
int32_t				azMultiplier;				// Value set by toggle switch
	
#define				AZIMUTH_ADDRESS	0			// EPROM Address for stored "currentAzimuth"

volatile bool		isCalibrating = false;		// True when calibration is in progress


/*
 *	These buffers are used to queue stuff to be displayed on the LCD. Attempting to
 *	update the display from within the "Read_Encoder()" interrupt handling function caused
 *	the whole system to lockup because of how the Arduino handles interrupts. Use of the
 *	"String" class makes handling the strings easier than if we had used "char" arrays.
 */

String	lcdBuffer_1;						// Line 1 buffer
String	lcdBuffer_2;						// Line 2 buffer


/*
 *	Define most of the various strings to be displayed on the LCD (some are built
 *	piecemeal in the code), but these are the fixed ones
 */

#define	helloString			"AR-22 Controller"			// Normal first line of display
#define versionString		"  Version 5.4   "			// Curent software version
#define	currentAzString		"Current Az:  "				// Normal 2nd line
#define	targetAzString		"Target  Az:  "				// Line 1 when rotator is moving
#define timeoutString		"*** Timeout  ***"			// Line 1 when rotator timed out
#define	azSaveString		"* Azimuth Save *"			// Line 1 when saving currentAzimuth
#define	calibrateString		"* Calibrating * "			// Line 1 when calibration in progress
#define	calibratedString	"*  Calibrated  *"			// Line 1 when calibration is finished


/*
 *	Define a few other miscellaneous variables
 */

String	commandBuffer;						// For reading commands from USB port
String	workBuffer;							// General use buffer


/*
 *	If the elevation processor is attached, we need a buffer for messages coming from
 *	it:
 */

#if ( ELEVATION )

String	elevRxBuffer;						// For responses from the elevation processor

#endif

#define SOFT_RESET_PIN 19					// LOW is for software reset (interrupt enabled pin)


/*
 *	Create the rotary encoder object and the display object. Note, the address of some of
 *	the displays I bought on the internet were set to 0x3F, as opposed to the standard 0x27.
 *	If necessary, I think that can be changed with jumpers on the board.
 *	
 *	The address and display size definitions are in "My_Rotator.h". Note that I have two
 *	different controllers which have different LCD addresses; both are in the header file.
 *
 *	If your encoder seems to work backwards, flip the numerical definitions of the pin
 *	numbers in the "My_Rotator.h" file.
 */

	Rotary Encoder = Rotary ( ENCODER_PIN_A, ENCODER_PIN_B );
	LiquidCrystal_I2C lcdDisplay ( LCD_ADDRESS, LCD_WIDTH, LCD_HEIGHT );


/*
 *	The "setup()" function is called by the Adruino boot loader when the controller
 *	is first powered up, or after a reset. Here, we initialize anything that needs
 *	to be set up before we really go to work.
 */

void setup ()
{
	Serial.begin ( BAUD_RATE );				// See comments in "My_Rotator.h"
	delay ( 1000 );							// Blindly wait 1 second for connection

	if ( ELEVATION )						// Elevation processor attached?
		Serial2.begin ( BAUD_RATE );		// Yes, enable second serial port


/*
 *	PULSE_DEGREES is defined in "My_Rotator.h", and is the number of degrees per cam
 *	switch index pulse times 1000. Based on that value, we need to compute the values
 *	for the number of degrees to count for each timer or cam pulse along with the
 *	minimum and maximum azimuths.
 */

	timerDegrees = PULSE_DEGREES / 6;			// Degrees to count for each pulse (x1000)
	maxAzimuth   = timerDegrees * 360;			// Maximum azimuth
	minAzimuth   = 0;							// Zero is zero!

 	#if	DEBUG
 		Serial.print ( "Timer_Deg = " );
 		Serial.print ( timerDegrees );
 		Serial.print ( ",\tMax_Az = " );
 		Serial.println ( maxAzimuth );
 	#endif

	EEPROM.get ( AZIMUTH_ADDRESS, currentAzimuth );		// Last known position of the rotator
	lastGoodAzimuth = currentAzimuth;					// We assume it's good


/*
 *	Initialize all the flags and timeout timers associated with rotation
 */

	targetAzimuth	= currentAzimuth;			// Make them the same
	isRotating		= false;					// Rotator isn't turning
	isCalibrating	= false;					// Not being calibrated
	azChanged		= false;					// currentAzimuth has not changed
	tgtChanged		= false;					// Neither has targetAzimuth
	debounceTime	= 0;						// Only non-zero when first pulse received
	pulseTime		= 0;						// No pulses yet
	rotatorStopTime = millis ();				// Last time rotator stopped turning
	pulseType		= NO_PULSE;					// No index pulse yet


/*
 *	Setup the I/O pins associated with the rotator proper
 */

	pinMode ( MOTOR_DIRECTION, OUTPUT ); 			// Direction relay
	pinMode ( MOTOR_RUN,       OUTPUT ); 			// Motor run relay
	pinMode ( MOTOR_INDEX,     INPUT_PULLUP ); 		// 6 degree indicator


/*
 *	And the pins for the azimuth increment toggle switch and azimuth saved LED
 */

	pinMode	( SAVED_LED,  OUTPUT );					// Operates LED to show azimuth is saved
	pinMode	( ONE_DEGREE, INPUT_PULLUP);			// When LOW, each encoder click is 1 degree
	pinMode ( TEN_DEGREE, INPUT_PULLUP);			// When LOW, each encoder click is 10 degrees

	digitalWrite ( SAVED_LED, LED_ON );				// Turn the azimuth saved LED on


/*
 *	Initialize the display.
 */

	lcdDisplay.begin ();							// Initialize the LCD
	lcdDisplay.backlight ();						// Turn on the backlight

	LCD_Print ( 1, String ( helloString ));			// We're almost on the air!
	LCD_Print ( 2, String ( versionString ));

	delay ( 2000 );


/*
 *	Set up the encoder pins:
 */

	pinMode ( ENCODER_PIN_A,  INPUT_PULLUP ); 		// Encoder has pullups on the board but
	pinMode ( ENCODER_PIN_B,  INPUT_PULLUP ); 		// we're going to also use the internal ones
	pinMode ( CAL_SWITCH, 	  INPUT_PULLUP );		// Definitely needed here


/*
 *	Set up the interrupts for the rotary encoder. Some documents online would lead one to
 *	believe that the encoder can be handled without interrupts, but I couldn't make it work!
 *	The pin definitions are in "My_Rotator.h". They might need to be flip-flopped there if
 *	the encoder seems to work backwards.
 */

	attachInterrupt ( digitalPinToInterrupt ( ENCODER_PIN_A ), Read_Encoder, CHANGE );
	attachInterrupt ( digitalPinToInterrupt ( ENCODER_PIN_B ), Read_Encoder, CHANGE );


/*
 *	Set up the interrupt handler for the cam switch index pulse processing. Since adding the
 *	LCA-110 SSR to the pulse circuitry, we trigger on the end of the pulse (hence RISING) which
 *	minimizes the amount of time needed for debouncing. The pulses from the SSR are very well
 *	defined.
 */

	attachInterrupt ( digitalPinToInterrupt ( MOTOR_INDEX ), Pulse_ISR, RISING );


/*
 *	Set up the timer interrupt handler.
 */

	Timer1.initialize ( timerInterval );			// Initialize the timer
	Timer1.attachInterrupt ( Timer_ISR );			// Define interrupt handler


/*
 *	Set up the software reset pin. I tried to make it work as an interrupt, but moving
 *	the encoder was causing the processor to reboot on occasion.
 */

	pinMode ( SOFT_RESET_PIN, INPUT_PULLUP ); 		// Use pullup resistor

	azMultiplier = DEFAULT_INCR;					// Assume default azimuth increment
}


/*
 *	Once the "setup()" function has completed, the "loop()" function runs forever or at
 *	least until the power is turned off or the hardware is reset.
 *
 *	The functions called from the main loop all execute rather quickly and use a number of
 *	flags amongst themselves to determine the state of the program at any time. The order in
 *	which they are called is semi-critical.
 *	
 *	A word of explanation about "systemTime". The "millis()" function doesn't work correctly in
 *	interrupt service routines where we need to record the times at which the interrupts
 *	occurred, so we set it every time through the loop and use that value instead of calls to
 *	"millis()" in the interrupt functions.
 */

void loop ()
{
	systemTime = millis ();						// millis() isn't reliable in ISR functions
	
	Say_Hello ();								// See if we need to redisplay the normal line 1

	Get_Command ();								// Get a command and process it (if any there)


/*
 *	Eventually, the next few statements should be moved into a separate function, but
 *	as this is all still in the experimental stage, we'll leave it inline for now.
 *
 *	What we're doing (if the elevation processor is attached) is to see if it has anything
 *	to say that needs to be sent back to the controlling program on the PC.
 */

	if ( ELEVATION )
	{
		if ( Serial2.available () )				// Anything to read?
		{
			elevRxBuffer = Serial2.readStringUntil ( '\r' );	// Read the entire response
			Serial.print ( elevRxBuffer );						// Send it to the PC
			Serial.print ( '\r' );								// Along with a carriage return
		}
	}


/*
 *	As we don't allow the operator to change the position of the target azimuth increment
 *	selector switch while the encoder is being operated or while the rotator is turning, we
 *	will set the "azMultiplier" here. If the switch is not installed or is in the center-
 *	off position, the increment is determined by the value of "DEFAULT_INCR" (defined in
 *	the "My_Rotator.h" file).
 *
 *	Disabling the interrupts fixed an issue where an encoder interrupt occurred in the
 *	middle of reading the switch which sometimes caused a click of the encoder to change
 *	the target asimuth by the wrong amount.
 *
 *	This code only compiles if the switch is installed.
 */

#if AZ_SW_INSTALLED										// If the switch is installed

	if ( !isRotating )									// Only if not turning
	{
		noInterrupts ();								// Disable interrupts

		azMultiplier = DEFAULT_INCR;					// Assume default azimuth increment

		if ( digitalRead ( ONE_DEGREE ) == LOW )		// One degree per click selected?
			azMultiplier = 1;							// Yep!

		else if ( digitalRead ( TEN_DEGREE ) == LOW )	// Ten degrees per click selected?
			azMultiplier = 10;							// Yes it is!

		interrupts ();							// Interrupts ok again
	}

#endif											// #if AZ_SW_INSTALLED

	Display_Target ();							// Only if changed
	Start_Rotator ();							// If necessary, turn the rotator

	if ( isRotating )							// These only need checked if turning
	{
		rotatorStopTime = systemTime;			// Rotator is still turning
		Check_Timeout ();						// Check for rotator timeout
		Read_Pulse ();							// Check for an index pulse
		Check_Reversal ();						// See if we need to reverse direction
	}

	else										// If not rotating
	{
		Update_Az ();							// Update display and maybe the EEPROM
		Check_LCD ();							// Anything to display?
	}

	if ( digitalRead ( CAL_SWITCH ) == LOW )	// Operator want to do calibration?
		Calibrate ();							// Yes, then do it

	Ck_Reboot ();								// Operator want to reboot?
}


/*
 *	"Get_Command" was totally overhauled in Version 5.4. In earlier versions of the 
 *	software it simply read a command from the serial port and translated the character
 *	to a number from the "commandTable".
 *
 *	In Version 5.4, the translation process and the "Process_Command" function were
 *	eliminated. "Get_Command" now also processes the command.
 *
 *	Also in version 5.4, I added handling for the 'W' command. This command requests
 *	both an azimuth and elevation change. The format is "waaa ee". We process it
 *	just like the 'M' command, but then forward it to the elevation controller if that
 *	option is enabled. As near as I can tell this is the only command used by the
 *	SatPC32 program that the elevation controller is interested in.
 *	
 *	Note that the N1MM Rotor program uses a return ('\r') character as a command line
 *	terminator as opposed to the newline character ('\n') generally used by "C" language
 *	programs, thus we use the "Serial.readStringUntil()" function to read the command line.
 *
 *	Also note that if using the Arduino IDE serial monitor (or other terminal emulator)
 *	to send commands, the terminator (button at the bottom of the screen) needs to be set
 *	for "Carriage Return".
 *	
 *	When using the serial monitor, the command letters may be entered in either upper
 *	or lower case.
 */

bool Get_Command ()
{
char command;									// The command character

	if ( !Serial.available () )					// Anything to read?
		return false;							// Nope!

	command = toupper ( Serial.peek () );		// Get the command letter & translate to upper case

	commandBuffer = Serial.readStringUntil ( '\r' );	// Read the entire command for later use

	if ( DEBUG )
	{
		Serial.print ( "Command = " );		Serial.print ( command );
		Serial.print ( ",  Buffer = " );	Serial.println ( commandBuffer );
	}


/*
 *	The "command" variable now contains the single character command letter and the
 *	"commandBuffer" contains the entire command line.
 */

	switch ( command )
	{
		case	'C':							// Respond to request for current azimuth
		{
			workBuffer = "AZ=";
			workBuffer += Int_2_Az (( currentAzimuth + AZ_ROUNDING ) / timerDegrees );
			workBuffer += '\r';					// Add a carriage return

			Serial.print ( workBuffer );		// Send the answer


/*
 *	If in debugging mode, we also output a newline which makes the output easier
 *	to read on the screen:
 */

			if ( DEBUG )
				Serial.print ( '\n' );

			Serial.flush ();						// Make sure everything gets sent
			break;
		}

		case	'M':								// Turn to specified azimuth
		case	'W':								// Azimuth + elevation
		{
			workBuffer = commandBuffer.substring ( 1, 4 );		// Should be 3 digit azimuth
			targetAzimuth = workBuffer.toInt ();				// Make it a number

			targetAzimuth *= timerDegrees;			// Internal representation

			if ( targetAzimuth < minAzimuth )		// Shouldn't happen
				targetAzimuth = minAzimuth;

			if ( targetAzimuth > maxAzimuth )		// Also shouldn't happen
				targetAzimuth = maxAzimuth;

			tgtChanged = true;						// Signal to update the display

			if ( ELEVATION && ( command == 'W' ))	// Does this need to go to the elevation controller?
			{
				Serial2.print ( commandBuffer );	// Yes forward command to it
				Serial2.print ( '\r' );				// And add a carriage return
			}

			break;									// Done
		}

		case	'S':								// Cancel current command
		case	'A':								// Stop rotation
		{
			targetAzimuth = currentAzimuth;			// whever we are is now the target
			Stop_Motor ( "Cancel" );				// Turn motor off
			break;
		}

		case	'Z':								// Initialize azimuths
		{
			workBuffer = commandBuffer.substring ( 1, 4 );	// Should be 3 digit azimuth
			targetAzimuth = workBuffer.toInt ();			// Make it a number

			targetAzimuth *= timerDegrees;					// Internal representation

			if ( targetAzimuth < minAzimuth )				// Shouldn't happen
				targetAzimuth = minAzimuth;

			if ( targetAzimuth > maxAzimuth )				// Also shouldn't happen
				targetAzimuth = maxAzimuth;

			currentAzimuth = targetAzimuth;					// Make them equal
			
			azChanged = false;								// Unchanged since last save

			EEPROM.put ( AZIMUTH_ADDRESS, currentAzimuth );	// Save current azimuth
			digitalWrite ( SAVED_LED, LED_ON );				// Turn on azimuth saved LED

			lcdBuffer_1 = azSaveString;

			break;								// Done
		}

		case	'L':							// Start counter-clockwise rotation
		{
			targetAzimuth = maxAzimuth;			// Open-ended move toward 360 degrees
			tgtChanged = true;
			break;
		}

		case	'R':							// Start clockwise rotation
		{
			targetAzimuth = minAzimuth;			// Open-ended move toward 0 degrees
			tgtChanged = true;
			break;
		}

		case	'F':							// Perform calibration
		{
			Calibrate ();						// Just do it!
			break;
		}

		default:
			break;
	}											// End of command processing "switch"

	command = "";								// Clear them for the next time
	commandBuffer = "";
}


/*
 *	Start_Rotator checks to see if we need to actually turn the rotator. If the 
 *	"currentAzimuth" and "targetAzimuth" are not equal, then we need to turn it. If
 *	the "targetAzimuth" is less than the "currentAzimuth", we need to turn the rotator
 *	clockwise. If the "targetAzimuth" is greater than the "currentAzimuth", we need to
 *	turn the rotator counter-clockwise.
 *
 *	When we initiate the motion, we also set the time at which the motion started. The
 *	"Check_Timeout" function, which is called from the main loop will check to see if
 *	the "rotatorTimeout" value has been exceeded. If that happens, we assume that
 *	the rotator has hit the mechanical end stop in whichever direction it was turning.
 */

void Start_Rotator ()
{
	if ( isRotating )							// Already turning?
		return;									// Don't restart

	if ( targetAzimuth == currentAzimuth )		// Nothing to do
		return;									// We're done!

	if ( targetAzimuth > currentAzimuth )		// Need CCW rotation
	{
		directionRelay = CCW;					// Set rotation direction
		deltaAzimuth   = timerDegrees;			// Current azimuth will increase
	}

	else
	{
		directionRelay = CW;					// Set rotation direction
		deltaAzimuth   = -timerDegrees;			// Current azimuth will decrease
	}

	Start_Motor ( directionRelay );				// Start the rotator
	Display_Rotation ();						// Give operator status
}


/*
 *	"Check_Timeout" checks to see if the rotator has timed out. That happens if the
 *	rotator "isRotating" and we didn't see a (real) "MOTOR_INDEX" pulse within the time
 *	limit set by "rotatorTimeout". If this happens, we kill the motor and set the current
 *	and target azimuths to either "minAzimuth" or "maxAzimuth" depending on which way the
 *	rotator was turning then inform the operator.
 *	
 *	If a timeout occurred, the function returns "true"; otherwise it returns "false".
 *
 *	New in Version 4.0, we also flash the azimuth saved LED for about a second to get the
 *	operator's attention.
 */

 bool Check_Timeout ()
 {
 	if ( !isRotating )										// Actually turning?
 		return false;										// Nope

#if DEBUG

	if ( pulseType )
	{
		Serial.print ( "ST: " );		Serial.print (systemTime );
		Serial.print ( ",  PT(" );		Serial.print ( pulseType );
		Serial.print ( "): " );			Serial.print ( pulseTime );
		Serial.print ( ",  Diff: " );	Serial.print ( systemTime - pulseTime );
		Serial.print ( ",  RT: " );		Serial.println ( rotatorTimeout );
	}

#endif

	if (( systemTime - pulseTime ) <= rotatorTimeout )		// Out of time?
		return false;										// Nope!

	Stop_Motor ( "Timeout" );								// Turn motor off

	Backup ( CAL_ADJUSTMENT * timerInterval / 1000 );		// Back it off the end stop by
															// "CAL_ADJUSTMENT" degrees


/*
 *	If a timeout occurs, we assume that it's because the rotator hit the mechanical
 *	end stop. That being the case, we figure out which way it was turning and set
 *	the "currentAzimuth" to "minAzimuth" or "maxAzimuth" appropriately. "targetAzimuth 
 *	is also set to that value.
 */

	if ( directionRelay == CW )					// Rotating clockwise?
		currentAzimuth = minAzimuth;			// Yes, we're at 0 degrees
 	
	else
		currentAzimuth = maxAzimuth;			// No, then 360 degrees

 	targetAzimuth = currentAzimuth;				// Set targetAzimuth to Current
	tgtChanged    = false;
	
	EEPROM.put ( AZIMUTH_ADDRESS, currentAzimuth );		// Write new azimuth
	lastGoodAzimuth = currentAzimuth;					// It's now the last good one


/*
 *	To make the timeout more obvious, we're going to blink the azimuth saved LED a few times,
 *	then leave it on.
 */

	Blink_LED ();

	lcdBuffer_1 = timeoutString;					// Inform operator
	return true;
}


/*
 *	"Timer_ISR" is not exactly an interrupt service routine. The interrupts are actually
 *	processed inside the "Timer1" library code. This is a callback function that is called
 *	from within the library when the time expires.
 *
 *	All we do is indicate that we saw a soft index pulse provided the rotator is
 *	actually turning and there is no "HARD_PULSE" waiting to be processed.
 */
 
void Timer_ISR ()
{
	if ( !isRotating )								// Really turning?
		return;										// No, nothing to see here!

	if ( pulseType == HARD_PULSE )					// Pending real index pulse?
		return;										// Yes, ignore the soft one

	pulseType = SOFT_PULSE;							// Indicate we got a timer interrupt
}


/*
 *	Pulse_ISR is the interrupt service routine which is executed any time we see a cam switch
 *	pulse from the rotator. All it does is to record the time at which the pulse occurred.
 *
 *	If the "debounceTime" is non-zero, that indicates we have already seen the pulse and
 *	are deferring processing until the debounce time elapses. In this case we do nothing.
 *
 *	Modified in Version 5.4 to use a longer timeout (2 seconds) on initial startup in order
 *	to account for startup inertia and the possibility that the rotator stopped with the cam
 *	switch closed. Once we have seen 2 cam switch pulses, we reset the timeout value to the
 *	time specified by "ROTATOR_TIMEOUT".
 */

void Pulse_ISR ()
{
	if ( debounceTime )							// Already saw a pulse this cycle?
		return;									// Yes

	if ( hardPulseCount > 1 )					// Teo or more can switch pulses seen?
		rotatorTimeout = ROTATOR_TIMEOUT;		// If yes, set normal timeout value

	hardPulseCount++;							// Bump the counter

	debounceTime = systemTime;					// For pulse processing logic
	pulseTime    = systemTime;					// For timeout function
	pulseType    = HARD_PULSE;					// Real index pulse
	Timer1.restart ();							// Reset the timer
}


/*
 *	Read_Pulse is called from the main loop anytime the rotator "isRotating" and also from
 *	the "Calibrate()" function.
 *	
 *	We check the value of "pulseType" to see if there is actually a real or psuedo index
 *	pulse to be processed. If "NO_PULSE", we do nothing.
 *	
 *	If there is a pulse to be processed, we increment or decrement the "currentAzimuth"
 *	based on the direction of rotation that was set in the "Start_Rotator()" function. We
 *	see if the "targetAzimuth" has been reached, and if so, we stop the rotation.
 *	
 *	If the "pulseType" is "HARD_PULSE", we defer processing it until the "PULSE_DEBOUNCE"
 *	time has elapsed. If the "PULSE_DEBOUNCE" time has passed, we reset the timeout and
 *	debounce variables. 
 */

void Read_Pulse ()
{
	if ( !isRotating )								// Really turning?
		return;										// No, nothing to see here!

	if ( pulseType == NO_PULSE )					// Pulse to process?
		return;										// Nope!

	if ( pulseType == HARD_PULSE )					// Only check debounce for real pulse
	{
		if (( systemTime - debounceTime ) < PULSE_DEBOUNCE )
			return;

		else
			debounceTime = 0;				// Clear the valid pulse received indicator
	}

	currentAzimuth += deltaAzimuth;			// New current azimuth (could be + or -)
	lastGoodAzimuth = currentAzimuth;		// Got a pulse, so it's a good number

	azChanged = true;						// Flag for EEPROM update

	if ( Tgt_Reached ())					// Done rotating?
	{
		targetAzimuth = currentAzimuth;		// Yes!
		Stop_Motor ( "Read_Pulse" );		// Stop the motor
	}
	
	pulseType = NO_PULSE;					// No pulse until we get the next one
	Display_Rotation ();					// Give operator status
}


/*
 *	"Tgt_Reached" returns "true" if the "currentAzimuth" and "targetAzimuths" are
 *	equal, or if the "currentAzimuth" has exceeded the minimum or maximum limit.
 *	If not, it returns false.
 */

bool Tgt_Reached()
{
	if ( currentAzimuth == targetAzimuth )
		return true;


/*
 *	Because of the new goofy azimuth math in Version 3.0, every now and then, the current
 *	azimuth was exceeding the limits, so instead of just the simple test to see if the current
 *	and target azimuths were equal, we also needed to see if the current azimuth had gone
 *	past one of the limits and if so, set it to the limit.
 *	
 *	However, we can't do that if the rotator is being calibrated as the calibrate function
 *	works by forcing the target (and thus the current) azimuth to be way beyond the limits.
 */

	if ( !isCalibrating )							// If not doing a calibration
	{
		if ( currentAzimuth > maxAzimuth )			// Range check the azimuths
		{
			currentAzimuth  = maxAzimuth;
			lastGoodAzimuth = currentAzimuth;
			return true;
		}

		if ( currentAzimuth < minAzimuth )
		{
			currentAzimuth  = minAzimuth;
			lastGoodAzimuth = currentAzimuth;
			return true;
		}
	}

	return false;
}


/*
 *	"Check_Reversal" looks to see if something changed the "targetAzimuth" such that the
 *	rotator is now turning in the wrong direction. This can be done from the N1MM program
 *	or via the rotary encoder.
 *	
 *	If we see this is the case, we just stop the rotation, and the next call to "Start_Rotator"
 *	should turn it around.
 */

void Check_Reversal ()
{
	if ( !isRotating )							// Actually rotating?
		return;									// No, return

	if (( directionRelay == CW ) && ( currentAzimuth >= targetAzimuth ))
		return;

	if (( directionRelay == CCW ) && ( currentAzimuth <= targetAzimuth ))
		return;

	Stop_Motor ( "Ck Reversal" );				// Turn motor off
}


/*
 *	The Read_Encoder function is called via the Arduino's interrupt mechanism, and
 *	never invoked by any other means.
 */

void Read_Encoder ()
{

/*
 *	Read the encoder. The Rotary object actually does the reading of the state of
 *	the encoder pins and determines which way it is moving. If the knob is being
 *	turned counter-clockwise, we decrement "targetAzimuth". If the knob is being
 *	turned clockwise, we increment "targetAzimuth".
 *
 *	We never let the "targetAzimuth" become less than "minAzimuth", or more than
 *	"maxAzimuth".
 *	
 *	If the "isCalibrating" flag is set, we ignore any movement of the encoder knob.
 *	This has the effect of disabling the interrupts without really doing so. If we
 *	were to actually disable the interrupts when the rotator is being calibrated,
 *	the serial communications and the display would also stop working.
 *
 *	Modified in Version 4.0 to use the setting of the azimuth increment toggle switch
 *	to change the target azimuth by 1, 5 or 10 degrees per encoder click. The switch setting
 *	is actually read each time through the "loop()" function to save time in here.
 */

	long	azDelta = timerDegrees * azMultiplier;
	
	if ( isCalibrating )					// Ignore interrupt when rotator is being
		return;								// calibrated
		
	unsigned char Result = Encoder.process ();

	if ( Result == DIR_NONE )				// Encoder didn't move (didn't think this
		return;								// was actually possible, but it is!)

	else if ( Result == DIR_CW )			// Encoder rotated clockwise
		targetAzimuth += azDelta;			// So increment the targetAzimuth
		
	else if ( Result == DIR_CCW )			// Encoder rotated counter-clockwise
		targetAzimuth -= azDelta;			// So decrement the targetAzimuth
		
	if ( targetAzimuth < minAzimuth )		// Limit checks
		targetAzimuth = minAzimuth;

	if ( targetAzimuth > maxAzimuth )
		targetAzimuth = maxAzimuth;

	tgtChanged = true;						// Signal display update needed
}


/*
 *	Say_Hello checks to see if the default  message is already on line 1 of the display, and if
 *	so, just returns. It also returns if the "helloTimeout" since the time some other type of
 *	message was displayed has not expired.
 */

void Say_Hello ()
{
String	Temp_Buffer;

	if ( helloMsg )										// Already displaying message?
		return;											// Don't need to do it again

	if ( isRotating )									// If the rotator is turning
		return;											// Target azimuth is on line 1
		
	helloTime = systemTime;								// Current run time

	if ( helloTime - notHelloTime < helloTimeout )		// Time expired?
		return;											// Nope!

	LCD_Print ( 1, String ( helloString ));				// We're on the air!

	Temp_Buffer = currentAzString;
	Temp_Buffer += Int_2_Az (( currentAzimuth + AZ_ROUNDING ) / timerDegrees );

	LCD_Print ( 2, Temp_Buffer );

	notHelloTime = helloTime;							// Update last time displayed
	helloMsg = true;									// Message is displayed
}


/*
 *	Display_Rotation lets the operator know the rotator is turning by updating the value of the
 *	"currentAzimuth" on the second line of the display.
 *	
 *	Note that we use a temporary variable for the display of the "currentAzimuth". We do
 *	this because when the rotator is being calibrated, it is possible for the "currentAzimuth"
 *	to go negative, which would not display correctly. To avoid this, we will convert negative
 *	azimuth readings to the equivalent positive number. For example -6 degrees => 354
 *	degrees.
 */

void Display_Rotation ()
{
char Temp [4];

	long Temp_Azimuth = currentAzimuth;				// Make a copy
	
	while ( Temp_Azimuth < 0 )						// If negative
		Temp_Azimuth += maxAzimuth;					// Make high positive direction

	Temp_Azimuth += AZ_ROUNDING;					// Round it off
	Temp_Azimuth /= timerDegrees;					// Normalize

	String Az = Int_2_Az ( Temp_Azimuth );			// Temp holder for azimuth
	Az.toCharArray ( Temp, 4 );						// Copy data to character array buffer
	Temp[3] = '\0';									// Null terminator

	lcdDisplay.setCursor ( 13, 1 );					// Azimuth starts at 12th character on line 1
	lcdDisplay.printstr ( Temp );					// Display it

	#if DEBUG
		Serial.print ( "AZ = " );					// Send to serial monitor also
		Serial.println ( Temp );
	#endif
}


/*
 *	Display_Target displays the target azimuth on the first line of the display anytime the
 *	rotator is commanded to turn (except when being calibrated).
 */

void Display_Target ()
{
	if ( isCalibrating )						// If calibrating, no target azimuth display
		return;

	if ( !tgtChanged )							// If the target azimuth hasn't changed
		return;									// No need to re-display it
		
	lcdBuffer_1 = targetAzString;				// Inform operator rotator is turning
	
	lcdBuffer_1 += Int_2_Az (( targetAzimuth + AZ_ROUNDING ) / timerDegrees );
	
	Check_LCD ();								// Immediate display
	
	helloMsg = false;							// We dont use this to display hello msg
	notHelloTime = systemTime;					// Time stamp
	tgtChanged = false;
}


/*
 *	"Check_LCD" looks for data in the LCD buffers, and if anything is in either one,
 *	it uses "LCD_Print" to display them.
 */

void Check_LCD ()
{
	if ( lcdBuffer_1.length ())				// Anything in line 1 buffer?
	{
		LCD_Print ( 1, lcdBuffer_1 );		// Yep!, display it
		lcdBuffer_1 = "";					// Clear the buffer
		helloMsg = false;					// We dont use this to display hello msg
		notHelloTime = systemTime;			// Time stamp
	}

	if ( lcdBuffer_2.length ())				// Anything in line 2 buffer?
	{
		LCD_Print ( 2, lcdBuffer_2 );		// Yep!, display it
		lcdBuffer_2 = "";					// Clear the buffer
	}
}


/*
 *	"LCD_Print" displays something on the LCD display. Arguments are which line to put it
 *	on, and what to put there. Note, we allow the caller to specify lines 1 or 2, and
 *	convert those to 0 and 1.
 */

void LCD_Print ( int line, String info )
{
int		i;										// Loop counter
int	 	lineNumber;								// Adjusted line number
int		bufferLength;							// Actual number of characters in the string
char	buffer[LCD_WIDTH + 4];					// Need a regular array for print function

	if ( line < 1 || line > LCD_HEIGHT )		// Range check
		return;

	lineNumber = line - 1;						// Adjust requested line number
	bufferLength = info.length ();				// Length of data

	if ( bufferLength > LCD_WIDTH )			// Make sure not too long
		bufferLength = LCD_WIDTH;				// If too long, truncate

	info.toCharArray ( buffer, bufferLength + 1 );		// Copy data to internal buffer

	if ( bufferLength < LCD_WIDTH )					// Need padding?
		for ( i = bufferLength; i < LCD_WIDTH; i++ )	// Yep!
			buffer[i] = ' ';

	buffer[LCD_WIDTH] = '\0';					// Need a null at the end
	
	lcdDisplay.setCursor ( 0, lineNumber );		// 1st character position on lineNumber
	lcdDisplay.printstr ( buffer );				// Display it

	#if DEBUG
		Serial.println ( buffer );				// Also send to serial monitor
	#endif
}


/*
 *	"Int_2_Az" converts a number (assumed to be a legitimate real azimuth, i.e., 360 or less)
 *	into a 3 digit String. 
 */

String	Int_2_Az ( long azimuth )
{
String	answer;							// Build the answer here

	if ( azimuth < 10 )					// Single digit number?
		answer = "00";					// Then 2 leading zeroes

	else if ( azimuth < 100 )			// 2 digit number?
		answer = "0";					// Only 1 leading zero

	answer += azimuth;					// Add on the actual number

	return answer;						// And return it
}


/*
 *	"Update_Az" checks to see if the "azChanged" flag is set. If not, it does nothing. If
 *	the flag is set, it checks to see if the rotator has not moved in "EEPROM_TIMEOUT"
 *	milliseconds (set to 10 in "My_Rotator.h"). If that is the case, we update the EEPROM
 *	and turn on the azimuth saved LED.
 */

void Update_Az ()
{
 	if ( !azChanged )										// Azimuth changed?
 		return;												// No, nothing to do here

	if ( systemTime - rotatorStopTime < EEPROM_TIMEOUT )	// Ten seconds since last movement?
		return;												// Nope!

	azChanged = false;										// Unchanged since last save

	EEPROM.put ( AZIMUTH_ADDRESS, currentAzimuth );			// Write new azimuth
	digitalWrite ( SAVED_LED, LED_ON );						// Turn on azimuth saved LED

	lcdBuffer_1 = azSaveString;
}


/*
 *	The "Calibrate" function can be called from the serial input ("F"). The N1MM Rotor
 *	program does not use the command, however it can be sent from SatPC32 using the
 *	manual input option. This function is also called whenever the operator pushes the
 *	encoder knob to activate the encoder switch.
 *	
 *	It should be noted that generally, the rotator is self-calibrating. That is, anytime
 *	a timeout occurrs during normal operation, we assume it's because the rotator hit
 *	the mechanical end stop and we set the "currentAzimuth" and "targetAzimuth" to
 *	either "maxAzimuth" or "minAzimuth" as appropriate.
 *
 *	Note however that sometimes some rotators might miss a cam pulse if the switch is
 *	not setup correctly in the rotator an any point in the rotation. It is important 
 *	to verify the cam switch operation before you put the rotator on the tower or the
 *	roof if it's not already there!
 *	
 *	The function in some ways mimics the rotation handling functions in the main loop,
 *	but handles them a bit differently. Here, we want and expect to see a timeout! The
 *	function takes control away from the main loop movement processing so any commands
 *	received from the serial interface will be ignored until the calibration process is
 *	complete. The "Read_Encoder" function also checks to see if we're in a calibration
 *	cycle and will ignore any movement of the encoder knob.
 */
 
void Calibrate ()
{
	isCalibrating = true;				// Calibration in progress
	
	lcdBuffer_1 = calibrateString;		// Tell operator
	Check_LCD ();						// Force display


/*
 *	See if the rotator thinks it's closer to the minimum or maximum limit. This way, 
 *	we start turning in the nearest direction to minimize the time it takes to do
 *	the calibration (could be as long as about 45 seconds). We have a 50-50 shot at
 *	being correct, depending how far the rotator is out of calibration.
 *	
 *	What we do here is after selecting the most likely direction to turn, we add (or
 *	subtract) 360 degrees to the target azimuth. This should guarantee that we hit the
 *	end stop sooner or later.
 */

	if ( lastGoodAzimuth < ( maxAzimuth / 2 ))			// Closer to 0 degrees?
		targetAzimuth = minAzimuth - maxAzimuth;		// Yes, aim way past there

	else
		targetAzimuth = maxAzimuth * 2;			// Aim way past 360 degrees

	Start_Rotator ();							// Start the rotator

	while ( isRotating )						// As long as still turning
	{
		systemTime = millis ();					// Control is taken away from main loop
		Ck_Reboot ();							// Operator want to reboot?

		if ( Check_Timeout ())					// If it timed out
		{
			isCalibrating = false;				// No longer calibrating
			tgtChanged    = false;
		
			lcdBuffer_1 = calibratedString;		// Show finished
			Check_LCD ();						// Force display
			
			return;								// And we're done
		}
		
		Read_Pulse ();							// Check for an index pulse
	}
	
	tgtChanged = false;
}


/*
 *	"Start_Motor" does just what it says. Note that there is a 10mS delay between the
 *	time the setting of the motor direction relay is established and when we energize
 *	the power relay just to make sure the direction relay has fully operated before
 *	we apply power to the motor.
 */

void Start_Motor ( bool Dir )
{
	digitalWrite ( MOTOR_DIRECTION,  Dir );			// Set the direction select relay
	delay ( 10 );									// Debounce
	digitalWrite ( MOTOR_RUN, RUN );				// Turn motor on

	pulseType    = NO_PULSE;						// No index pulses yet
	isRotating   = true;							// It is turning
	pulseTime    = systemTime;						// Set start time for timeout function
	debounceTime = 0;								// Clear the debounce timer
	rotatorTimeout = INITIAL_TIMEOUT;				// Extra time for inertia
	Timer1.restart ();								// Reset the timer to zero
	digitalWrite ( SAVED_LED, LED_OFF );			// Turn off azimuth saved LED
}


/*
 *	"Stop_Motor" turns the rotator motor off. 
 *	
 *	Again, there is a 10mS delay between turning the power relay off and turning the
 *	direction relay off to make sure the power is really off before operating the
 *	direction relay.
 *	
 *	We also reset the "isRotating" flag and the variables associated with pulse processing
 *	and record the stop time which is used to see if the EEPROM needs to be updated.
 *
 *	The "where" argument is for debugging. If the "DEBUG" symbol is turned on, the function
 *	displays the reason the motor is being stopped.
 */
 
void Stop_Motor ( String where )					// Make sure not turning
{
	Timer1.stop ();									// Stop soft pulse interrupts

	digitalWrite ( MOTOR_RUN, STOP );				// Turn motor off
	delay ( 10 );									// Debounce
	digitalWrite ( MOTOR_DIRECTION, OFF );			// Direction relay off

	isRotating   	= false;						// No longer turning
	pulseTime    	= 0;							// For timeout function
	debounceTime 	= 0;							// For pulse processing function
	pulseType    	= NO_PULSE;						// No index pulses
	hardPulseCount	= 0;							// Reset cam pulse counter for next time
	rotatorTimeout	= INITIAL_TIMEOUT;				// And use long timeout for next time
	rotatorStopTime	= systemTime;					// Time it really stopped

	#if DEBUG
		Serial.print ( "Stop_Motor - " );
		Serial.println ( where );
	#endif
}


/*
 *	"Backup" is used to back the rotator up a little bit when it hits the end stop. 
 *	
 *	It is used by "Check_Timeout" to back the rotator off the mechanical end stop by 
 *	"CAL_ADJUSTMENT" degrees. 
 *	
 *	It does this on the QT. In other words, it doesn't mess with the azimuths of
 *	any of the status flags.
 */

 void Backup ( uint32_t backupTime )
 {
 uint32_t delayTime = backupTime - 20;					// Adjusted for relay bounce delays

 	if ( delayTime <= 0 )								// Nothing to do
 		return;

	Timer1.stop ();

 	digitalWrite ( MOTOR_DIRECTION, !directionRelay );	// Reverse the motor
	delay ( 10 );										// Debounce
	digitalWrite ( MOTOR_RUN, RUN );					// Turn motor on
	
	delay ( delayTime );								// Wait!
	
	digitalWrite ( MOTOR_RUN, STOP );					// Turn motor off
	delay ( 10 );										// Debounce
	digitalWrite ( MOTOR_DIRECTION,  OFF );				// Direction relay off
 }


/*
 *	"Ck_Reboot" looks to see if the operator pressed the reset button on the back of the
 *	controller, and if so, saves the current azimuth and does a software initiated restart.
 */

void Ck_Reboot ()
{
	if ( digitalRead ( SOFT_RESET_PIN ) == LOW )			// Operator want to reboot?
	{
		EEPROM.put ( AZIMUTH_ADDRESS, currentAzimuth );		// Write new azimuth

		Blink_LED ();										// Flash the azimuth saved LED

		soft_restart ();									// Reboot
	}
}


/*
 *	Whenever a timeout or soft reset occurs, we flash the azimuth saved led.
 */

void Blink_LED ()
{
	for ( int i = 0; i < 5; i++ )
	{
		digitalWrite ( SAVED_LED, LED_OFF );		// Turn the azimuth saved LED off
		delay ( 100 );								// Wait 100 milliseconds
		digitalWrite ( SAVED_LED, LED_ON );			// Turn the azimuth saved LED on
		delay ( 100 );								// Wait another 100 milliseconds
	}
}
