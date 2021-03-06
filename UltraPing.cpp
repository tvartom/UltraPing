// ---------------------------------------------------------------------------
// UltraPing, forked by Lasse Löfquist - ultraping@tvartom.com
// Copyright 2017 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html
//
// Forked from Tim Eckel's excellent NewPing
// ---------------------------------------------------------------------------
// NewPing
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2016 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html
// ---------------------------------------------------------------------------
// See "UltraPing.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------

#include <UltraPing.h>


// ---------------------------------------------------------------------------
// UltraPing constructor
// ---------------------------------------------------------------------------

UltraPing::UltraPing(uint8_t trigger_pin, uint8_t echo_pin, unsigned int max_distance) {
#if ULTRAPING_DO_BITWISE == true
	_triggerBit = digitalPinToBitMask(trigger_pin); // Get the port register bitmask for the trigger pin.
	_echoBit = digitalPinToBitMask(echo_pin);       // Get the port register bitmask for the echo pin.

	_triggerOutput = portOutputRegister(digitalPinToPort(trigger_pin)); // Get the output port register for the trigger pin.
	_echoInput = portInputRegister(digitalPinToPort(echo_pin));         // Get the input port register for the echo pin.

	_triggerMode = (uint8_t *) portModeRegister(digitalPinToPort(trigger_pin)); // Get the port mode register for the trigger pin.
#else
	_triggerPin = trigger_pin;
	_echoPin = echo_pin;
#endif

	set_max_distance(max_distance); // Call function to set the max sensor distance.

#if (defined (__arm__) && defined (TEENSYDUINO)) || ULTRAPING_DO_BITWISE != true
	pinMode(echo_pin, INPUT);     // Set echo pin to input (on Teensy 3.x (ARM), pins default to disabled, at least one pinMode() is needed for GPIO mode).
	pinMode(trigger_pin, OUTPUT); // Set trigger pin to output (on Teensy 3.x (ARM), pins default to disabled, at least one pinMode() is needed for GPIO mode).
#endif

#if defined (ARDUINO_AVR_YUN)
	pinMode(echo_pin, INPUT);     // Set echo pin to input for the Arduino Yun, not sure why it doesn't default this way.
#endif

#if ULTRAPING_ONE_PIN_ENABLED != true && ULTRAPING_DO_BITWISE == true
	*_triggerMode |= _triggerBit; // Set trigger pin to output.
#endif
}


// ---------------------------------------------------------------------------
// Standard ping methods
// ---------------------------------------------------------------------------

unsigned int UltraPing::ping(unsigned int max_distance) {
	if (max_distance > 0) set_max_distance(max_distance); // Call function to set a new max sensor distance.

	if (!ping_trigger()) return  ULTRAPING_NO_ECHO; // Trigger a ping, if it returns false, return NO_ECHO to the calling function.

	while (ULTRAPING_ISACTIVE(readEcho())) {                // Wait for the ping echo.
		if (micros() > _max_time) return ULTRAPING_NO_ECHO; // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
	}

	return (micros() - (_max_time - _maxEchoTime) - ULTRAPING_PING_OVERHEAD); // Calculate ping time, include overhead.
}

unsigned int UltraPing::ping_threshold(unsigned int threshold_distance, unsigned int max_distance) {
	unsigned int hit[] = {ULTRAPING_NO_ECHO};
	ping_multi(hit, 1, threshold_distance, max_distance);
	return hit[0];
}

unsigned int UltraPing::ping_multi(unsigned int hit[], unsigned int maximum_hits, unsigned int threshold_distance, unsigned int max_distance) {
	if (max_distance > 0) set_max_distance(max_distance); // Call function to set a new max sensor distance.

	unsigned int offset = 0;
	unsigned int i = 0;
	while(i < maximum_hits) {
		if (!ping_trigger()) return 0; // Trigger a ping, if it returns false, return 0 hits (Something wrong)
		unsigned long first_max_time = _max_time; //The max_time from first ping, is also used later as max for second ping.
		unsigned long first_start = (_max_time - _maxEchoTime) - ULTRAPING_PING_OVERHEAD;

		while (ULTRAPING_ISACTIVE(readEcho())) {                // Wait for the ping echo.
			if (micros() > _max_time) return i; // Stop the loop and return hits so far.
		}
		unsigned int first_length = micros() - first_start; // Calculate ping time, for first echo.

		if (offset == 0) { //Only first loop
			if (first_length > threshold_distance * ULTRAPING_US_ROUNDTRIP_LENGTH) {
				offset = hit[i++] = first_length; //If first echo, above threshold, register as a hit.
				if(i >= maximum_hits) return i; //If this method is used with maximum_hits == 1, and first_length is beyond threshold
			} else {
				offset = threshold_distance * ULTRAPING_US_ROUNDTRIP_LENGTH;
			}
		}
		// #######################################################################################################################################################
		while (micros() < first_start + offset);//Wait before we start next ping, to let secondary echos from first ping return before first echo from second ping.
		// #######################################################################################################################################################

		if (!ping_trigger()) return 0; // Trigger a second ping, if it returns false, return 0 hits (Something wrong)
		while (ULTRAPING_ISACTIVE(readEcho())) {                // Wait for the ping echo.
			if (micros() > first_max_time) return i; // No more echo within range from first ping, return result
		}
		unsigned long second_end_time = micros();

		unsigned long lengthSecond = second_end_time - (_max_time - _maxEchoTime) - ULTRAPING_PING_OVERHEAD;
		if (lengthSecond < ULTRAPING_THREE_QUARTERS(first_length)) { //If second ping is (significant) shorter than first, it must be an echo from first ping.
			//New hit!
			// Calculate ping time from the start of first ping, and register in hit
			// Push offset (waiting time) forward, so we don't find this hit again.
			// Increase number of total echos found. (i)
			offset = hit[i++] = second_end_time - first_start;
		} else {
			//Too long, might be first echo from second ping, for next try increase the waiting time for second ping.
			offset += first_length / 2;
		}
		if(i < maximum_hits) { //Only wait, if we are going to do more tries
			delay(ULTRAPING_PING_MEDIAN_DELAY / 1000); // Wait until all echos ebb away
		}
	}
	return i; //Maximum number of hits found, return those found so far
}

unsigned long UltraPing::ping_length(unsigned int max_distance) {
	unsigned long echoTime = ping(max_distance); // Calls the ping method and returns with the ping echo distance in uS.
	return ULTRAPING_US_2_LENGTH_UNIT(echoTime); // Convert uS to length unit.
}


unsigned long UltraPing::ping_median(uint8_t it, unsigned int max_distance) {
	unsigned int uS[it], last;
	uint8_t j, i = 0;
	unsigned long t;
	uS[0] = ULTRAPING_NO_ECHO;

	while (i < it) {
		t = micros();                  // Start ping timestamp.
		last = ping(max_distance);  // Send ping.

		if (last != ULTRAPING_NO_ECHO) {         // Ping in range, include as part of median.
			if (i > 0) {               // Don't start sort till second ping.
				for (j = i; j > 0 && uS[j - 1] < last; j--) // Insertion sort loop.
					uS[j] = uS[j - 1];                      // Shift ping array to correct position for sort insertion.
			} else j = 0;              // First ping is sort starting point.
			uS[j] = last;              // Add last ping to array in sorted position.
			i++;                       // Move to next ping.
		} else it--;                   // Ping out of range, skip and don't include as part of median.

		if (i < it && micros() - t < ULTRAPING_PING_MEDIAN_DELAY)
			delay((ULTRAPING_PING_MEDIAN_DELAY + t - micros()) / 1000); // Millisecond delay between pings.

	}
	return (uS[it >> 1]); // Return the ping distance median.
}

// -------------------------------------------------------------------------------------
// Input and output methods (Bitwise or normal)
// All of them are marked inline, so compiler will probably inline them at compile-time.
// -------------------------------------------------------------------------------------

inline boolean UltraPing::readEcho() {
	#if ULTRAPING_DO_BITWISE == true
		return *_echoInput & _echoBit;
	#else
		return digitalRead(_echoPin);
	#endif
}

inline void UltraPing::setTriggerActive() {
	#if ULTRAPING_DO_BITWISE == true
		*_triggerOutput |= _triggerBit;    // Set trigger pin high, this tells the sensor to send out a ping.
	#else
		digitalWrite(_triggerPin, HIGH);   // Set trigger pin high, this tells the sensor to send out a ping.
	#endif
}
inline void UltraPing::setTriggerNotActive() {
	#if ULTRAPING_DO_BITWISE == true
		*_triggerOutput &= ~_triggerBit;   // Set the trigger pin low.
	#else
		digitalWrite(_triggerPin, LOW);    // Set the trigger pin low.
	#endif
}
#if ULTRAPING_ONE_PIN_ENABLED == true
	inline void UltraPing::onePinSetTriggerMode() {
		#if ULTRAPING_DO_BITWISE == true
			*_triggerMode |= _triggerBit;  // Set trigger pin to output.
		#else
			pinMode(_triggerPin, OUTPUT); // Set trigger pin to output.
		#endif
	}
	inline void UltraPing::onePinSetEchoMode() {
		#if ULTRAPING_DO_BITWISE == true
			*_triggerMode &= ~_triggerBit; // Set trigger pin to input (when using one Arduino pin, this is technically setting the echo pin to input as both are tied to the same Arduino pin).
		#else
			pinMode(_triggerPin, INPUT);  // Set trigger pin to input (when using one Arduino pin, this is technically setting the echo pin to input as both are tied to the same Arduino pin).
		#endif
	}
#endif

// ---------------------------------------------------------------------------
// Standard and timer interrupt ping method support functions (not called directly)
// ---------------------------------------------------------------------------

boolean UltraPing::ping_trigger() {
	#if ULTRAPING_ONE_PIN_ENABLED == true
		onePinSetTriggerMode();
	#endif

	setTriggerNotActive();             // Should already be low, but this will make sure it is.
	delayMicroseconds(4);              // Wait for pin to go low.
	setTriggerActive();                // Set trigger pin high, this tells the sensor to send out a ping.
	delayMicroseconds(10);             // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
	setTriggerNotActive();             // Set trigger pin back to low.

	#if ULTRAPING_ONE_PIN_ENABLED == true
		onePinSetEchoMode(); // Set trigger pin to input (when using one Arduino pin, this is technically setting the echo pin to input as both are tied to the same Arduino pin).
	#endif

	if (ULTRAPING_ISACTIVE(readEcho())) return false;               // Previous ping hasn't finished, abort.
	_max_time = micros() + _maxEchoTime + ULTRAPING_MAX_SENSOR_DELAY; // Maximum time we'll wait for ping to start (most sensors are <450uS, the SRF06 can take up to 34,300uS!)
	while (ULTRAPING_ISNOTACTIVE(readEcho())) {                       // Wait for ping to start.
		if (micros() > _max_time) return false;             // Took too long to start, abort.
	}

	_max_time = micros() + _maxEchoTime; // Ping started, set the time-out.
	return true;                         // Ping started successfully.
}


void UltraPing::set_max_distance(unsigned int max_distance) {
#if ULTRAPING_ROUNDING_ENABLED == false
	_maxEchoTime = min(max_distance + 1, (unsigned int) ULTRAPING_MAX_SENSOR_DISTANCE + 1) * ULTRAPING_US_ROUNDTRIP_LENGTH; // Calculate the maximum distance in uS (no rounding).
#else
	_maxEchoTime = min(max_distance, (unsigned int) ULTRAPING_MAX_SENSOR_DISTANCE) * ULTRAPING_US_ROUNDTRIP_LENGTH + (ULTRAPING_US_ROUNDTRIP_LENGTH / 2); // Calculate the maximum distance in uS.
#endif
}


#if ULTRAPING_TIMER_ENABLED == true && ULTRAPING_DO_BITWISE == true

// ---------------------------------------------------------------------------
// Timer interrupt ping methods (won't work with non-AVR, ATmega128 and all ATtiny microcontrollers)
// ---------------------------------------------------------------------------

void UltraPing::ping_timer(void (*userFunc)(void), unsigned int max_distance) {
	if (max_distance > 0) set_max_distance(max_distance); // Call function to set a new max sensor distance.

	if (!ping_trigger()) return;         // Trigger a ping, if it returns false, return without starting the echo timer.
	timer_us(ULTRAPING_ECHO_TIMER_FREQ, userFunc); // Set ping echo timer check every ECHO_TIMER_FREQ uS.
}


boolean UltraPing::check_timer() {
	if (micros() > _max_time) { // Outside the time-out limit.
		timer_stop();           // Disable timer interrupt
		return false;           // Cancel ping timer.
	}

	if (ULTRAPING_ISACTIVE(readEcho())) {    // Ping echo received.
		timer_stop();                // Disable timer interrupt
		ping_result = (micros() - (_max_time - _maxEchoTime) - ULTRAPING_PING_TIMER_OVERHEAD); // Calculate ping time including overhead.
		return true;                 // Return ping echo true.
	}

	return false; // Return false because there's no ping echo yet.
}


// ---------------------------------------------------------------------------
// Timer2/Timer4 interrupt methods (can be used for non-ultrasonic needs)
// ---------------------------------------------------------------------------

// Variables used for timer functions
void (*intFunc)();
void (*intFunc2)();
unsigned long _ms_cnt_reset;
volatile unsigned long _ms_cnt;
#if defined(__arm__) && defined(TEENSYDUINO)
	IntervalTimer itimer;
#endif


void UltraPing::timer_us(unsigned int frequency, void (*userFunc)(void)) {
	intFunc = userFunc; // User's function to call when there's a timer event.
	timer_setup();      // Configure the timer interrupt.

#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
	OCR4C = min((frequency>>2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
	TIMSK4 = (1<<TOIE4);                  // Enable Timer4 interrupt.
#elif defined (__arm__) && defined (TEENSYDUINO) // Timer for Teensy 3.x
	itimer.begin(userFunc, frequency);           // Really simple on the Teensy 3.x, calls userFunc every 'frequency' uS.
#else
	OCR2A = min((frequency>>2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
	TIMSK2 |= (1<<OCIE2A);                // Enable Timer2 interrupt.
#endif
}


void UltraPing::timer_ms(unsigned long frequency, void (*userFunc)(void)) {
	intFunc = UltraPing::timer_ms_cntdwn;  // Timer events are sent here once every ms till user's frequency is reached.
	intFunc2 = userFunc;                 // User's function to call when user's frequency is reached.
	_ms_cnt = _ms_cnt_reset = frequency; // Current ms counter and reset value.
	timer_setup();                       // Configure the timer interrupt.

#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
	OCR4C = 249;           // Every count is 4uS, so 1ms = 250 counts - 1.
	TIMSK4 = (1<<TOIE4);   // Enable Timer4 interrupt.
#elif defined (__arm__) && defined (TEENSYDUINO)  // Timer for Teensy 3.x
	itimer.begin(UltraPing::timer_ms_cntdwn, 1000); // Set timer to 1ms (1000 uS).
#else
	OCR2A = 249;           // Every count is 4uS, so 1ms = 250 counts - 1.
	TIMSK2 |= (1<<OCIE2A); // Enable Timer2 interrupt.
#endif
}


void UltraPing::timer_stop() { // Disable timer interrupt.
#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
	TIMSK4 = 0;
#elif defined (__arm__) && defined (TEENSYDUINO) // Timer for Teensy 3.x
	itimer.end();
#else
	TIMSK2 &= ~(1<<OCIE2A);
#endif
}


// ---------------------------------------------------------------------------
// Timer2/Timer4 interrupt method support functions (not called directly)
// ---------------------------------------------------------------------------

void UltraPing::timer_setup() {
#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
	timer_stop(); // Disable Timer4 interrupt.
	TCCR4A = TCCR4C = TCCR4D = TCCR4E = 0;
	TCCR4B = (1<<CS42) | (1<<CS41) | (1<<CS40) | (1<<PSR4); // Set Timer4 prescaler to 64 (4uS/count, 4uS-1020uS range).
	TIFR4 = (1<<TOV4);
	TCNT4 = 0;    // Reset Timer4 counter.
#elif defined (__AVR_ATmega8__) || defined (__AVR_ATmega16__) || defined (__AVR_ATmega32__) || defined (__AVR_ATmega8535__) // Alternate timer commands for certain microcontrollers.
	timer_stop();                 // Disable Timer2 interrupt.
	ASSR &= ~(1<<AS2);            // Set clock, not pin.
	TCCR2 = (1<<WGM21 | 1<<CS22); // Set Timer2 to CTC mode, prescaler to 64 (4uS/count, 4uS-1020uS range).
	TCNT2 = 0;                    // Reset Timer2 counter.
#elif defined (__arm__) && defined (TEENSYDUINO)
	timer_stop(); // Stop the timer.
#else
	timer_stop();        // Disable Timer2 interrupt.
	ASSR &= ~(1<<AS2);   // Set clock, not pin.
	TCCR2A = (1<<WGM21); // Set Timer2 to CTC mode.
	TCCR2B = (1<<CS22);  // Set Timer2 prescaler to 64 (4uS/count, 4uS-1020uS range).
	TCNT2 = 0;           // Reset Timer2 counter.
#endif
}


void UltraPing::timer_ms_cntdwn() {
	if (!_ms_cnt--) {            // Count down till we reach zero.
		intFunc2();              // Scheduled time reached, run the main timer event function.
		_ms_cnt = _ms_cnt_reset; // Reset the ms timer.
	}
}

#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
ISR(TIMER4_OVF_vect) {
	intFunc(); // Call wrapped function.
}
#elif defined (__AVR_ATmega8__) || defined (__AVR_ATmega16__) || defined (__AVR_ATmega32__) || defined (__AVR_ATmega8535__) // Alternate timer commands for certain microcontrollers.
ISR(TIMER2_COMP_vect) {
	intFunc(); // Call wrapped function.
}
#elif defined (__arm__)
// Do nothing...
#else
ISR(TIMER2_COMPA_vect) {
	intFunc(); // Call wrapped function.
}
#endif


#endif


// ---------------------------------------------------------------------------
// Conversion method (rounds result to nearest cm or inch).
// ---------------------------------------------------------------------------
unsigned int UltraPing::convert_length(unsigned int echoTime) {
	return ULTRAPING_US_2_LENGTH_UNIT(echoTime); // Convert uS to length unit.
}

