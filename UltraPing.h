// ---------------------------------------------------------------------------
// UltraPing 1.0, forked from Tim Eckel's excellent NewPing
//
// AUTHOR/LICENSE:
// Lasse Löfquist - ultraping@tvartom.com
// Copyright 2017 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html
//
// Forked from Tim Eckel's excellent NewPing
//
// BACKGROUND:
// My first project with a ultrasonic sensor required a HC-SR04 to see beyond
// the first echo. I couldn't find anyone else who had solved this with pure
// software, but I realized it could be made.
// Instead of starting from scratch, I started to modify Tim Eckel's excellent
// library. Tim's library also taught me a lot of ultrasonic sensor programming.
// Would be honored if Tim wants to include any of my code in his project.
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
// From original NewPing Library:
//
// AUTHOR/LICENSE:
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2016 License: GNU GPL v3 http://www.gnu.org/licenses/gpl.html
//
// LINKS:
// Project home: https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
// Blog: http://arduino.cc/forum/index.php/topic,106043.0.html
//
// DISCLAIMER:
// This software is furnished "as is", without technical support, and with no 
// warranty, express or implied, as to its usefulness for any purpose.
//
// BACKGROUND:
// When I first received an ultrasonic sensor I was not happy with how poorly
// it worked. Quickly I realized the problem wasn't the sensor, it was the
// available ping and ultrasonic libraries causing the problem. The NewPing
// library totally fixes these problems, adds many new features, and breaths
// new life into these very affordable distance sensors. 
//
// ---------------------------------------------------------------------------
//
// FEATURES:
// * Works with many different ultrasonic sensors: SR04, SRF05, SRF06, DYP-ME007, URM37 & Parallax PING))).
// * Compatible with the entire Arduino line-up (and clones), Teensy family (including $19 96Mhz 32 bit Teensy 3.2) and non-AVR microcontrollers.
// * Interface with all but the SRF06 sensor using only one Arduino pin.
// * Doesn't lag for a full second if no ping/echo is received.
// * Ping sensors consistently and reliably at up to 30 times per second.
// * Timer interrupt method for event-driven sketches.
// * Built-in digital filter method ping_median() for easy error correction.
// * Uses port registers for a faster pin interface and smaller code size.
// * Allows you to set a maximum distance where pings beyond that distance are read as no ping "clear".
// * Ease of using multiple sensors (example sketch with 15 sensors).
// * More accurate distance calculation (cm, inches & uS).
// * Doesn't use pulseIn, which is slow and gives incorrect results with some ultrasonic sensor models.
// * Possible to see beyond first echo, and set threshold for first measured distance. (Exprimental)
// * Actively developed with features being added and bugs/issues addressed.
//
// CONSTRUCTOR:
//   UltraPing sonar(trigger_pin, echo_pin [, max_distance])
//     trigger_pin & echo_pin - Arduino pins connected to sensor trigger and echo.
//       NOTE: To use the same Arduino pin for trigger and echo, specify the same pin for both values.
//     max_distance - [Optional] Maximum distance you wish to sense. Default=500cm.
//
// METHODS:
//   sonar.ping([max_distance]) - Send a ping and get the echo time (in microseconds) as a result. [max_distance] allows you to optionally set a new max distance.
//   sonar.ping_length([max_distance]) - Send a ping and get the distance in whole length units. [max_distance] allows you to optionally set a new max distance.
//   sonar.ping_median(iterations [, max_distance]) - Do multiple pings (default=5), discard out of range pings and return median in microseconds. [max_distance] allows you to optionally set a new max distance.
//   sonar.ping_multi(hits[], maximum_hits, [threshold_distance], [max_distance]) - Exprimental! Detects several echo at different distance and return number of hits. Echo times of hits in the array.
//   ping_threshold(threshold_distance, [max_distance]) - Exprimental! Return echo time for first echo beyond threshold_distance. (Uses ping_multi internal)
//   UltraPing::convert_length(echoTime) - Convert echoTime from microseconds to length unit (rounds to nearest integer). Depends on LENGTH_UNIT_CM or LENGTH_UNIT_INCH
//   sonar.ping_timer(function [, max_distance]) - Send a ping and call function to test if ping is complete. [max_distance] allows you to optionally set a new max distance.
//   sonar.check_timer() - Check if ping has returned within the set distance limit.
//   UltraPing::timer_us(frequency, function) - Call function every frequency microseconds.
//   UltraPing::timer_ms(frequency, function) - Call function every frequency milliseconds.
//   UltraPing::timer_stop() - Stop the timer.
//
// HISTORY UltraPing:
//  2017-01-29 UltraPing v1.0 - Lasse Löfquist forked NewPing, renamed to
//  UltraPing.
//  Some mayor refactoring, made to more maintanable code, and possibility
//  to develop ping_multi.
//  Parallel logic removed, replaced by inline methods for IO and precompiler directives.
//  Length unit is controlled by precompiler directive, and only one unit
//  is possible to use at a time. Methods with cm or inch in signature are
//  replaced by a universal length unit-variant.
//  New experimental features: ping_multi and ping_threshold.
//  Need to be tested, might have broken something.
//
//
// NewPing Library history:
// 07/30/2016 v1.8 - Added support for non-AVR microcontrollers. For non-AVR
//   microcontrollers, advanced ping_timer() timer methods are disabled due to
//   inconsistencies or no support at all between platforms. However, standard
//   ping methods are all supported. Added new optional variable to ping(),
//   ping_in(), ping_cm(), ping_median(), and ping_timer() methods which allows
//   you to set a new maximum distance for each ping. Added support for the
//   ATmega16, ATmega32 and ATmega8535 microcontrollers. Changed convert_cm()
//   and convert_in() methods to static members. You can now call them without
//   an object. For example: cm = NewPing::convert_cm(distance);
//
// 09/29/2015 v1.7 - Removed support for the Arduino Due and Zero because
//   they're both 3.3 volt boards and are not 5 volt tolerant while the HC-SR04
//   is a 5 volt sensor.  Also, the Due and Zero don't support pin manipulation
//   compatibility via port registers which can be done (see the Teensy 3.2).
//
// 06/17/2014 v1.6 - Corrected delay between pings when using ping_median()
//   method. Added support for the URM37 sensor (must change URM37_ENABLED from
//   false to true). Added support for Arduino microcontrollers like the $20
//   32 bit ARM Cortex-M4 based Teensy 3.2. Added automatic support for the
//   Atmel ATtiny family of microcontrollers. Added timer support for the
//   ATmega8 microcontroller. Rounding disabled by default, reduces compiled
//   code size (can be turned on with ROUNDING_ENABLED switch). Added
//   TIMER_ENABLED switch to get around compile-time "__vector_7" errors when
//   using the Tone library, or you can use the toneAC, NewTone or
//   TimerFreeTone libraries: https://bitbucket.org/teckel12/arduino-toneac/
//   Other speed and compiled size optimizations.
//
// 08/15/2012 v1.5 - Added ping_median() method which does a user specified
//   number of pings (default=5) and returns the median ping in microseconds
//   (out of range pings ignored). This is a very effective digital filter.
//   Optimized for smaller compiled size (even smaller than sketches that
//   don't use a library).
//
// 07/14/2012 v1.4 - Added support for the Parallax PING)))� sensor. Interface
//   with all but the SRF06 sensor using only one Arduino pin. You can also
//   interface with the SRF06 using one pin if you install a 0.1uf capacitor
//   on the trigger and echo pins of the sensor then tie the trigger pin to
//   the Arduino pin (doesn't work with Teensy). To use the same Arduino pin
//   for trigger and echo, specify the same pin for both values. Various bug
//   fixes.
//
// 06/08/2012 v1.3 - Big feature addition, event-driven ping! Uses Timer2
//   interrupt, so be mindful of PWM or timing conflicts messing with Timer2
//   may cause (namely PWM on pins 3 & 11 on Arduino, PWM on pins 9 and 10 on
//   Mega, and Tone library). Simple to use timer interrupt functions you can
//   use in your sketches totally unrelated to ultrasonic sensors (don't use if
//   you're also using NewPing's ping_timer because both use Timer2 interrupts).
//   Loop counting ping method deleted in favor of timing ping method after
//   inconsistent results kept surfacing with the loop timing ping method.
//   Conversion to cm and inches now rounds to the nearest cm or inch. Code
//   optimized to save program space and fixed a couple minor bugs here and
//   there. Many new comments added as well as line spacing to group code
//   sections for better source readability.
//
// 05/25/2012 v1.2 - Lots of code clean-up thanks to Arduino Forum members.
//   Rebuilt the ping timing code from scratch, ditched the pulseIn code as it
//   doesn't give correct results (at least with ping sensors). The NewPing
//   library is now VERY accurate and the code was simplified as a bonus.
//   Smaller and faster code as well. Fixed some issues with very close ping
//   results when converting to inches. All functions now return 0 only when
//   there's no ping echo (out of range) and a positive value for a successful
//   ping. This can effectively be used to detect if something is out of range
//   or in-range and at what distance. Now compatible with Arduino 0023.
//
// 05/16/2012 v1.1 - Changed all I/O functions to use low-level port registers
//   for ultra-fast and lean code (saves from 174 to 394 bytes). Tested on both
//   the Arduino Uno and Teensy 2.0 but should work on all Arduino-based
//   platforms because it calls standard functions to retrieve port registers
//   and bit masks. Also made a couple minor fixes to defines.
//
// 05/15/2012 v1.0 - Initial release.
// ---------------------------------------------------------------------------

#ifndef UltraPing_h
#define UltraPing_h

#if defined (ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include <WProgram.h>
	#include <pins_arduino.h>
#endif

#if defined (__AVR__)
	#include <avr/io.h>
	#include <avr/interrupt.h>
#endif

#if !defined(ULTRAPING_LENGTH_UNIT_CM) && !defined (ULTRAPING_LENGTH_UNIT_INCH)
	#define ULTRAPING_LENGTH_UNIT_CM
	//#define ULTRAPING_LENGTH_UNIT_INCH
#endif

#if defined (ULTRAPING_LENGTH_UNIT_CM)
	#define ULTRAPING_US_ROUNDTRIP_LENGTH 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space. Default=57
#elif defined (ULTRAPING_LENGTH_UNIT_INCH)
	#define ULTRAPING_US_ROUNDTRIP_LENGTH 146     // Microseconds (uS) it takes sound to travel round-trip 1 inch (2 inches total), uses integer to save compiled code space. Defalult=146
#else
Choose one length unit!
#endif
// Shouldn't need to change these values unless you have a specific need to do so.
#ifndef ULTRAPING_MAX_SENSOR_DISTANCE
	#define ULTRAPING_MAX_SENSOR_DISTANCE 500 // In length unit (define LENGTH_UNIT_CM or LENGTH_UNIT_INCH) Maximum sensor distance can be as high as 500cm, (~200inch) no reason to wait for ping longer than sound takes to travel this distance and back. Default=500
#endif
#ifndef ULTRAPING_ONE_PIN_ENABLED
	#define ULTRAPING_ONE_PIN_ENABLED true    // Set to "false" to disable one pin mode which saves around 14-26 bytes of binary size. Default=true
#endif
#ifndef ULTRAPING_ROUNDING_ENABLED
	#define ULTRAPING_ROUNDING_ENABLED false  // Set to "true" to enable distance rounding which also adds 64 bytes to binary size. Default=false
#endif
#ifndef ULTRAPING_URM37_ENABLED
	#define ULTRAPING_URM37_ENABLED false     // Set to "true" to enable support for the URM37 sensor in PWM mode. Default=false
#endif
#ifndef ULTRAPING_TIMER_ENABLED
	#define ULTRAPING_TIMER_ENABLED true      // Set to "false" to disable the timer ISR (if getting "__vector_7" compile errors set this to false). Default=true
#endif


// Probably shouldn't change these values unless you really know what you're doing.
#define ULTRAPING_NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_distance. Default=0
#define ULTRAPING_MAX_SENSOR_DELAY 5800   // Maximum uS it takes for sensor to start the ping. Default=5800
#define ULTRAPING_ECHO_TIMER_FREQ 24      // Frequency to check for a ping echo (every 24uS is about 0.4cm accuracy). Default=24
#define ULTRAPING_PING_MEDIAN_DELAY 29000 // Microsecond delay between pings in the ping_median method. Default=29000
#define ULTRAPING_PING_OVERHEAD 5         // Ping overhead in microseconds (uS). Default=5
#define ULTRAPING_PING_TIMER_OVERHEAD 13  // Ping timer overhead in microseconds (uS). Default=13

#if ULTRAPING_URM37_ENABLED == true
	#undef  ULTRAPING_US_ROUNDTRIP_LENGTH
	#if defined (ULTRAPING_LENGTH_UNIT_CM)
		#define ULTRAPING_US_ROUNDTRIP_LENGTH 50      // Every 50uS PWM signal is low indicates 1cm distance. Default=50
	#elif defined (ULTRAPING_LENGTH_UNIT_INCH)
		#define ULTRAPING_US_ROUNDTRIP_LENGTH 127 // If 50uS is 1cm, 1 inch would be 127uS (50 x 2.54 = 127). Default=127
	#endif

	#define ULTRAPING_ISACTIVE(VALUE) (!(VALUE))
	#define ULTRAPING_ISNOTACTIVE(VALUE) (VALUE)
#else
	#define ULTRAPING_ISACTIVE(VALUE) (VALUE)
	#define ULTRAPING_ISNOTACTIVE(VALUE) (!(VALUE))
#endif

//Used in ping_multi
#define ULTRAPING_THREE_QUARTERS(VALUE) (((VALUE) / 2 + (VALUE) / 4)) // Bitwise approx for VALUE * .75


// Conversion from uS to distance
#if ULTRAPING_ROUNDING_ENABLED == false
	#define ULTRAPING_US_2_LENGTH_UNIT(echoTime) (echoTime / ULTRAPING_US_ROUNDTRIP_LENGTH)
#else
	//(round result to nearest cm or inch).
	#define ULTRAPING_US_2_LENGTH_UNIT(echoTime) (max(((unsigned int)echoTime + ULTRAPING_US_ROUNDTRIP_LENGTH / 2) / ULTRAPING_US_ROUNDTRIP_LENGTH, (echoTime ? 1 : 0)))
#endif

// Detect non-AVR microcontrollers (Teensy 3.x, Arduino DUE, etc.) and don't use port registers or timer interrupts as required.
#if (defined (__arm__) && defined (TEENSYDUINO))
	#undef  ULTRAPING_PING_OVERHEAD
	#define ULTRAPING_PING_OVERHEAD 1
	#undef  ULTRAPING_PING_TIMER_OVERHEAD
	#define ULTRAPING_PING_TIMER_OVERHEAD 1
	#define ULTRAPING_DO_BITWISE true
#elif !defined (__AVR__)
	#undef  ULTRAPING_PING_OVERHEAD
	#define ULTRAPING_PING_OVERHEAD 1
	#undef  ULTRAPING_PING_TIMER_OVERHEAD
	#define ULTRAPING_PING_TIMER_OVERHEAD 1
	#undef  ULTRAPING_TIMER_ENABLED
	#define ULTRAPING_TIMER_ENABLED false
	#define ULTRAPING_DO_BITWISE false
#else
	#define ULTRAPING_DO_BITWISE true
#endif

// Disable the timer interrupts when using ATmega128 and all ATtiny microcontrollers.
#if defined (__AVR_ATmega128__) || defined (__AVR_ATtiny24__) || defined (__AVR_ATtiny44__) || defined (__AVR_ATtiny84__) || defined (__AVR_ATtiny25__) || defined (__AVR_ATtiny45__) || defined (__AVR_ATtiny85__) || defined (__AVR_ATtiny261__) || defined (__AVR_ATtiny461__) || defined (__AVR_ATtiny861__) || defined (__AVR_ATtiny43U__)
	#undef  ULTRAPING_TIMER_ENABLED
	#define ULTRAPING_TIMER_ENABLED false
#endif

// Define timers when using ATmega8, ATmega16, ATmega32 and ATmega8535 microcontrollers.
#if defined (__AVR_ATmega8__) || defined (__AVR_ATmega16__) || defined (__AVR_ATmega32__) || defined (__AVR_ATmega8535__)
	#define OCR2A OCR2
	#define TIMSK2 TIMSK
	#define OCIE2A OCIE2
#endif

class UltraPing {
	public:
		UltraPing(uint8_t trigger_pin, uint8_t echo_pin, unsigned int max_distance = ULTRAPING_MAX_SENSOR_DISTANCE);
		unsigned int ping(unsigned int max_distance = 0);

		unsigned int ping_multi(unsigned int hits[], unsigned int maximum_hits, unsigned int threshold_distance = 0, unsigned int max_distance = 0);
		unsigned int ping_threshold(unsigned int threshold_distance, unsigned int max_distance = 0);

		unsigned long ping_length(unsigned int max_distance = 0);
		unsigned long ping_median(uint8_t it = 5, unsigned int max_distance = 0);
		static unsigned int convert_length(unsigned int echoTime);
#if ULTRAPING_TIMER_ENABLED == true
		void ping_timer(void (*userFunc)(void), unsigned int max_distance = 0);
		boolean check_timer();
		unsigned long ping_result;
		static void timer_us(unsigned int frequency, void (*userFunc)(void));
		static void timer_ms(unsigned long frequency, void (*userFunc)(void));
		static void timer_stop();
#endif
	private:
		inline boolean readEcho();
		inline void setTriggerActive();
		inline void setTriggerNotActive();
#if ULTRAPING_ONE_PIN_ENABLED == true
		inline void onePinSetTriggerMode();
		inline void onePinSetEchoMode();
#endif

		boolean ping_trigger();
		void set_max_distance(unsigned int max_distance);
#if ULTRAPING_TIMER_ENABLED == true
		boolean ping_trigger_timer(unsigned int trigger_delay);
		boolean ping_wait_timer();
		static void timer_setup();
		static void timer_ms_cntdwn();
#endif
#if ULTRAPING_DO_BITWISE == true
		uint8_t _triggerBit;
		uint8_t _echoBit;
		volatile uint8_t *_triggerOutput;
		volatile uint8_t *_echoInput;
		volatile uint8_t *_triggerMode;
#else
		uint8_t _triggerPin;
		uint8_t _echoPin;
#endif
		unsigned int _maxEchoTime;
		unsigned long _max_time;
};


#endif
