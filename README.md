# UltraPing #
## Arduino library for Ultrasonic sensors ##
* Possible to read multiple echos. (Eg see beyond first echo)
* Possible to use a threshold to only read distances further away
* A fork of Tim Eckel's New Ping.

## How does it work? ##
The ultrasonic sensor terminate its measuring on a ping after the first echo is recieved.
By starting a new ping, that will terminate not on its own echo, rather
on a second echo from the first ping, it is possible to see behind the first echo.
It require the first echo not to be too close to the sensor, and also result some loss in accuracy,
but it works.
To make it work the library makes several measuring rounds, gradually increasing the timing between
the pings. It takes som time for the sensor to restart, so echos that are too close to each other will
not be possible to detect.

```
#!arduino

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
```
