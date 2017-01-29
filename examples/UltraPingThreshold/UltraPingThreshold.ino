//Example ping_threshold

//You can change settings for UltraPing by defining parameters before include-statement.
#define ONE_PIN_ENABLED false //Default is true
#define LENGTH_UNIT_CM        //Default, but you can change to INCH
#include <UltraPing.h>


//Settings for this example:
#define BAUD 57600           // Make sure your terminal is set to same.
#define TRIGGER_PIN 12
#define ECHO_PIN 13          //Can be connected to same if ONE_PIN_ENABLED == true

//Declare the UltraPing-object, adjust your pins.
UltraPing up(TRIGGER_PIN, ECHO_PIN);

void setup() {
	Serial.begin(BAUD);
	while(!Serial);
	Serial.println("UltraPing Example - Demonstrating ping_threshold");
	Serial.println("Only showing detected hits between threshold and max distance.");
}


void loop() {
	//ping_threshold(...) returns echo time for the first hit between threshold and max distance.
	//convert_length(...) converts it to the selected length unit. (cm or inch)
	int hit = up.convert_length(up.ping_threshold(50, 60));
	if (hit > 0) {
		Serial.println(hit);
	}
	delay(50);
}
