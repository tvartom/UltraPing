//Example ping_multi with ascii-graph of detected echos in terminal.

//You can change settings for UltraPing by defining parameters before include-statement.
#define ONE_PIN_ENABLED true //Default is true
#define LENGTH_UNIT_CM //Default, but you can change to INCH
#include <UltraPing.h>


//Settings for this example:
#define MAX_DISTANCE 100     //In length unit, centimeter is default.
#define THRESHOLD_DISTANCE 0 //In length unit, hits shorter than this limit is not reported.
#define MAXIMUM_HITS 10      // Max number of hits to detect.
#define BAUD 57600           // Make sure your terminal is set to same.
#define TRIGGER_PIN 12
#define ECHO_PIN 12          //Can be connected to same if ONE_PIN_ENABLED == true

//Declare the UltraPing-object, adjust your pins.
UltraPing up(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

unsigned int hit[MAXIMUM_HITS];
char output[MAX_DISTANCE + 2];

void setup() {
	Serial.begin(BAUD);
	while(!Serial);
	Serial.println("UltraPing Example - Demonstrating ping_multi with an ASCII-Graph");
}


void loop() {
	//Clear the output String
	for(int i = 0; i <= MAX_DISTANCE; i++) output[i] = '_';
	output[MAX_DISTANCE+1] = 0;

	//Measure up to MAXIMUM_HITS number of echos, beyond the THRESHOLD_DISTANCE.
	//A measure takes longer time if first echo is close (Even if threshold is set
	//beyond.) (But the size of the window between threshold_distance and max_distance also matters)
	//The function returns the number of hits stored in the hit-array.
	int hits = up.ping_multi(hit, MAXIMUM_HITS, THRESHOLD_DISTANCE);

	//Replace the corresponding chars in output string with the hits.
	//Hits are in microsecond. Use convert_length to get in length unit. (cm or inch)
	//Uses min-function to make sure not writing outside of string.
	for(int i = 0; i < hits; i++) {
		output[min(MAX_DISTANCE,up.convert_length(hit[i]))] = '|';
	}

	//Print it out. A terminal with monospace font will best to represent the graph..
	Serial.println(output);
	//Don't trigger to often, echos can still be around.
	delay(50);
}

