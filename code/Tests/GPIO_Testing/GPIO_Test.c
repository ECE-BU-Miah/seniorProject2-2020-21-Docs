/* GPIO Test 
*    Test of Remote Control Library GPIO header
*    by takinga given GPIO port and atempting
*    to open it and then toggle the pins output
*    several times to confirm that its functional
*/

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

// Linix Library headers
#include <unistd.h>

// Robot Controll Library headers
#include <rc/gpio.h>

// Custom Library headers
#include "core.h"

int main(int argc, char* argv[]){
	printf("\tStarting GPIO Test...\n");

	// Initalize variables
	int success = 0;
	int chip = 3;
	int pin = 2;

	// Check for Help menue argument
	if(argc > 1 && !strcmp(argv[1],"-help")){
		printf("\t--- [ Help ] ---\n");
		printf("\t> Set Pin: -p [int]\n");
		printf("\t> Set Chip: -c [int]\n");
		return 0;
	}

	// Check for single character arguments
	int argId = 1;
	char* ptr;
	while(argId < argc) {
		// Check if current main argumetn is in valid format "-[Char]"
		char* mainArg = argv[argId++];
		bool mainFailed = !((mainArg[0] == '-') && (mainArg[2] == '\0'));
		if(!mainFailed) {
			switch (mainArg[1]) {
			case 'c': // Chip Select Argument
				if(argId < argc) {
					char* subArg = argv[argId++];
				 	if(subArg[0] >= '0' && subArg[0] <= '9') {
					 	chip = strtol(subArg, &ptr, 10);
						break;
					}
				}
				ASSERT(false,"ERROR: -c(Set Chip) should be in format of \"-c [int]\".\n")
			case 'p': // Pin Select argument
				if(argId < argc) {
					char* subArg = argv[argId++];
				 	if(subArg[0] >= '0' && subArg[0] <= '9') {
					 	pin = strtol(subArg, &ptr, 10);
						break;
					}
				}
				ASSERT(false,"ERROR: -p(Set Pin) should be in format of \"-p [int]\".\n")
			default: // Invalid argument
				mainFailed = true; break;
			}
		}
		ASSERT(!mainFailed, "ERROR: \"%s\" is not a valid argument. Use \"%s -help\" for list of valid arguments.\n", mainArg, argv[0]);
	}

	// Start Test of the GPIO Pin
	printf("\tRuning Test on GPIO pin %d for chip %d...\n", pin, chip);

	// Initalize GPIO pin
	success = rc_gpio_init(chip, pin, GPIOHANDLE_REQUEST_OUTPUT);
	ASSERT(success != -1, "\tERROR: Failed to initalize pin %d on chip %d.\n", pin, chip);


	// Toggle GPIO pin on and off
	int pinState = 1;
	for(int i = 0; i < 4; i++) {
		printf("\tGPIO pin [%s]\n", (pinState == 1) ? "ON" : "OFF");
		success = rc_gpio_set_value(chip, pin, pinState);
		ASSERT(success != -1, "\tERROR: Failed to set pin value.\n");
		sleep(1);
		pinState = pinState^1;
	}

	return 0;
}
