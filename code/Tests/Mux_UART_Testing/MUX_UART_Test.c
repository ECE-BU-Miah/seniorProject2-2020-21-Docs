/* Multiplexer Array Test 
*    
*/

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

// Robot Control Library headers
#include <rc/uart.h>
#include <rc/gpio.h>

// Custom headers
#define DEBUG_XBEECOM 0
#define XBEE_ARRAY_DEBUG 1
#include "core.h"
#include "atCom.h"
#include "xbeeArray.h"

int main(){
	printf("\tStarting Multiplexer XBee Array Test...\n");

	// Initalize state variables
	xbeeArray_settings array = {
		5,1,   // Uart buses Top(1) and Side(5) 
		3,1,  // GPIO 0 (Chip 1 Pin 25)
		3,2,  // GPIO 1 (Chip 1 Pin 17)
		0x1111 // Target Remote XBee's 16-bit address
	};

	// Initalize storage variables
	int result;
	ubyte strengths[5];

	// Initalize XBee Reflector Array
	printf("\tInitalizeing XBee Reflector Array...\n");
	result = xbeeArray_Init(&array);
	ASSERT(result == 0, "\tERROR: Failed to Initalize XBee Array.\n");

	// Get Stenght Values from the XBee Reflector Array
	printf("\tGetting Strength values from XBee Reflector Array...\n");
	result = xbeeArray_GetStrengths(&array, strengths);
	ASSERT(result == 0, "\tERROR: Failed to Get Signal Strength values from the XBee Array.\n");
	printf("\tTop Strength: %02X\n", strengths[0]);
	fprintHexBuffer(strengths+1, 4, "\tSide Strengths: ", "\n");

	// Close XBee Reflector Array
	printf("\tCloseing XBee Reflector Array...\n");
	result = xbeeArray_Close(&array);
	ASSERT(result == 0, "\tERROR: Failed to Close XBee Array.\n");

	return result;
}
