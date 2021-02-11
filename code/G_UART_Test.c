/* Gated UART Test 
*    Test of ATCom and XBeeCom Custom Libraries
*    And a Test of if all connected XBees are cordinators
*    will the response packet from a remote endpoint
*    be recived by all the corrdinators even if only
*    one connected cordinator sent the inital command to it
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
#define DEBUG_XBEECOM 1
#include "../Lib/ATCom.h"

#define ASSERT(condition,failTxt...) if(!(condition)){ printf(failTxt); return -1;}

void msleep(int milliSeconds) { usleep(milliSeconds*1000); }

int main(){
	printf("\tStarting Gated UART Test...\n");

	// Initalize state variables
	int bus_1 = 1;
	int bus_2 = 2;

	// Initalize storage variables
	int success;
	uint8_t buff[255];

	
	// Initalize UART ports to XBees
	success = XBee_InitUART(bus_1);
	ASSERT(success != -1, "\tERROR: Failed to Initalize UART Port %d.\n",bus_1);
	
	success = XBee_InitUART(bus_2);
	ASSERT(success != -1, "\tERROR: Failed to Initalize UART Port %d.\n",bus_2);

	// Send Remote DB command to XBee Cordinator 1
	success = SendRemoteATCommand(bus_1, 0x1111, 0x6462);
	if(success == -1) {  printf("\tERROR: Failed to write command to UART Port %d.\n",bus_1); return -1; }
	printf("\tBytes Sent: %d\n", success);

	sleep(1);

	// Read back command from XBee Cordinaor 1
	success = ReadCommand(bus_1, buff, 256);
	if(success == -1) {  printf("\tERROR: Failed to read frame from UART Port %d.\n",bus_1); return -1; }

	printf("\tBytes Recived: %d", success);
	if(success > 0) printf("\n\t");
	for(int i=0; i<success; i++)
		printf("%X ",buff[i]);
	printf("\n");

	// Read back command from XBee Cordinaor 2
	success = ReadCommand(bus_1, buff, 256);
	if(success == -1) {  printf("\tERROR: Failed to read frame from UART Port %d.\n",bus_1); return -1; }

	printf("\tBytes Recived: %d", success);
	if(success > 0) printf("\n\t");
	for(int i=0; i<success; i++)
		printf("%X ",buff[i]);
	printf("\n");


	// Close UART ports to XBees
	success = XBee_CloseUART(bus_1);
	if(success == -1) {  printf("\tERROR: Failed to Close UART Port %d.\n",bus_1); return -1; }

	success = XBee_CloseUART(bus_2);
	if(success == -1) {  printf("\tERROR: Failed to Close UART Port %d.\n",bus_2); return -1; }

	return 0;
}
