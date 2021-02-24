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
#include "CoreLib.h"
#define DEBUG_XBEECOM 0
#include "ATCom.h"

int main(){
	printf("\tStarting Gated UART Test...\n");

	// Initalize state variables
	int bus_1 = 1;
	int bus_2 = 2;
	uint16_t remoteAddr = 0x1111;

	// Initalize storage variables
	int success;
	uint8_t buff[255];
	
	// Initalize UART ports to XBees
	success = XBee_InitUART(bus_1);
	ASSERT(success != -1, "\tERROR: Failed to Initalize UART Port %d.\n",bus_1);
	
	success = XBee_InitUART(bus_2);
	ASSERT(success != -1, "\tERROR: Failed to Initalize UART Port %d.\n",bus_2);

	// Send Remote DB command to XBee Cordinator 1
	printf("\tSending Remote AT Command...\n");
	success = SendRemoteATCommand(bus_1, remoteAddr, 0x6462);
	if(success == -1) {  printf("\tERROR: Failed to write command to UART Port %d.\n",bus_1); return -1; }
	printf("\tBytes Sent: %d\n", success);

	msleep(100);

	// Read back remote command from XBee Cordinaor 1
	success = ReadCommand(bus_1, buff, 256);
	if(success == -1) {  printf("\tERROR: Failed to read frame from UART Port %d.\n",bus_1); return -1; }

	printf("\tBytes Recived to bus %d: %d", bus_1, success);
	if(success > 0) fprintHexBuffer(buff, success, "\t\n", "\n");

	// Read back remote command from XBee Cordinaor 2
	success = ReadCommand(bus_2, buff, 256);
	if(success == -1) {  printf("\tERROR: Failed to read frame from UART Port %d.\n",bus_2); return -1; }

	printf("\tBytes Recived to bus %d: %d", bus_2, success);
	if(success > 0) fprintHexBuffer(buff, success, "\t\n", "\n");

	// Send Local AT Command to XBee Cordinator 2
	printf("\tSending Local AT Command...\n");
	success = SendLocalATCommand(bus_2, 0x6462);
	if(success == -1) {  printf("\tERROR: Failed to write command to UART Port %d.\n",bus_2); return -1; }
	printf("\tBytes Sent: %d\n", success);

	msleep(100);

	// Read back local command from XBee Cordinaor 2
	success = ReadCommand(bus_2, buff, 256);
	if(success == -1) {  printf("\tERROR: Failed to read frame from UART Port %d.\n",bus_2); return -1; }

	printf("\tBytes Recived to bus %d: %d", bus_2, success);
	if(success > 0) fprintHexBuffer(buff, success, "\t\n", "\n");

	// Close UART ports to XBees
	success = XBee_CloseUART(bus_1);
	if(success == -1) {  printf("\tERROR: Failed to Close UART Port %d.\n",bus_1); return -1; }

	success = XBee_CloseUART(bus_2);
	if(success == -1) {  printf("\tERROR: Failed to Close UART Port %d.\n",bus_2); return -1; }

	return 0;
}
