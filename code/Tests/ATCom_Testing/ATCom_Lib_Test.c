/* AT Command Library Test 
*    Test of ATCom and some elements from other Custom Librarys.
*    First testing local AT commands and then
*    testing remote AT coommands as well as that
*    response data can be extracted from the AT command responses
*/

// C Library headers
#include <stdio.h>
#include <string.h>
//#include <stdbool.h>
#include <stdint.h>

// Custom headers
#include "core.h"
#define DEBUG_XBEECOM 0
#include "atCom.h"

int LocalATCom_Test(int bus);
int RemoteATCom_Test(int bus, uint16_t destAddr);

int main(){
	printf("\tStarting Gated UART Test...\n");

	// Initalize state variables
	int bus = 1;
	uint16_t remoteAddr = 0x0003;

	// Initalize storage variables
	int success;
	
	// Initalize UART ports to XBees
	success = xbeeCom_InitUART(bus);
	ASSERT(success != -1, "\tERROR: Failed to Initalize UART Port %d.\n", bus);

	// Test Local DB AT Command
	LocalATCom_Test(bus);

	// Test Remote DB AT Command
	RemoteATCom_Test(bus, remoteAddr);

	// Close UART ports to XBees
	success = xbeeCom_CloseUART(bus);
	ASSERT(success != -1, "\tERROR: Failed to Close UART Port %d.\n", bus);

	return 0;
}

int LocalATCom_Test(int bus) {
	// Initalize storage variables
	int success;
	uint8_t buff[255];
	uint8_t strength = 0;

	// Send local DB command to XBee Cordinator 1
	printf("\tSending Local AT Command...\n");
	success = atCom_SendLocal(bus, 0x6462);
	ASSERT(success != -1, "\tERROR: Failed to write command to UART Port %d.\n", bus);
	printf("\tBytes Sent: %d\n", success);

	msleep(100); // Safety Delay (Non-Esential)

	// Read back local command from XBee Cordinaor 1
	success = xbeeCom_ReadCommand(bus, buff, 256);
	ASSERT(success != -1, "\tERROR: Failed to read frame from UART Port %d.\n", bus);
	int msgLength = success;

	printf("\tBytes Recived to bus %d: %d\n", bus, msgLength);
	if(msgLength > 0) fprintHexBuffer(buff, msgLength, "\t", "\n");

	// Strip response Data (ex)DB -> Decible Strength)
	success = atCom_StripResponseData(buff, msgLength, &strength, 1);
	ASSERT(success != -1, "\tERROR: Failed to strip response data from given msg.\n");
	printf("\tResponse Data is 0x%02X\n", strength);

	return 0;
}

int RemoteATCom_Test(int bus, uint16_t destAddr) {
	// Initalize storage variables
	int success;
	uint8_t buff[255];
	uint8_t strength = 0;

	// Send remote DB command to XBee Cordinator 1
	printf("\tSending Remote AT Command...\n");
	success = atCom_SendRemote(bus, destAddr, 0x6462);
	ASSERT(success != -1, "\tERROR: Failed to write command to UART Port %d.\n", bus);
	printf("\tBytes Sent: %d\n", success);

	msleep(100); // Safety Delay (Non-Esential)

	// Read back remote command from XBee Cordinaor 1
	success = xbeeCom_ReadCommand(bus, buff, 256);
	ASSERT(success != -1, "\tERROR: Failed to read frame from UART Port %d.\n", bus);
	int msgLength = success;

	printf("\tBytes Recived to bus %d: %d\n", bus, msgLength);
	if(msgLength > 0) fprintHexBuffer(buff, msgLength, "\t", "\n");

	// Strip response Data (ex)DB -> Decible Strength)
	success = atCom_StripResponseData(buff, msgLength, &strength, 1);
	ASSERT(success != -1, "\tERROR: Failed to strip response data from given msg.\n");
	printf("\tResponse Data is 0x%02X\n", strength);

	return 0;
}
