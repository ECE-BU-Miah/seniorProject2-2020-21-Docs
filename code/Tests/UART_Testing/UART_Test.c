/* UART Test 
*    Test of Remote Control Library UART header
*    by takinga given UART bus and atempting
*    to open it then send a predefined message out
*    and then read a message coming into the port
*/

// C Library headers
#include <stdio.h>
#include <string.h>

// Linix Library headers
#include <unistd.h>

// Robot Controll Library headers
#include <rc/uart.h>

// Custom Library headers
#include "core.h"

int main(int argc, char* argv[]) {
	// Initalize Storage Variables
	int success;       // Success/Respose from external functions
	uint8_t buff[255]; // Recived Message buffer

	// Initalize Property Variables
	uint8_t msg[8] = { 0x7E,0x00,0x04,0x08,0x01,0x64,0x62,0x30 };
	//uint8_t msg[13] = {0x7E,0x00,0x09,0x01,0x01,0x00,0x03,0x00,0x74,0x65,0x73,0x74,0x3A};
	//uint8_t msg[19] = { 0x7E, 0x00, 0x0f, 0x17, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11, 0x02, 0x64, 0x62, 0xFD };
	int bus = 1;

	// Help menue argument
	if (argc > 1 && !strcmp(argv[1], "-help")) {
		printf("\t--- [Help] ---\n");
		printf("\t> Set Bus: -b [0-9]\n");
		return 0;
	}

	// Check for set bus argument
	if (argc > 1) {
		ASSERT(!strcmp(argv[1], "-b"), "\tERROR: Invalid argument \"%s\" type help for list of valid arguments.\n", argv[1]);
		ASSERT(argc > 2, "\tERROR: Bus Select must be in the format of \"-b [0-9]\"\n");
		char inpChar = argv[2][0];
		ASSERT((inpChar >= '0' && inpChar <= '9'), "\tERROR: Invalid Input for -b since \"%s\" is not a number between 0-9.\n", argv[2])
			bus = inpChar - '0';
	}

	printf("\tStarting UART Test on bus %d...\n", bus);

	// int rc_uart_init(int bus, int baudrate, float timeout, int canonical_en, int stop_bits, int parity_en);
	success = rc_uart_init(bus, 115200, 0.1f, 0, 1, 0);
	ASSERT(success != -1, "\tERROR: Failed to Initalize UART Port %d.\n", bus)

		// int rc_uart_write(int bus, uint8_t* data, size_t bytes);
		success = rc_uart_write(bus, msg, 8);
	ASSERT(success != -1, "\tERROR: Failed to write to  UART Port %d.\n", bus)

		printf("\tBytes Sent: %d\n", success);

	// int rc_uart_read_bytes(int bus, uint8_t* buf, size_t bytes);
	success = rc_uart_read_bytes(bus, buff, 256);
	ASSERT(success != -1, "\tERROR: Failed to read from  UART Port %d.\n", bus)

		// Print out recived Bytes
		printf("\tBytes Recived: %d", success);
	if (success > 0) printf("\n\t");
	for (int i = 0; i < success; i++)
		printf("%X ", buff[i]);
	printf("\n");

	// Cleanup/Close UART port
	success = rc_uart_close(bus);
	ASSERT(success != -1, "\tERROR: Failed to Close UART Port %d.\n", bus)

		return 0;
}
