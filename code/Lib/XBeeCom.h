#ifndef XEBB_COM_H
#define XBEE_COM_H

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// Robot Control Library headers
#include <rc/uart.h>

// Cutom lIbrary headers
#include "CoreLib.h"

// Prototypes
int XBee_InitUART(int bus);
int XBee_CloseUART(int bus);
int SendCommand(int bus, unsigned char* msg, int length);
int ReadCommand(int bus, unsigned char* buf, int bufSize);
unsigned char CalculateChecksum(unsigned char* msg, int length);
bool CheckChecksum(unsigned char* msg, int length);
unsigned char GetFrameState(unsigned char* msg, int length);

// Initalize UART port to XBee
// @peram bus: UART bus address for the Beagle Bone Blu
// @return 0 for success or -1 for failure
int XBee_InitUART(int bus){
        // int rc_uart_init(int bus, int baudrate, float timeout, int canonical_en, int stop_bits, int parity_en);
        return rc_uart_init(bus, 9600, 0.1f, 0, 1, 0);
}

// Close UART port to XBee
// @peram bus: UART bus address for the Beagle Bone Blu
// @return 0 for success or -1 for failure
int XBee_CloseUART(int bus){
        // int rc_uart_init(int bus, int baudrate, float timeout, int canonical_en, int stop_bits, int parity_en);
        return  rc_uart_close(bus);
}

// Send a given Command using UART connection
// @peram bus: UART bus address for the Beagle Bone Blue
// @peram msg: pointer to buffer containg command message
// @peram length: length of message in the given buffer
// @return number of sent bytes or -1 for failure
int SendCommand(int bus, unsigned char* msg, int length){
        // Send Command
        // int rc_uart_write(int bus, uint8_t* data, size_t bytes);
        int success = rc_uart_write(bus, msg , length);
	if(success == -1) return -1;

#if DEBUG_XBEECOM
	// Print out Command that was sent in hex
        fprintHexBuffer(msg, length, "\t[DEBUG] CMD: ", "\n");
	//printf("\t[DEBUG] CMD: ");
	//for(uint16_t i=0; i < length; i++)
	//	printf("0x%02X ",msg[i]);
        //printf("\n");
#endif

        return success;
}

// Read in Command response into given buffer
// @peram bus: UART bus address for the Beagle Bone Blue
// @peram buf: pointer to unsigned char buffer to hold input
// @peram bufSize: Size of buffer given to "buf"
// @return recived message length or -1 for failure
int ReadCommand(int bus,  unsigned char* buf, int bufSize){
         // Read in response
         // int rc_uart_read_bytes(int bus, uint8_t* buf, size_t bytes);
        int num_Bytes = rc_uart_read_bytes(bus, buf, bufSize);
	if(num_Bytes == -1) { return -1; }

         // Check that the response is not empty
        if(num_Bytes == 0){
#if DEBUG_XBEECOM
                printf("\t[DEBUG] Did not recive a response message.\n");
#endif
                return -1;
        }

#if DEBUG_XBEECOM
	printf("\t[DEBUG] Recived: %d Bytes\n",num_Bytes);
        fprintHexBuffer(buf, num_Bytes, "\t[DEBUG] MSG: ", "\n");
	//printf("\t[DEBUG] MSG: ");
	//for(int i=0; i < num_Bytes; i++)
	//	printf("0x%02X ",buf[i]);
	//printf("\n");
#endif

        // Check for valid checksum
        ASSERT(CheckChecksum(buf,num_Bytes), "\tInvalid Checksum :(\n");
#if DEBUG_XBEECOM
        printf("\t[DEBUG] Valid Checksum!\n");
#endif

        // Check if Error flag set in response
        unsigned char frameState = GetFrameState(buf,num_Bytes);
        ASSERT(frameState == 0, "\tCommand Errored out :(\n");
#if DEBUG_XBEECOM
        printf("\t[DEBUG] Command Ran Sucesfuly!\n");
#endif

        return num_Bytes;
}

// Calculate AT Message checksum
// @peram msg: Pointer to buffer containing AT Message
// @peram length: length of AT Meassage in "msg" buffer
// @return Checksum byte
unsigned char CalculateChecksum(unsigned char* msg, int length) {
        unsigned int sum = 0;
        for(int i=3; i <= length-2; i++)
                sum += msg[i];
        return (0xFF - (sum & 0xFF));
}

// Calculate AT Message checksum and compare it
// @peram msg: Pointer to buffer containing AT Message
// @peram length: length of AT Meassage in "msg" buffer
// @return bool for if vaild checksum
bool CheckChecksum(unsigned char* msg, int length){
        return (CalculateChecksum(msg,length) == msg[length-1]);
}

// Extract Return AT Message state from AT Return message
// @peram msg: Pointer to buffer containing AT Message
// @peram length: length of AT Meassage in "msg" buffer
// @return Frame State or 255 for failure
unsigned char GetFrameState(unsigned char* msg, int length){
        unsigned char frameType = msg[3];        // Get Frame Type
        if(frameType == 0x97 && length > 17) { return msg[17]; }
        else if(frameType == 0x88 && length > 7) { return msg[7]; }
        else {
                printf("\tInvalied frame type 0x%02X or formating encountered.\n",frameType);
                return 255;
        }
}

#endif
