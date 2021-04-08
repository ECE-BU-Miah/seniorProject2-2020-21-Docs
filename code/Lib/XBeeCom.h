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
#include "core.h"

// Prototypes
int xbeeCom_InitUART(int bus);
int xbeeCom_CloseUART(int bus);
int xbeeCom_SendCommand(int bus, ubyte* msg, int length);
int xbeeCom_ReadCommand(int bus, ubyte* buf, int bufSize);
ubyte xbeeCom_CalculateChecksum(ubyte* msg, int length);
bool xbeeCom_CheckChecksum(ubyte* msg, int length);
ubyte xbeeCom_GetFrameState(ubyte* msg, int length);

// Initalize UART port to XBee
// @peram bus: UART bus address for the Beagle Bone Blu
// @return 0 for success or -1 for failure
int xbeeCom_InitUART(int bus){
        // int rc_uart_init(int bus, int baudrate, float timeout, int canonical_en, int stop_bits, int parity_en);
        return rc_uart_init(bus, 115200, 0.1f, 0, 1, 0);
}

// Close UART port to XBee
// @peram bus: UART bus address for the Beagle Bone Blu
// @return 0 for success or -1 for failure
int xbeeCom_CloseUART(int bus){
        // int rc_uart_close(int bus);
        return  rc_uart_close(bus);
}

// Send a given Command using UART connection
// @peram bus: UART bus address for the Beagle Bone Blue
// @peram msg: pointer to buffer containg command message
// @peram length: length of message in the given buffer
// @return number of sent bytes or -1 for failure
int xbeeCom_SendCommand(int bus, ubyte* msg, int length){
        // Send Command
        // int rc_uart_write(int bus, uint8_t* data, size_t bytes);
        int success = rc_uart_write(bus, msg , length);
        if(success == -1) return -1;

#if DEBUG_XBEECOM
        // Print out Command that was sent in hex
        fprintHexBuffer(msg, length, "\t[DEBUG] CMD: ", "\n");
#endif

        return success;
}

// Read in Command response into given buffer
// @peram bus: UART bus address for the Beagle Bone Blue
// @peram buf: pointer to ubyte buffer to hold input
// @peram bufSize: Size of buffer given to "buf"
// @return recived message length or -1 for failure
int xbeeCom_ReadCommand(int bus,  ubyte* buf, int bufSize){
         // Read in response
         // int rc_uart_read_bytes(int bus, uint8_t* buf, size_t bytes);
        int num_Bytes = rc_uart_read_bytes(bus, buf, bufSize);
        ASSERT(num_Bytes != -1, "\tRC Read Bytes Failed.\n");

         // Check that the response is not empty
        if(num_Bytes == 0){
#if DEBUG_XBEECOM
                printf("\t[DEBUG] Did not recive a response message.\n");
#endif
                return -3;
        }

#if DEBUG_XBEECOM
        printf("\t[DEBUG] Recived: %d Bytes\n",num_Bytes);
        fprintHexBuffer(buf, num_Bytes, "\t[DEBUG] MSG: ", "\n");
#endif

        // Check for valid checksum
        if(!xbeeCom_CheckChecksum(buf,num_Bytes)) {
                core_printf("\tInvalid Checksum :(\n");
                return -2;
        }
#if DEBUG_XBEECOM
        printf("\t[DEBUG] Valid Checksum!\n");
#endif

        // Check if Error flag set in response
        ubyte frameState = xbeeCom_GetFrameState(buf,num_Bytes);
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
ubyte xbeeCom_CalculateChecksum(ubyte* msg, int length) {
        unsigned int sum = 0;
        for(int i=3; i <= length-2; i++)
                sum += msg[i];
        return (0xFF - (sum & 0xFF));
}

// Calculate AT Message checksum and compare it
// @peram msg: Pointer to buffer containing AT Message
// @peram length: length of AT Meassage in "msg" buffer
// @return bool for if vaild checksum
bool xbeeCom_CheckChecksum(ubyte* msg, int length){
        return (xbeeCom_CalculateChecksum(msg,length) == msg[length-1]);
}

// Extract Return AT Message state from AT Return message
// @peram msg: Pointer to buffer containing AT Message
// @peram length: length of AT Meassage in "msg" buffer
// @return Frame State or 255 for failure
ubyte xbeeCom_GetFrameState(ubyte* msg, int length){
        ubyte frameType = msg[3];        // Get Frame Type
        if(frameType == 0x97 && length > 17) { return msg[17]; }
        else if(frameType == 0x88 && length > 7) { return msg[7]; }
        else {
                core_printf("\tInvalied frame type 0x%02X or formating encountered.\n",frameType);
                return 255;
        }
}

#endif
