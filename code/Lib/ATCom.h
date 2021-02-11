#ifndef AT_COM_H
#define AT_COM_H

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// Custom headers
#include "XBeeCom.h"

// Prototypes
int SendRemoteATCommand(int bus, uint16_t destAddr, uint16_t cmd);
int SendLocalATCommand(int bus, uint16_t cmd);
int ReadATResponseData(int bus, unsigned char* buf, int bufSize);

// Send a Remote AT Command to given local destination address
// @peram bus: UART bus address for the Beagle Bone Blue
// @peram destAddr: 16-bit destination address(MY) of destination XBee
// @peram cmd: 16-bit command to send to destination XBee
// @return Number of sent bytes or -1 for failure
int SendRemoteATCommand(int bus, uint16_t destAddr, uint16_t cmd){
	// Create framework for Remote AT command
    // Header - [Local Addr] - cmd_mode(2) - [Command] - [Checksum]
    static unsigned char msg[19] = {
        0x7E, 0x00,0x0F, 0x17, 0x01, 
        //0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
        0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
        0x00,0x00, 0x02, 0x00,0x00, 0x00 //0x64,0x62, 0x00
    };

	// Write 16-bit destination peram (Stored post order LINUX)
	msg[13] = ((unsigned char*)&destAddr)[1];
    msg[14] = ((unsigned char*)&destAddr)[0];

    // Write 16-bit command peram (Stored post order LINUX)
	msg[16] = ((unsigned char*)&cmd)[1];
    msg[17] = ((unsigned char*)&cmd)[0];

	// Calculate and Write checksum with patern
	msg[18] = CalculateChecksum(msg,19);

    // Send Command
    return SendCommand(bus, msg, 19);
}

// Send a Local AT Command over UART
// @peram bus: UART bus address for the Beagle Bone Blue
// @peram cmd: 16-bit command to send to destination XBee
// @return Number of sent bytes or -1 for failure
int SendLocalATCommand(int bus, uint16_t cmd){
    // Create framework for Local AT command
    // Header - [Command] - [Checksum]
    static unsigned char msg[9] = {
        0x7E, 0x00,0x04, 0x08, 0x01, 0x00,0x00, 0x00
    };

    // Write 16-bit command peram (Stored post order LINUX)
	msg[16] = ((unsigned char*)&cmd)[1];
    msg[17] = ((unsigned char*)&cmd)[0];

	// Calculate and Write checksum with patern
	msg[18] = CalculateChecksum(msg,19);

    // Send Command
    return SendCommand(bus, msg, 19);
}

// Read Data from a AT Command Response over UART
// @peram bus: UART bus address for the Beagle Bone Blue
// @peram buf: unsigned char buffer to store recived data in
// @peram bufSize: size of the given buffer
// @return Number of data bytes recived or -1 for failure
int ReadATResponseData(int bus, unsigned char* buf, int bufSize){
    // Setup variables
    int success = 0;
    uint8_t msg[255];

    // Read in Command Response
    success = ReadCommand(bus, msg, 256);
    if(success == -1) return -1;
    
    // Get AT Command Frame type
    unsigned char frameType = msg[3];
    if(!(frameType == 0x97 || frameType == 0x88)) {
        printf("\tERROR: 0x%02X is not valid AT Command Response frame type.\n",frameType);
        return -1;
    }

    // Pull Response Data out of the frame
    int start_byte = (frameType == 0x97) ? 1 : 1;
    int num_bytes = buf[start_byte-1];
    if(num_bytes > bufSize) {
        printf("\tERROR: Returned data is bigger than the given buffer.\n");
        return -1;
    }
    // TODO: use "memcpy()" data to given buffe
    //       based on num_byes from frame and
    //       frame specific start byte location
    //       (Need XBee to look at frame structure in XCTU)

    return num_bytes;
}

#endif