#ifndef AT_COM_H
#define AT_COM_H

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

// Custom headers
#include "core.h"
#include "xbeeCom.h"

// Prototypes
int SendRemoteATCommand(int bus, uint16_t destAddr, uint16_t cmd);
int SendLocalATCommand(int bus, uint16_t cmd);
int StripATResponseData(unsigned char* msg, int length, unsigned char* buf, int bufSize);
int ReadRemoteATResponseData(int bus, unsigned char* buf, int bufSize);
int ReadLocalATResponseData(int bus, unsigned char* buf, int bufSize);

// Send a Remote AT Command to given local destination address
// @param bus: UART bus address for the Beagle Bone Blue
// @param destAddr: 16-bit destination address(MY) of destination XBee
// @param cmd: 16-bit command to send to destination XBee
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

	// Write 16-bit destination param (Stored post order LINUX)
	msg[13] = ((unsigned char*)&destAddr)[1];
    msg[14] = ((unsigned char*)&destAddr)[0];

    // Write 16-bit command param (Stored post order LINUX)
	msg[16] = ((unsigned char*)&cmd)[1];
    msg[17] = ((unsigned char*)&cmd)[0];

	// Calculate and Write checksum with patern
	msg[18] = CalculateChecksum(msg,19);

    // Send Command
    return SendCommand(bus, msg, 19);
}

// Send a Local AT Command over UART
// @param bus: UART bus address for the Beagle Bone Blue
// @param cmd: 16-bit command to send to destination XBee
// @return Number of sent bytes or -1 for failure
int SendLocalATCommand(int bus, uint16_t cmd){
    // Create framework for Local AT command
    // Header - [Command] - [Checksum]
    static unsigned char msg[8] = {
        0x7E, 0x00,0x04, 0x08, 0x01, 0x00,0x00, 0x00
    };

    // Write 16-bit command param (Stored post order LINUX)
	msg[5] = ((unsigned char*)&cmd)[1];
    msg[6] = ((unsigned char*)&cmd)[0];

	// Calculate and Write checksum with patern
	msg[7] = CalculateChecksum(msg,8);

    // Send Command
    return SendCommand(bus, msg, 8);
}

// Strip Data from a AT Command Response
// @param msg: unsigend char buffer of recived bytes from AT Command Response
// @param msgLength: length of AT command bytes in 'msg' buffer 
// @param buf: unsigned char buffer to store strped data in
// @param bufSize: size of the given buffer 'buf'
// @return Number of data bytes recived or -1 for failure
int StripATResponseData(unsigned char* msg, int msgLength, unsigned char* buf, int bufSize){
    // Get AT Command Frame type
    unsigned char frameType = msg[3];
    ASSERT((frameType == 0x97) || (frameType == 0x88), 
        "\tERROR: 0x%02X is not valid AT Command Response frame type.\n",frameType
    );

    // Pull Response Data out of the frame
    int start_byte = (frameType == 0x97) ? 18 : 8;
    ASSERT(start_byte+1 < msgLength, "\tERROR: Message contains incorectley formated frame.\n");
    
    int num_bytes = msgLength - (start_byte + 1);
    ASSERT(num_bytes <= bufSize, "\tERROR: Returned data is bigger than the given buffer.\n");

    memcpy(buf, msg+start_byte, num_bytes);
    return num_bytes;
}

// Read Data from a AT Command Response over UART
// @param bus: UART bus address for the Beagle Bone Blue
// @param buf: unsigned char buffer to store recived data in
// @param bufSize: size of the given buffer
// @return Number of data bytes recived or -1 for failure
int ReadRemoteATResponseData(int bus, unsigned char* buf, int bufSize){
    // Setup variables
    int length = 0;
    uint8_t msg[20];

    // Read in Command Response
    length = ReadCommand(bus, msg, 20);
    if(length < 0) return length;

    return StripATResponseData(msg, length, buf, bufSize);
}

// Read Data from a AT Command Response over UART
// @param bus: UART bus address for the Beagle Bone Blue
// @param buf: unsigned char buffer to store recived data in
// @param bufSize: size of the given buffer
// @return Number of data bytes recived or -1 for failure
int ReadLocalATResponseData(int bus, unsigned char* buf, int bufSize){
    // Setup variables
    int length = 0;
    uint8_t msg[10];

    // Read in Command Response
    length = ReadCommand(bus, msg, 10);
    if(length < 0) return length;

    return StripATResponseData(msg, length, buf, bufSize);
}

#endif