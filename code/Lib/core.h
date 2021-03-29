// Library of general usefull core featrues and functionality
#ifndef CORE_LIB_H
#define CORE_LIB_H

/// ----- Headers----- ///

// C Library headers
#include <stdio.h>
#include <unistd.h>

// Robot Control Library headers
#include <rc/start_stop.h>

/// ----- Defenitions----- ///

// Define Assert statement
#define ASSERT(condition,failTxt...) if(!(condition)){ printf(failTxt); return -1;}
#define ASSERT_NOMSG(condition) if(!(condition)){return -1;}

// Define Main Assert statement
#ifndef CORE_MAIN_ASSERT_LAMBDA
	#define CORE_MAIN_ASSERT_LAMBDA
#endif
#define MAIN_ASSERT(condition,failTxt...) if(!(condition)){ printf(failTxt); CORE_MAIN_ASSERT_LAMBDA; return -1;}
#undef CORE_MAIN_ASSERT_LAMBDA

// Array size statement
#define ARRSIZE(x) (sizeof(x)/sizeof(x[0]))

/// ----- Type Definitions ----- ///

typedef unsigned char ubyte;

/// ----- Prototypes ----- ///

void msleep(int milliSeconds);
void printHexBuffer(void* buf, int bufSize);
void fprintHexBuffer(void* buf, int bufSize, char* header, char* footer);

/// ----- Function Defenitions ----- ///

// Milliseconds Sleep
// @param milliSeconds: Time in milliseconds to sleep for
void msleep(int milliSeconds) { usleep((milliSeconds)*1000); }

// Print out byte array in hexidecimal
// @param buf: Pointer to array of bytes to print out
// @param bufSize: sice of given buffer 'buf' in bytes
void printHexBuffer(void* buf, int bufSize){
	for(int i=0; i<bufSize; i++)
		printf("%02X ", ((ubyte*)buf)[i]);
}

// Print out byte array in hexidecimal with endcaps
// @param buf: Pointer to array of bytes to print out
// @param bufSize: sice of given buffer 'buf' in bytes
// @param header: Formated string to print out before the buffer
// @param footer: Formated string to print out affter the buffer
void fprintHexBuffer(void* buf, int bufSize, char* header, char* footer){
	printf(header);
	for(int i=0; i<bufSize; i++)
		printf("%02X ", ((ubyte*)buf)[i]);
	printf(footer);
}

#endif