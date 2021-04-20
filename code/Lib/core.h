// Library of general usefull core featrues and functionality
#ifndef CORE_LIB_H
#define CORE_LIB_H

/// ----- Headers----- ///

// C Library headers
#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>

// Robot Control Library headers
#include <rc/start_stop.h>

/// ----- Definitions----- ///

// Define Assert statement
#define ASSERT_NOMSG(condition) if(!(condition)){return -1;}
#ifndef CORE_DISABLE_MESSAGES
    #define ASSERT(condition,failTxt...) if(!(condition)){ printf(failTxt); return -1;}
#else
    #define ASSERT(condition,failTxt...) ASSERT_NOMSG(condition)
#endif

// Define Main Assert statement
#ifndef CORE_MAIN_ASSERT_LAMBDA
    #define CORE_MAIN_ASSERT_LAMBDA
#endif
#define MAIN_ASSERT(condition,failTxt...) if(!(condition)){ printf(failTxt); rc_set_state(EXITING);;}
#undef CORE_MAIN_ASSERT_LAMBDA

// Array size statement
#define ARRSIZE(x) (sizeof(x)/sizeof(x[0]))

/// ----- Type Definitions ----- ///

typedef unsigned char ubyte;

/// ----- Prototypes ----- ///

inline static void msleep(int milliSeconds);
inline static void core_printf( const char* format, ...);
void printHexBuffer(void* buf, int bufSize);
void fprintHexBuffer(void* buf, int bufSize, char* header, char* footer);
//void core_Print();

/// ----- Function Defenitions ----- ///

// Milliseconds Sleep
// @param milliSeconds: Time in milliseconds to sleep for
inline static void msleep(int milliSeconds) { usleep((milliSeconds)*1000); }

// Core Formated Print
inline static void core_printf( const char* format, ...) {
#ifndef CORE_DISABLE_MESSAGES
    va_list argptr;
    va_start(argptr, format);
    printf(format, argptr); 
    va_end(argptr);
#endif
return;
}

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