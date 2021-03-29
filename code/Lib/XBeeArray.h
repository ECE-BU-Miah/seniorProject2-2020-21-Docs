#ifndef XBEE_ARRAY_H
#define XBEE_ARRAY_H

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

// Robot Control Library headers
#include <rc/gpio.h>

// Custom headers
#include "CoreLib.h"
#include "ATCom.h"

#if XBEE_ARRAY_DEBUG
    #undef DEBUG_ASSERT
    #define DEBUG_ASSERT(condition,ftext...) ASSERT(condition,ftext)
#endif

typedef struct {
    int top_uart_bus, side_uart_bus;
    int io_chip_0, io_pin_0;
    int io_chip_1, io_pin_1;
    uint16_t remoteAddr;
} XBeeArray_Settings;

int XBeeArray_Init(XBeeArray_Settings* array);
int XBeeArray_Close(XBeeArray_Settings* array);
int XBeeArray_GetStrengths(XBeeArray_Settings* array, unsigned char strengths[5]);

int XBeeArray_Init(XBeeArray_Settings* array) {
    int result;

    // Initalize UART busses
    result = XBee_InitUART(array->top_uart_bus);
    DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to initalize top UART on bus %d\n", array->top_uart_bus);
    XBee_InitUART(array->side_uart_bus);
    DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to initalize side UART on bus %d\n", array->side_uart_bus);

    // Initalize GPIO pins
    result = rc_gpio_init(array->io_chip_0, array->io_pin_0, GPIOHANDLE_REQUEST_OUTPUT);
    DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to initalize output gpio %d_%d.\n", array->io_chip_0, array->io_pin_0);
    result = rc_gpio_init(array->io_chip_1, array->io_pin_1, GPIOHANDLE_REQUEST_OUTPUT);
    DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to initalize output gpio %d_%d.\n", array->io_chip_1, array->io_pin_1);

    return 0;
}

int XBeeArray_Close(XBeeArray_Settings* array) {
    int result;

    // Close UART busses
    result = XBee_CloseUART(array->top_uart_bus);
    DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to close top UART on bus %d\n", array->top_uart_bus);
    result = XBee_CloseUART(array->side_uart_bus);
    DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to close side UART on bus %d\n", array->side_uart_bus);

    return 0;
}

int XBeeArray_GetStrengths(XBeeArray_Settings* array, unsigned char strengths[5]){
    int result;
    int readAttempts = 0;

    readAttempts = 0;
    do {
        readAttempts++;
        
        // Send Remote AT Command from the top XBee
        result = SendRemoteATCommand(array->top_uart_bus, array->remoteAddr, 0x6462);
        DEBUG_ASSERT(result != -1, "\t[DEBUG] ERROR: Failed to write Remote AT command to UART Port %d.\n", array->top_uart_bus);

        // Read in Signel Stength for the remote from the top XBee
        result = ReadRemoteATResponseData(array->top_uart_bus, &(strengths[0]), 1);

        if(result < 0)
        {
            msleep(10);
            DEBUG_ASSERT(rc_uart_flush(array->top_uart_bus) == 0, "\t[DEBUG] failed to flush UART for bus %d.\n", array->top_uart_bus);
        }
    }  while(result < 0 && readAttempts < 4);
    DEBUG_ASSERT(result >= 0, "\t[DEBUG] ERROR: Failed to read Remote AT command response data on UART Port %d.\n", array->top_uart_bus);

    // Flush Side UART in preperation for local AT commands
    msleep(1);
    result = rc_uart_flush(array->side_uart_bus);
    DEBUG_ASSERT(result == 0, "\t[DEBUG] failed to flush UART for bus %d.\n", array->side_uart_bus);

    // Check Strength of recived message for the side XBees
    for(int i=0; i<4; i++) {
        // Set Multiplexer chanel to 'i'
        result = rc_gpio_set_value(array->io_chip_0, array->io_pin_0, (i)&1);
        DEBUG_ASSERT(result != -1, "\t[DEBUG] ERROR: Failed to set pin value for GPIO %d_%d.\n", array->io_chip_0, array->io_pin_0);
        result = rc_gpio_set_value(array->io_chip_1, array->io_pin_1, (i>>1)&1);
        DEBUG_ASSERT(result != -1, "\t[DEBUG] ERROR: Failed to set pin value for GPIO %d_%d.\n", array->io_chip_1, array->io_pin_1);      
        
        readAttempts = 0;
        do {
            readAttempts++;

            // Send Local AT Command to side XBee 'i'
            result = SendLocalATCommand(array->side_uart_bus, 0x6462);
            DEBUG_ASSERT(result != -1, "\t[DEBUG] ERROR: Failed to write Local AT command to UART Port %d.\n", array->side_uart_bus);        

            // Read in signal strength from side XBee 'i'
            result = ReadLocalATResponseData(array->side_uart_bus, &(strengths[i+1]), 1);

            if(result < 0)
            {
                msleep(1);
                DEBUG_ASSERT(rc_uart_flush(array->side_uart_bus) == 0, "\t[DEBUG] failed to flush UART for bus %d.\n", array->side_uart_bus);
            }
        } while(result < 0 && readAttempts < 4);
        DEBUG_ASSERT(result >= 0, "\t[DEBUG] ERROR: Failed to read Local AT command response data on UART Port %d.\n", array->side_uart_bus);
    }

    return 0;
}

#if XBEE_ARRAY_DEBUG
    #undef DEBUG_ASSERT
    #define DEBUG_ASSERT(condition,ftext...) ASSERT_NOMSG(condition)
#endif

#endif