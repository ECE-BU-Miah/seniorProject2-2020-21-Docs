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
#include "core.h"
#include "atCom.h"

#if XBEE_ARRAY_DEBUG
    #define XBEE_ARRAY_DEBUG_ASSERT(condition,failTxt...) ASSERT(condition,failTxt)
#else
    #define XBEE_ARRAY_DEBUG_ASSERT(condition,failTxt...) ASSERT_NOMSG(condition)
#endif

typedef struct {
    int top_uart_bus, side_uart_bus;
    int io_chip_0, io_pin_0;
    int io_chip_1, io_pin_1;
    uint16_t remoteAddr;
} xbeeArray_settings;

int xbeeArray_Init(xbeeArray_settings* array);
int xbeeArray_Close(xbeeArray_settings* array);
int xbeeArray_GetStrengths(xbeeArray_settings* array, ubyte strengths[5]);

int xbeeArray_Init(xbeeArray_settings* array) {
    int result;

    // Initalize UART buses
    result = xbeeCom_InitUART(array->top_uart_bus);
    XBEE_ARRAY_DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to initalize top UART on bus %d\n", array->top_uart_bus);
    xbeeCom_InitUART(array->side_uart_bus);
    XBEE_ARRAY_DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to initalize side UART on bus %d\n", array->side_uart_bus);

    // Initalize GPIO pins
    result = rc_gpio_init(array->io_chip_0, array->io_pin_0, GPIOHANDLE_REQUEST_OUTPUT);
    XBEE_ARRAY_DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to initalize output gpio %d_%d.\n", array->io_chip_0, array->io_pin_0);
    result = rc_gpio_init(array->io_chip_1, array->io_pin_1, GPIOHANDLE_REQUEST_OUTPUT);
    XBEE_ARRAY_DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to initalize output gpio %d_%d.\n", array->io_chip_1, array->io_pin_1);

    return 0;
}

int xbeeArray_Close(xbeeArray_settings* array) {
    int result;

    // Close UART buses
    result = xbeeCom_CloseUART(array->top_uart_bus);
    XBEE_ARRAY_DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to close top UART on bus %d\n", array->top_uart_bus);
    result = xbeeCom_CloseUART(array->side_uart_bus);
    XBEE_ARRAY_DEBUG_ASSERT(result == 0, "\t[DEBUG] ERROR: Failed to close side UART on bus %d\n", array->side_uart_bus);

    return 0;
}

int xbeeArray_GetStrengths(xbeeArray_settings* array, ubyte strengths[5]){
    int result;
    int readAttempts = 0;

    readAttempts = 0;
    do {
        readAttempts++;
        
        // Send Remote AT Command from the top XBee
        result = atCom_SendRemote(array->top_uart_bus, array->remoteAddr, 0x6462);
        XBEE_ARRAY_DEBUG_ASSERT(result != -1, "\t[DEBUG] ERROR: Failed to write Remote AT command to UART Port %d.\n", array->top_uart_bus);

        // Read in signal strength for the remote from the top XBee
        result = atCom_ReadRemoteResponseData(array->top_uart_bus, &(strengths[0]), 1);

        if(result < 0)
        {
            usleep(1000*10);
            XBEE_ARRAY_DEBUG_ASSERT(rc_uart_flush(array->top_uart_bus) == 0, "\t[DEBUG] failed to flush UART for bus %d.\n", array->top_uart_bus);
        }
    }  while(result < 0 && readAttempts < 4);
    XBEE_ARRAY_DEBUG_ASSERT(result >= 0, "\t[DEBUG] ERROR: Failed to read Remote AT command response data on UART Port %d.\n", array->top_uart_bus);

    // Flush side UART in preparation for local AT commands
    usleep(1000*1);
    result = rc_uart_flush(array->side_uart_bus);
    XBEE_ARRAY_DEBUG_ASSERT(result == 0, "\t[DEBUG] failed to flush UART for bus %d.\n", array->side_uart_bus);

    // Check strength of received message for the side XBees
    for(int i=0; i<4; i++) {
        // Set Multiplexer chanel to 'i'
        result = rc_gpio_set_value(array->io_chip_0, array->io_pin_0, (i)&1);
        XBEE_ARRAY_DEBUG_ASSERT(result != -1, "\t[DEBUG] ERROR: Failed to set pin value for GPIO %d_%d.\n", array->io_chip_0, array->io_pin_0);
        result = rc_gpio_set_value(array->io_chip_1, array->io_pin_1, (i>>1)&1);
        XBEE_ARRAY_DEBUG_ASSERT(result != -1, "\t[DEBUG] ERROR: Failed to set pin value for GPIO %d_%d.\n", array->io_chip_1, array->io_pin_1);      
        
        readAttempts = 0;
        do {
            readAttempts++;

            // Send Local AT Command to side XBee 'i'
            result = atCom_SendLocal(array->side_uart_bus, 0x6462);
            XBEE_ARRAY_DEBUG_ASSERT(result != -1, "\t[DEBUG] ERROR: Failed to write Local AT command to UART Port %d.\n", array->side_uart_bus);        

            // Read in signal strength from side XBee 'i'
            result = atCom_ReadLocalResponseData(array->side_uart_bus, &(strengths[i+1]), 1);

            if(result < 0)
            {
                usleep(1000*1);
                XBEE_ARRAY_DEBUG_ASSERT(rc_uart_flush(array->side_uart_bus) == 0, "\t[DEBUG] failed to flush UART for bus %d.\n", array->side_uart_bus);
            }
        } while(result < 0 && readAttempts < 4);
        XBEE_ARRAY_DEBUG_ASSERT(result >= 0, "\t[DEBUG] ERROR: Failed to read Local AT command response data on UART Port %d.\n", array->side_uart_bus);
    }

    return 0;
}

#endif