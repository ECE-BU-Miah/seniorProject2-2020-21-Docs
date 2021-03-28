/* Array_Recorder.c
*    
*/

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>

// Custom headers
#define DEBUG_XBEECOM 0
#define XBEE_ARRAY_DEBUG 1
#include "xbeeArray.h"
#include "step.h"

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy) {
        rc_set_state(EXITING);
        return;
}

// Prototypes
int StripFloat(char* str, float* value);

// Main
int main(){
    printf("\tStarting Angle Test...\n");

    // set signal handler so the loop can exit cleanly
    signal(SIGINT, __signal_handler);

    // Set up PID file for this program
    if(rc_kill_existing_process(2.0)<-2) return -1;
    rc_make_pid_file();

    // Initalize state variables
    struct XBeeArray_Settings array = {
        5,1,   // Uart buses Top(1) and Side(5) 
        3,1,  // GPIO 0 (Chip 1 Pin 25)
        3,2,  // GPIO 1 (Chip 1 Pin 17)
        0x1111 // Target Remote XBee's 16-bit address
    };

    // Initalize storage variables
    int result;
    unsigned char strengths[5];
    StepperMotor sm;

    // Initalize XBee Reflector Array
    printf("\tInitalizeing XBee Reflector Array...\n");
    result = XBeeArray_Init(array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to Initialize XBee Array.\n");

    // Initialize Stepper Motor
    printf("\tInitializing Stepper Motor...\n");
    MAIN_ASSERT(stepper_init(&sm, 3, 4) != -1, "\tERROR: Failed to initialize Stepper Motor.\n");
    
    // Main Program loop
    rc_set_state(RUNNING);
    while(rc_get_state() == RUNNING)
    {
        // Get Strength Values from the XBee Reflector Array
        result = XBeeArray_GetStrengths(array, strengths);
        ASSERT(result == 0, "\tERROR: Failed to Get Signal Strength values from the XBee Array.\n");

        for (int i = 1; i < 5; ++i)
        {
            printf("%d ", strengths[i]);
        }
        printf("\n");
        msleep(500);
    }
    
    // Close XBee Reflector Array
    printf("\tCloseing XBee Reflector Array...\n");
    result = XBeeArray_Close(array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to Close XBee Array.\n");

    // Close the stepper motor
    printf("\tClosing Stepper Motor...\n");
    MAIN_ASSERT(stepper_cleanup() != -1, "\tERROR: Failed to close Stepper Motor\n");

    rc_remove_pid_file();    // remove pid file LAST

    return result;
}
