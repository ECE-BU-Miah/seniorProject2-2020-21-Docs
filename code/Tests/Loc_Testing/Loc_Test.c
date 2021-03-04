/* Multiplexer Array Test 
*    
*/

// C Library headers
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

// Robot Control Library headers
#include <rc/uart.h>
#include <rc/gpio.h>
#include <rc/start_stop.h>

// Custom headers
#define DEBUG_XBEECOM 0
#define XBEE_ARRAY_DEBUG 1
#include "CoreLib.h"
#include "ATCom.h"
#include "XBeeArray.h"
#include "step.h"

#define STEPS_PER_MEASUREMENT 5
#define DEG_PER_MEASUREMENT (int)(STEPS_PER_MEASUREMENT*DEG_PER_STEP)
#define NUM_MEASUREMENTS 90/DEG_PER_MEASUREMENT

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        rc_set_state(EXITING);
        return;
}

// Local functions
int getMeasurement(struct XBeeArray_Settings array, unsigned int current, unsigned char* strengths, unsigned char* distance_strengths, unsigned char* directional_strengths);
double getDistance(unsigned char* distance_strengths, int num_strengths);
int getAngle(unsigned char* directional_strengths, int num_strengths);

int main(){
    printf("\tStarting Location Test...\n");

    // set signal handler so the loop can exit cleanly
    signal(SIGINT, __signal_handler);

    // make sure another instance isn't running
    // if return value is -3 then a background process is running with
    // higher privaledges and we couldn't kill it, in which case we should
    // not continue or there may be hardware conflicts. If it returned -4
    // then there was an invalid argument that needs to be fixed.
    if(rc_kill_existing_process(2.0)<-2) return -1;

    // make PID file to indicate your project is running
    // due to the check made on the call to rc_kill_existing_process() above
    // we can be fairly confident there is no PID file already and we can
    // make our own safely.
    rc_make_pid_file();

    // Initalize state variables
    int position = 0; // Angular position of reflector array (degrees)
    int direction = 1; // Direction of stepper motor
    unsigned int current = 0; // Current measurement number
    struct XBeeArray_Settings array = {
        5,1,   // Uart buses Top(5) and Side(1) 
        3,1,  // GPIO 0 (Chip 3 Pin 1)
        3,2,  // GPIO 1 (Chip 3 Pin 2)
        0x1111 // Target Remote XBee's 16-bit address
    };

    // Initalize storage variables
    int result;
    unsigned char strengths[5]; // Temporary storage during measurement
    unsigned char distance_strengths[NUM_MEASUREMENTS]; // Strengths from transmitter
    unsigned char directional_strengths[4*NUM_MEASUREMENTS]; // Strengths from side XBees
    double distance; // Distance to remote in m
    int angle; // Angle to remote in degrees

    // Initalize XBee Reflector Array
    printf("\tInitializing XBee Reflector Array...\n");
    result = XBeeArray_Init(array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to Initialize XBee Array.\n");

    // Initialize Stepper Motor
    StepperMotor sm;
    printf("\tInitializing Stepper Motor...\n");
    MAIN_ASSERT(stepper_init(&sm, 3, 4) != -1, "\tERROR: Failed to initialize Stepper Motor.\n");

    // Keep looping until state changes to EXITING
    rc_set_state(RUNNING);
    // Get the initial measurement
    result = getMeasurement(array, current, strengths, distance_strengths, directional_strengths);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    while(rc_get_state() != EXITING)
    {
        // Move the stepper motor
        step(&sm, direction, STEPS_PER_MEASUREMENT);

        // Update current measurement number
        current += direction;

        // Update position and direction
        position += direction*DEG_PER_MEASUREMENT;

        // Check if a full sweep has been completed
        if ((position >= (90 - DEG_PER_MEASUREMENT)) || (position <= 0))
        {
            // Calculate the distance to the remote
            distance = getDistance(distance_strengths, sizeof(distance_strengths));
            printf("Distance to remote: %.4f\n", distance);
            angle = getAngle(directional_strengths, sizeof(directional_strengths));
            printf("Angle to remote: %d\n", angle);
            // Change direction
            direction *= -1;
        }

        // Get Strength Values from the XBee Reflector Array
        result = getMeasurement(array, current, strengths, distance_strengths, directional_strengths);
        MAIN_ASSERT(result == 0, "\tERROR: Failed to get measurement.\n");
    }

    // Close XBee Reflector Array
    printf("\tClosing XBee Reflector Array...\n");
    result = XBeeArray_Close(array);
    MAIN_ASSERT(result == 0, "\tERROR: Failed to close XBee Array.\n");

    // Close the stepper motor
    printf("\tClosing Stepper Motor...\n");
    MAIN_ASSERT(stepper_cleanup() != -1, "\tERROR: Failed to close Stepper Motor\n");

    rc_remove_pid_file();    // remove pid file LAST

    printf("\tExiting...\n");
    return result;
}

int getMeasurement(struct XBeeArray_Settings array, unsigned int current, unsigned char* strengths, unsigned char* distance_strengths, unsigned char* directional_strengths)
{
    // Get Strength Values from the XBee Reflector Array
    int result = XBeeArray_GetStrengths(array, strengths);
    ASSERT(result == 0, "\tERROR: Failed to get signal strength values from the XBee Array.\n");

    // Store the strengths in the proper locations
    distance_strengths[current] = strengths[0]; // Transmitter XBee
    for (int i = 1; i <= 4; ++i) // Side XBees
    {
        directional_strengths[current + (i - 1)*10] = strengths[i];
    }

    return result;
}

double getDistance(unsigned char* distance_strengths, int num_strengths)
{
    // Find the average path loss in dBm
    int sum = 0;
    for (int i = 0; i < num_strengths; ++i)
    {
        sum += distance_strengths[i];
    }
    double avg_loss = (double)sum/num_strengths;

    // Calculate the distance in meters
    double distance = 2.9979*pow(10, avg_loss/20)/(4*M_PI*24);
    return distance;
}

int getAngle(unsigned char* directional_strengths, int num_strengths)
{
    // Calculate a value based on the surrounding strengths as well as the current one
    double val = 0.5*directional_strengths[num_strengths-1] + directional_strengths[0] + 0.5*directional_strengths[1];
    // Find the minimum value
    double min = val;
    int min_idx = 0;
    for (int i = 1; i < num_strengths-1; ++i)
    {
        val = 0.5*directional_strengths[i-1] + directional_strengths[i] + 0.5*directional_strengths[i + 1];
        if (val < min)
        {
            min = val;
            min_idx = i;
        }
    }

    // Convert the index to an angle in degrees
    int angle = min_idx*9;
    return angle;
}