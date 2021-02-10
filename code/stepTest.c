/**
 * @file stepper_example.c
 */

#include <stdio.h>
#include <signal.h>
#include <rc/motor.h>
#include <rc/time.h>
#include <rc/start_stop.h>

// Number of degrees per step on the stepper motor
#define DEG_PER_STEP 1.8

// Motor ports used on BBBlue
#define M1 3
#define M2 4

void step(int direction, int numSteps);

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        rc_set_state(EXITING);
        return;
}

/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
    double position = 0;
    int direction = 1;
    int stepSize = 5;
    
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

    // set signal handler so the loop can exit cleanly
    signal(SIGINT, __signal_handler);

    // Keep looping until state changes to EXITING
    rc_set_state(RUNNING);
    rc_motor_init();
    while(rc_get_state()!=EXITING){
        // Step 90 degrees
        while (position < 90)
        {
            step(direction, stepSize);
            position += 1.8*stepSize;
        }
        rc_usleep(500000);
        
        // Reverse direction and reset position
        direction *= -1;
        position = 0;
    }

    // Close file descriptors
    rc_remove_pid_file();    // remove pid file LAST
    return 0;
}

void step(int direction, int numSteps)
{
    static int state = 0;

    for (int i = 0; i < numSteps; ++i)
    {
        // Handle directional control
        if(direction > 0) {
            state++;
        } else if (direction < 0) {
            state--;
        }

        // Handle state rollover
        if(state >= 4) state = 0;
        else if(state < 0) state = 3;

        // Winding states
        switch(state) {
            case 0:
            default:
                rc_motor_set(M1,1);
                rc_motor_set(M2,1);
                break;
            case 1:
                rc_motor_set(M1,-1);
                rc_motor_set(M2,1);
                break;
            case 2:
                rc_motor_set(M1,-1);
                rc_motor_set(M2,-1);
                break;
            case 3:
                rc_motor_set(M1,1);
                rc_motor_set(M2,-1);
                break;
        }

        // Sleep to allow motor to turn
        rc_usleep(12000);
    }
}