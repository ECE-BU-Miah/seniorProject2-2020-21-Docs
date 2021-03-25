// C Library headers
#include <stdio.h>
#include <signal.h>

// Robot Control Library headers
#include <rc/time.h>
#include <rc/start_stop.h>
#include <rc/button.h>

#include "extraMath.h"
#include "CoreLib.h"
#include "odometry.h"
#include "robot.h"

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        rc_set_state(EXITING);
        return;
}

// Function to print angle on pause press
static void __on_pause_press(void)
{
    double angle = odometry_getAngle();
    printf("Angle: %f degrees\n", angle);
    return;
}

// Function to reset encoders on mode press
static void __on_mode_press(void)
{
    odometry_setZeroRef();
    return;
}

int main()
{
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

    // Initialize pause button
    if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
    {
        fprintf(stderr,"ERROR: failed to init buttons\n");
        return -1;
    }
    // Set the callback function
    rc_button_set_callbacks(RC_BTN_PIN_PAUSE, __on_pause_press, NULL);

    // Initialize mode button
    if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US))
    {
        fprintf(stderr,"ERROR: failed to init buttons\n");
        return -1;
    }
    // Set the callback function
    rc_button_set_callbacks(RC_BTN_PIN_MODE, __on_mode_press, NULL);

    // Keep looping until state changes to EXITING
    rc_set_state(RUNNING);

    // Initalize Encoders
    printf("\tInitalizing Quadrature Encoders...\n");
    MAIN_ASSERT(rc_encoder_eqep_init() == 0, "\tERROR: Failed to Initialize  Quadrature Encoders.\n");

    odometry_setZeroRef();

    while(rc_get_state() != EXITING)
    {}

    // Close the encoders
    printf("\tClosing Encoders...\n");
    MAIN_ASSERT(rc_encoder_eqep_cleanup() != -1, "\tERROR: Failed to close Encoders\n");

    rc_remove_pid_file();    // remove pid file LAST
}