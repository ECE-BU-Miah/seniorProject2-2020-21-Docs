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

// Robot Control Headers
#include <rc/motor.h>
#include <rc/encoder_eqep.h>

// Custom Headers
#include "core.h"

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy) {
        rc_set_state(EXITING);
        return;
}

// Prototypes
int ProcessArguments(int argc, char* argv[],  int* _pwmFrequency, double* _duty);
int AddRecored(FILE* fp, int pwm, float duty, int steps1, int steps2 );
int StripFloat(char* str, float* value);
int StripInt(char* str, int* value);

// Main
int main(int argc, char* argv[]){
	printf("\tStarting Quadrature Value Recorder...\n");

	// set signal handler so the loop can exit cleanly
    signal(SIGINT, __signal_handler);

	// Set up PID file for this program
	if(rc_kill_existing_process(2.0)<-2) return -1;
    rc_make_pid_file();
	rc_set_state(RUNNING);

	// Initalize state variables
	int _pwmFrequency = RC_MOTOR_DEFAULT_PWM_FREQ;
	int _motor1 = 1, _motor2 = 2;
	int _encoder1 = 1, _encoder2 = 2;
	double _duty = 0;

	// Initalize storage variables
	int result;
	FILE* fp;

	// Read in peramiters
	result = ProcessArguments(argc, argv, &_pwmFrequency, &_duty);
	MAIN_ASSERT(result != -1, "\tERROR: Failed to Read in Arguments.\n");
	if(result == 1) return 1;

    // Initialize Motors
    printf("\tInitializing Motor...\n");
    MAIN_ASSERT(rc_motor_init_freq(_pwmFrequency) != -1, "\tERROR: Failed to initialize Motors.\n");

	// Initalize Encoders
	printf("\tInitalizeing Quadrature Encoders...\n");
	MAIN_ASSERT(rc_encoder_eqep_init() == 0, "\tERROR: Failed to Initialize  Quadrature Encoders.\n");

	// Open output file in Append mode
	printf("\tOpening file to write out to...\n");
	fp = fopen("Quadratur_PWM.csv", "a");
	MAIN_ASSERT(fp != NULL, "\tERROR: Failed to open file.\n");
	
	// Lock steper motors beforehand for safty
	MAIN_ASSERT(rc_motor_set(3, 1) != -1, "\tERROR: Failed to Set Motor 1.\n");
	MAIN_ASSERT(rc_motor_set(4, 1) != -1, "\tERROR: Failed to Set Motor 1.\n");

	// Main Program loop
	const int _numSteps = 10;
	int steps1[_numSteps];
	int steps2[_numSteps];
	printf("\tSrarting record...\n");
	while(rc_get_state() != EXITING) {
		// Set Motor Duty Cycles
		MAIN_ASSERT(rc_motor_set(_motor1, _duty) != -1, "\tERROR: Failed to Set Motor 1.\n");
		MAIN_ASSERT(rc_motor_set(_motor2, _duty) != -1, "\tERROR: Failed to Set Motor 1.\n");

		// Run through moment sequence
		for(int i = 0; i < _numSteps; i++) {
			// Clear Encoder steps
			MAIN_ASSERT(rc_encoder_eqep_write(_encoder1, 0) != -1, "\tERROR: Failed to write to encoder 1.\n");
			MAIN_ASSERT(rc_encoder_eqep_write(_encoder2, 0) != -1, "\tERROR: Failed to write to encoder 2.\n");

			// Wait for 1 decisecond for robot to move
			msleep(500);

			// Read in step counds from 1 decisecond of movement
			steps1[i] = rc_encoder_eqep_read(_encoder1);
			MAIN_ASSERT(steps1[i] != -1, "\tERROR: Failed to read from encoder 1.\n");
			steps2[i] = rc_encoder_eqep_read(_encoder2);
			MAIN_ASSERT(steps2[i] != -1, "\tERROR: Failed to read from encoder 2.\n");

			if(rc_get_state() == EXITING) break;
		}
		if(rc_get_state() == EXITING) break;

		// Write out recorded steps
		for(int i = 0; i < _numSteps; i++) {
			result = AddRecored(fp, _pwmFrequency, _duty, steps1[i], steps2[i]);
			MAIN_ASSERT(result != -1, "\tERROR: Failed to add record.\n");
			if(rc_get_state() == EXITING) break;
		}

		rc_set_state(EXITING);
	}

	// Safty delay for system to stabalize
	MAIN_ASSERT(rc_motor_set(_motor1, 0) != -1, "\tERROR: Failed to Set Motor 1.\n");
	MAIN_ASSERT(rc_motor_set(_motor2, 0) != -1, "\tERROR: Failed to Set Motor 1.\n");
	msleep(1000);

	// Close output file
	result = fclose(fp);
	MAIN_ASSERT(result == 0, "\tERROR: Faield to close file.\n");

	// Close Quadrature Encoders
	printf("\tCloseing XBee Reflector Array...\n");
	result = rc_encoder_eqep_cleanup();
	MAIN_ASSERT(result == 0, "\tERROR: Failed to Close Quadrature Encoders.\n");

	// Close the motors
    printf("\tClosing Stepper Motor...\n");
    MAIN_ASSERT(rc_motor_cleanup() != -1, "\tERROR: Failed to close Stepper Motor\n");

    rc_remove_pid_file();    // remove pid file LAST

	return result;
}

int AddRecored(FILE* fp, int pwm, float duty, int steps1, int steps2) {
	// Initalize storage variables
	int result;
	
	// Write out header if the file is empty
	if(ftell(fp) == 0) {
		result = fprintf(fp, "pwm,duty,steps1,steps2\n");
		ASSERT(result >= 0, "\tERROR: Failed to write to the file\n");
	}

    // Write out the ange, distance, and height values to the file
    result = fprintf(fp, "%d,%1.1f,%d,%d\n",pwm,(double)duty,steps1,steps2);
    ASSERT(result >= 0, "\tERROR: Failed to write to the file\n");

	return 0;
}

int ProcessArguments(int argc, char* argv[],  int* _pwmFrequency, double* _duty) {
	// Check for Help menue argument
	if(argc > 1 && !strcmp(argv[1],"-help")){
		printf("\t--- [ Help ] ---\n");
		printf("\t> Set PWM Frequency{Hz} : -f [int]\n");
		printf("\t>        Set duty Cycle : -d [float]\n");
		return 1;
	}
	
	// Check for value set operators
	int result = 0;
	int argId = 1;
	while(argId < argc) {
		// Check if current main argumetn is in valid format "-[Char]"
		char* mainArg = argv[argId++];
		bool mainFailed = !((mainArg[0] == '-') && (mainArg[2] == '\0'));
		if(!mainFailed) {
			switch (mainArg[1]) {
			case 'f': // PWM Frequency Argument
				if(argId < argc) {
					int pwm;
					result = StripInt(argv[argId++], &pwm);
				 	if(result > 0) {
					 	*_pwmFrequency = pwm;
						break;
					}
				}
				printf("\tERROR: -f(Set PWM Frequency{Hz}) should be in format of \"-f [int]\".\n");
                return -1;
			case 'd': // Duty Cycle Argument
				if(argId < argc) {
					float duty;
					result = StripFloat(argv[argId++], &duty);
				 	if(result > 0) {
					 	*_duty = duty;
						break;
					}
				}
				printf("\tERROR: -d(Set duty Cycle) should be in format of \"-d [float]\".\n");
                return -1;
			default: // Invalid argument
				mainFailed = true; break;
			}
		}
		ASSERT(!mainFailed, "ERROR: \"%s\" is not a valid argument. Use \"%s -help\" for list of valid arguments.\n", mainArg, argv[0]);
	}

	return 0;
}

int StripFloat(char* str, float* value) {
	int len;
	int result = sscanf(str, "%f %n", value, &len);
	if(result==1 && !str[len]){
		return len;
	}
	return -1;
}

int StripInt(char* str, int* value) {
	int len;
	int result = sscanf(str, "%d %n", value, &len);
	if(result==1 && !str[len]){
		return len;
	}
	return -1;
}
