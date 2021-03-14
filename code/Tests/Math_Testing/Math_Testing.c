/* UART Test 
*    Test of Remote Control Library UART header
*    by takinga given UART bus and atempting
*    to open it then send a predefined message out
*    and then read a message coming into the port
*/

// C Library headers
#include <stdio.h>
#include <string.h>

// Linix Library headers
#include <unistd.h>

// Custom Library headers
//#define EXTRA_MATH_DISABLE_INLINE
#include "extraMath.h"

int main() {
	printf("\t[Modulo Function]\n");
	for(int i = -5; i <= 5; i++)
		printf("\t   %d mod_i 3 = %d\n", i, mod_i(i,3));
	for(double i = -3; i <= 3; i+=0.5)
		printf("\t   %.2f mod 2 = %.2f\n", i, mod(i,2));

	printf("\t[Sign Function]\n");
	for(int i = -5; i <= 5; i++)
		printf("\t   sign %d = %d\n", i, sign_i(i));
	for(double i = -3; i <= 3; i+=0.5)
		printf("\t   sign %.2f = %.2f\n", i, sign(i));

	printf("\t[Min Function]\n");
	for(int i = -5; i <= 5; i++)
		printf("\t   %d min_i 3 = %d\n", i, min_i(i,3));
	for(double i = -3; i <= 3; i+=0.5)
		printf("\t   %.2f min_i 1 = %.2f\n", i, min(i,1));
	
	printf("\t[Max Function]\n");
	for(int i = -5; i <= 5; i++)
		printf("\t   %d min_i 3 = %d\n", i, max_i(i,3));
	for(double i = -3; i <= 3; i+=0.5)
		printf("\t   %.2f min_i 1 = %.2f\n", i, max(i,1));

	printf("\t[Clamp Function]\n");
	for(int i = -5; i <= 5; i++)
		printf("\t   %d clamp [-2,3] = %d\n", i, clamp_i(i,-2,3));
	for(double i = -3; i <= 3; i+=0.5)
		printf("\t   %.2f clamp [-1.5,1] = %.2f\n", i, clamp(i,-1.5,1));

	return 0;
}
