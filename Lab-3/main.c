// main.c
// Runs on MSP432
// Lab 3 - Compiler Optimization
// Draven Schilling
// 12/17/19
// Runs some VERY inefficient code to time the duration of the main loop.
// toggles a GPIO output every main loop cycle
// The goal is to see the effects of compiler optimization
// which will be shown as a shorter period on the GPIO out signal.
//  **************************************************

#include <stdio.h>
#include <stdlib.h>
#include "msp.h"

void init_gpio();
void nestedFunct(int times);
void innerNestedFunct(int times);
void innermostNestedFunct();
int dummyFunct(int length);

void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	init_gpio();

	while(1){
	    P5->OUT = P5->OUT^1; // toggle output (1/2 period will be program duration)
	    nestedFunct(10);
	}
}

// Setup P5.0 as GPIO output.
void init_gpio()
{
    // P5.0
    P5->SEL0 &= ~0b1; // use GPIO function
    P5->SEL1 &= ~0b1;
    P5->DIR |= 0b1;   // make output
    P5->OUT &= ~0b1;  // setup output low to start
    return;
}

/* create deep nested short functions
 *
 * the compiler should be able to optimize how these short functions are called
 * by expanding them.
 */
void nestedFunct(int times){
    int i = 0;
    for(i = 0; i < times; i++)
        innerNestedFunct(times);
}

void innerNestedFunct(int times){
    int i = 0;
    for(i = 0; i < times; i++)
        innermostNestedFunct(times);
}

void innermostNestedFunct(int times){
    int i = 0;
    for(i = 0; i < times; i++){
        int dummy = dummyFunct(times);
    }
}

// Highly inefficient calculation method
int dummyFunct(int length){
    int i = 0;
    /*
     * - The compiler should be able to simplify the number of steps in the calculation.\
     * - Also I used many multiples of 2 such that bit shifting would be more optimal than multiplication
     * - I used longs even though the range of data is much smaller. to see if it may optimize the allocation type
     * - I created a lot of dummy variables which should be optimized out.
     * - I also went through many nested if statements which should be optimized to a single if.
     */
    long* dummy_ptr = malloc(sizeof(long)*length); // using longs unnecessarily
    for(i = 0; i < length; i++){ // ++i is generally more efficient
        dummy_ptr[i] = (((i + 4)*96/40)%10)+(i>0?dummy_ptr[i-1]:64); // inefficient arithmetic statement
        int j = 4, k = 9, l = 0, m = 7, n = 3, o = 1, p = 5; // repeatedly create unused variables

        // Some compounded if statements (will always go through all given 5 < length < 11
        if(length > 2)
            if(length > 5)
                if(length < 12)
                    if(length < 11)
                        dummy_ptr[i] = dummy_ptr[i] % 13;
    }

    int ret = (length = 0 ? 0 : (int)dummy_ptr[length-1]);
    free(dummy_ptr);
    return ret;
}
