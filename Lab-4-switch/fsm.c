/**
 * fsm.c
 */

#ifndef FSM_C_
#define FSM_C_

#include "fsm.h"
#include "msp.h"

// turns the fan and compressor off
void outputsOff(){
    P2->OUT &= ~((1<<4)|(1<<6));
}

//turns the fan and compressor on
void outputsOn(){
    P2->OUT |= ((1<<4)|(1<<6));
}

//turns the fan on and compressor off (defrost mode)
void defrost(){
    P2->OUT &= ~(1<<4);
    P2->OUT |= (1<<6);
}

#endif

