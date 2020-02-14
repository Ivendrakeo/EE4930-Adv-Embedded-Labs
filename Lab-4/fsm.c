/**
 * fsm.c
 */

#ifndef FSM_C_
#define FSM_C_

#include "fsm.h"
#include "msp.h"

// create state table array
stateElement stateTable [3][4] = {
  {{NORMAL_OFF, outputsOff},{NORMAL_ON, outputsOn},{NORMAL_OFF, outputsOff},{DEFROST_MODE, defrost}},
  {{NORMAL_ON, outputsOn},{NORMAL_ON, outputsOn},{NORMAL_OFF, outputsOff},{DEFROST_MODE, defrost}},
  {{DEFROST_MODE, outputsOff},{NORMAL_ON, outputsOn},{NORMAL_OFF, outputsOff},{DEFROST_MODE, defrost}},
};

// function to handle the state machine update
// accepts current state and current event, returns next state
state stateUpdate (state current, event input){
    stateElement currentstate = stateTable[current][input];
    // run the proper action function
    (*currentstate.action)();
    //return next state info
    return currentstate.nextstate;
}

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

