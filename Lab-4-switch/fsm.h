/*
 * fsm.h
 *
 *  Created on: Jan 13, 2020
 *      Author: schillingdl
 */

#ifndef FSM_H_
#define FSM_H_

typedef enum {
    NORMAL_OFF,
    NORMAL_ON,
    DEFROST_MODE
}state;

typedef enum {
    IDLE,
    TURN_ON,
    TURN_OFF,
    THERE_IS_ICE
}event;

// turns the fan and compressor off
void outputsOff();

//turns the fan and compressor on
void outputsOn();

//turns the fan on and compressor off (defrost mode)
void defrost();

// function to handle the state machine update
// accepts current state and current event, returns next state
state stateUpdate (state current, event input);




#endif /* FSM_H_ */
