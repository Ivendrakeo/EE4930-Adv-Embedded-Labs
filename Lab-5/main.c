/**
 * main.c
 */

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_delay.h"

#define ADC_RANGE 4096 // for 12 bit conversions

void initPCM();

void initADC();

void initWDA();

void initGPIOOutput();

//Global Variables
uint8_t temperature = 0; //temperature reading
uint8_t flip = 1; // boolean value for running the WDT_A interrupt every other time ~8min intervals

void main(void)
{

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    // setup lcd
    //LCD_Config();
    //LCD_clear();
    //LCD_home();
    //LCD_contrast(10);

    // init ports
    initGPIOOutput();
    initADC();
    initWDA();
    initPCM();

    while(1){
        //Empty main loop. Everything is interrupt controlled.
    }
}


void initWDA(){
    //Configure WDT_A
    WDT_A->CTL = 0x5A5D; // unlock WDT_A, vloclk src, interval mode, reset count, ~250ms initial timer
    NVIC->ISER[0] |= (1<<3);  // enable WDT_A interrupt in NVIC
}

void initPCM()
{
    // setup clock sources
    CS->KEY = 0x0000695A; //unlock
    CS->CLKEN |= 1<<8; // turn VLOCLK on;
    CS->CTL1 &= ~(0b1110111); // clear SM/HSMCLK, and MCLK sources
    CS->CTL1 |= 0b0010001; // set SMCLK/HSMCLK, MCLK to VLOCLK
    CS->KEY = 1; // lock

    // Configure for LPM3
    SCB->SCR  |= (1<<2); // Set Sleep Deep bit
    PCM->CTL0 = 0x695A0008; // unlock PCM/PMR, request LMP3, LP_AM_LDO_VCORE0
    SCB->SCR  |= (0b10); // Set Sleep On Exit
}

// Initializes Outputs for signaling a new temperature is completed
// initializes P2.4 for signaling
void initGPIOOutput(){
    // P2.4
    P2->SEL0 &= ~(1<<4); // use GPIO function
    P2->SEL1 &= ~(1<<4);
    P2->DIR |= (1<<4);   // make output
    P2->OUT &= ~(1<<4);  // setup output low to start
}

//Initialize ADC6 on pin 4.7
//Interrupt upon completed conversion
void initADC(){
    //setup pins 4.7 in analog mode
    P4->SEL0 |= 0b1<<7;
    P4->SEL1 |= 0b1<<7;
    // start sampling on SC bit,
    // source a timer,
    // use SMCLK,
    // 96 sample/hold time
    // turn core on
    ADC14->CTL0 &= 0x0;
    ADC14->CTL0 |= (1<<26) |  (1<<21) | (0b101<<12) | (0b101<<8) | (1<<4);
    // 12 bit resolution and use memory location 4 to start
    ADC14->CTL1 &= 0xF0000000;
    ADC14->CTL1 |= (0b10<<4) | (4<<16);
    ADC14->MCTL[4] |= 0x6;  // input on A6
}

// Interrupt Handler for the watchdog timer
void WDT_A_IRQHandler(void){
    if(flip){ // run the interrupt every other time it is triggered
        WDT_A->CTL = 0x5A5A; // unlock WDT_A, ~4min timer (up from 250ms)
        ADC14->CTL0 |= 0b10;  // enable ADC
        ADC14->CTL0 |= 1; // start ADC conversions
        while(((ADC14->CTL0 & (1<<16))>>16)); // wait for conversion to finish
        temperature = (uint8_t)((float)ADC14->MEM[4]*0.1498535-64);
        P2->OUT |= 1<<4; // signal temperature is ready
        //LCD_goto_xy(0,0);
        //LCD_print_str("Temp: ");
        //LCD_print_udec5(temperature);
        ADC14->CTL0 &= ~(0b10010); // disable and turn off ADC
        P2->OUT &= ~(1<<4);
        flip = 0;
    } else {
        flip = 1;
    }
}

