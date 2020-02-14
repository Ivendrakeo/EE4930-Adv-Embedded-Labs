/**
 * main.c
 */

#include "fsm.h"

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_delay.h"

void initGPIOInputs();

void initGPIOOutputs();

void initADC();

#define ADC_RANGE 4096 // for 12 bit conversions

//Global Variables
uint8_t temperature = 0; //temperature reading
uint8_t humidity = 0; // humidity reading
uint8_t humidity_setpoint = 50; // set point (default 50)

void main(void)
{

    event input;
    state current = NORMAL_OFF; // start in off mode
    outputsOff(); // start with everything off

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    // setup lcd
    LCD_Config();
    LCD_clear();
    LCD_home();
    LCD_contrast(10);

    // init ports
    initADC();
    initGPIOInputs();
    initGPIOOutputs();

    while(1){
        ADC14->CTL0 |= 1; // start ADC conversions
        LCD_goto_xy(0,0);
        LCD_print_str("Temp: ");
        LCD_print_udec5(temperature);
        LCD_goto_xy(0,1);
        LCD_print_str("Set: ");
        LCD_print_udec5(humidity_setpoint);
        LCD_goto_xy(0,2);
        LCD_print_str("Humid: ");
        LCD_print_udec5(humidity);
        LCD_goto_xy(0,3);
        if((P1->IN & 1<<5) >> 5 == 1){ // check for high signal from the ice sensor
            LCD_print_str("DEFROST: Y ");
        } else {
            LCD_print_str("DEFROST: N ");
        }

        // determine input event
        input = IDLE;
        if(humidity >= (humidity_setpoint + 5)){
            input = TURN_ON;
        } else if(humidity <= (humidity_setpoint - 5)){
            input = TURN_OFF;
        }
        if((P1->IN & 1<<5) >> 5 == 1){ // check for high signal from the ice sensor
            input = THERE_IS_ICE;
        }

        switch(input){
        case IDLE:
            switch(current){
            case NORMAL_OFF:
                outputsOff();
                current = NORMAL_OFF;
                break;
            case NORMAL_ON:
                outputsOn();
                current = NORMAL_ON;
                break;
            case DEFROST_MODE:
                outputsOff();
                current = DEFROST_MODE;
                break;
            }
            break;
        case TURN_ON:
            switch(current){
            case NORMAL_OFF:
                outputsOn();
                current = NORMAL_ON;
                break;
            case NORMAL_ON:
                outputsOn();
                current = NORMAL_ON;
                break;
            case DEFROST_MODE:
                outputsOn();
                current = NORMAL_ON;
                break;
            }
            break;
        case TURN_OFF:
            switch(current){
            case NORMAL_OFF:
                outputsOff();
                current = NORMAL_OFF;
                break;
            case NORMAL_ON:
                outputsOff();
                current = NORMAL_OFF;
                break;
            case DEFROST_MODE:
                outputsOff();
                current = NORMAL_OFF;
                break;
            }
            break;
        case THERE_IS_ICE:
            switch(current){
            case NORMAL_OFF:
                defrost();
                current = DEFROST_MODE;
                break;
            case NORMAL_ON:
                defrost();
                current = DEFROST_MODE;
                break;
            case DEFROST_MODE:
                defrost();
                current = DEFROST_MODE;
                break;
            }
            break;
        }
    }
}

// Setup both MSP onboard pushbuttons & Ice Sensor
// initializes pushbutton P1.1 as GPIO input
// initializes pushbutton P1.4 as GPIO input
// initializes P1.5 as GPIO input (Ice Sensor)
void initGPIOInputs()
{
    // P1.1
    P1->SEL0 &= ~0b10; // use GPIO function
    P1->SEL1 &= ~0b10;
    P1->DIR &= ~0b10;   // make input
    P1->REN |= 0b10; // allow pull up/down
    P1->OUT |= 0b10;  // setup as pull up
    P1->IES &= ~0b10; // trigger interrupt on rising edge
    P1->IE |= 0b10; // enable interrupt
    // P1.4
    P1->SEL0 &= ~(0b1<<4); // use GPIO function
    P1->SEL1 &= ~(0b1<<4);
    P1->DIR &= ~(0b1<<4);   // make input
    P1->REN |= (0b1<<4); // allow pull up/down
    P1->OUT |= (0b1<<4);  // setup as pull up
    P1->IES &= (0b1<<4); // trigger interrupt on rising edge
    P1->IE |= (0b1<<4); // enable interrupt
    // P1.5
    P1->SEL0 &= ~(0b1<<5); // use GPIO function
    P1->SEL1 &= ~(0b1<<5);
    P1->DIR &= ~(0b1<<5);   // make input
    P1->REN |= (0b1<<5); // allow pull up/down
    P1->OUT |= (0b1<<5);  // setup as pull up
    P1->IES &= (0b1<<5); // trigger interrupt on rising edge
    P1->IE |= (0b1<<5); // enable interrupt

    NVIC->ISER[1] |= (1<<3);  // enable I/O P1 interrupt in NVIC
}

// Initializes Outputs for both the fan and compressor
// initializes P2.4 as GPIO output (compressor)
// initializes P2.6 as GPIO output (fan)
void initGPIOOutputs(){
    // P2.4
    P2->SEL0 &= ~(1<<4); // use GPIO function
    P2->SEL1 &= ~(1<<4);
    P2->DIR |= (1<<4);   // make output
    P2->OUT &= ~(1<<4);  // setup output low to start
    // P2.6
    P2->SEL0 &= ~(1<<6); // use GPIO function
    P2->SEL1 &= ~(1<<6);
    P2->DIR |= (1<<6);   // make output
    P2->OUT &= ~(1<<6);  // setup output low to start
}

//Initialize ADC6 on pin 4.7
//Initialize ADC7 on pin 4.6
//Interrupt upon completed conversion
void initADC(){
    //setup pins 4.6 and 4.7 in analog mode
    P4->SEL0 |= 0b11<<6;
    P4->SEL1 |= 0b11<<6;
    // start sampling on SC bit,
    // source a timer,
    // use SMCLK,
    // repeat sequence of channels conversion mode
    // 96 sample/hold time
    // turn core on
    ADC14->CTL0 &= 0x0;
    ADC14->CTL0 |= (1<<26) |  (1<<21) | (0b11<<17) | (0b101<<12) | (0b101<<8) | (1<<4);
    // 12 bit resolution and use memory location 4 to start
    ADC14->CTL1 &= 0xF0000000;
    ADC14->CTL1 |= (0b10<<4) | (4<<16);
    ADC14->MCTL[4] |= 0x6;  // input on A6 for pot1
    ADC14->MCTL[5] |= 0x7;  // input on A7 for pot2
    ADC14->MCTL[5] |= (1<<7);  // set EOS for location5
    ADC14->IER0 |= (1<<4);  // enable interrupt for mem[4]
    ADC14->IER0 |= (1<<5);  // enable interrupt for mem[5]
    ADC14->CTL0 |= 0b10;  // enable
    NVIC->ISER[0] |= (1<<24);  // enable ADC interrupt in NVIC
    ADC14->CTL0 |= 1; // start ADC conversions
}

// Interrupt Handler for ADC
// Read ADC and set to temperature/humidity
void ADC14_IRQHandler(void)
  {
      uint32_t pendingInterrupt = ADC14->IFGR0;
      if((pendingInterrupt & 1<<4) > 0){
          //pot1
          temperature = 100-(uint8_t)((((float)ADC14->MEM[4])/ADC_RANGE)*100);
          ADC14->CLRIFGR0 |= 1<<4; // Clear Interrupt
      } else if((pendingInterrupt & 1<<5) > 0){
          //pot2
          humidity = 100-(uint8_t)((((float)ADC14->MEM[5])/ADC_RANGE)*100);
          ADC14->CLRIFGR0 |= 1<<5; // Clear Interrupt
      }
      ADC14->CLRIFGR0 |= 0xFFFFFFFF; // Extra precaution
  }

// Interrupt Handler for the GPIO inputs
// increment / decrement the set point
// interrupt for the ice sensor is unused
void PORT1_IRQHandler(void){
    uint16_t pending_interrupt = P1->IV;
    if(pending_interrupt == 4){
        humidity_setpoint = (humidity_setpoint < 5) ? 0 : humidity_setpoint-5;
    } else if(pending_interrupt == 0xA){
        humidity_setpoint = (humidity_setpoint > 95) ? 100 : humidity_setpoint+5;
    } else if(pending_interrupt == 0xC){
        // Unused interrupt
    }
}
