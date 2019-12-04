//*****************************************************************************
//
// MSP432 blink_basic.c
//
// K.Widder
// 11/30/19 - modified to use MSOE library delays
// 12/3/19  - corrected Delay_3MHz_sec()
//
//****************************************************************************

#include "msp.h"
#include "msoe_lib_delay.c"

#define DELAYVAL 2  // 2 second delay

int main(void)
{

    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    // initialize gpio for RGB LED on P2.0 - 2
    /* GPIO Setup */
    P2->SEL0 &= ~(BIT0|BIT1|BIT2);  // select GPIO function on pins
    P2->SEL1 &= ~(BIT0|BIT1|BIT2);
    P2->DIR |= BIT0|BIT1|BIT2;      // make pins outputs
    P2->OUT &=  ~(BIT0|BIT1|BIT2);

    while(1)
    {
        Delay_3MHz_sec(DELAYVAL);  // delay
        P2->OUT ^=  BIT0;  // toggle red
        Delay_3MHz_sec(DELAYVAL);
        P2->OUT ^=  BIT1;  // toggle green
        Delay_3MHz_sec(DELAYVAL);
        P2->OUT ^=  BIT2;  // toggle blue
    }
}
