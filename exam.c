//*****************************************************************************
//
// EE493_exam1_p2.c
// 1/22/2020
//
// Optimize the following code for speed.
// Reference the line numbers of the code you change in each section of your solution
// Include comments to indicate what you did and how it improves execution speed
//
// Modified By Draven Schilling 1-27-20
//
//****************************************************************************
#include "msp.h"

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    // setup port pin for scope timing
    P3->DIR |= BIT6;
    P3->OUT &= ~BIT6;

    // EDIT: Remove init_A2D method and instead directly insert the contents here:

    // Sampling time, S&H=96, ADC14 on, ACLK
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_3 |
                   ADC14_CTL0_CONSEQ_2 | ADC14_CTL0_ON | ADC14_CTL0_MSC; // EDIT: set all ADC14->CTL0 configure bits in one statement rather than 2 separate ones
    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_6;  // input on A6
    P4->SEL0 |= 0x80; // use A/D
    P4->SEL1 |= 0x80;
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;
	
    int arr[4], arr1[100], alpha[4], omega[4];
    // EDIT: remove unnecessary i, j, k variables
    int x, y, z, beta, gamma, sum = 0;

    while(1)
    {
        beta = ADC14->MEM[0];

        // EDIT: unroll the alpha setting loop
        alpha[0] = 0; // EDIT: eliminate unnecessary multiplication
        alpha[1] = beta; // EDIT: eliminate unnecessary multiplication
        alpha[2] = beta<<1; // EDIT: left shift by 1 instead of multiply by 2
        alpha[3] = beta * 3;

        gamma = beta*beta*beta*beta; // EDIT: eliminate function call for gamma calculation

        // EDIT: unroll arr loop. Delete unnecessary x variable
        // EDIT: eliminate function calls for power and replace with constant calculated value
        arr[0] = gamma;
        arr[1] = gamma + 2;
        arr[2] = gamma + 4;
        arr[3] = gamma + 8;

        // EDIT: unroll omega loop. simplify arr access to constant expression
        omega[0] = alpha[0] + arr[3];
        omega[1] = alpha[1] + arr[2];
        omega[2] = alpha[2] + arr[1];
        omega[3] = alpha[3] + arr[0];

        y = x * 31;
        z = ((y + 3)<<2) - (x + 12); // EDIT: left shift by 2 instead of multiply by 4

        x = y + z; // EDIT: take out x expression as it does not need to be calculated inside the loop
        int k = 33*x; // EDIT: replace expression within the following loop with arbitrary variable 'k' which only
                      //       performs the calculation once.
        // EDIT: ideally, you would want to completely unroll this loop, but to limit the space I use,
        //       I will instead only unroll it from 100 to 10 sections
        for(i = 0; i < 100; i+10)
        {
            arr1[i] = k + (y * i) -  omega[i%4];
            arr1[i+1] = k + (y * (i+1)) -  omega[(i+1)%4];
            arr1[i+2] = k + (y * (i+2)) -  omega[(i+2)%4];
            arr1[i+3] = k + (y * (i+3)) -  omega[(i+3)%4];
            arr1[i+4] = k + (y * (i+4)) -  omega[(i+4)%4];
            arr1[i+5] = k + (y * (i+5)) -  omega[(i+5)%4];
            arr1[i+6] = k + (y * (i+6)) -  omega[(i+6)%4];
            arr1[i+7] = k + (y * (i+7)) -  omega[(i+7)%4];
            arr1[i+8] = k + (y * (i+8)) -  omega[(i+8)%4];
            arr1[i+9] = k + (y * (i+9)) -  omega[(i+9)%4];
        }

        // EDIT: simplify logic such that sum is only being added to where i is multiples of 9 up until 100.
        //       so remove/unroll loop and take out conditional statement and instead just sum up arr1's at
        //       every 9th place.
        sum += (arr1[0] + arr1[9] + arr1[18] + arr1[27] + arr1[36] + arr1[45] + arr1[54] +
                arr1[63] + arr1[72] + arr1[81 + arr1[90] + arr1[99]);

        P3->OUT ^= BIT6;  // EDIT: toggle for timing measurement
    }
}

// EDIT: remove power method

// EDIT: remove init_A2D method