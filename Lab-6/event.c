#define __MSP432P401R__
#include "msp.h"

#include <stdio.h>
#include <string.h>

#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_delay.h"

/* XDC module Headers */
#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Timestamp.h>

/* BIOS module Headers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>

#include <ti/drivers/Board.h>

#define TIMEOUT         12      /* Timeout value */
#define CLKPERIOD       500      // number of clock ticks per adc trigger
                                   // period of ~0.5 sec

#define TASKSTACKSIZE   512
#define ADCINTERRUPT    40 // interrupt number for the ADC
#define ADC_RANGE 4096 // for 12 bit conversions

void initADC();

Void clkFxn(UArg arg0);
Void inputTask(UArg arg0, UArg arg1);
Void lcdTask(UArg arg0, UArg arg1);
Void swiFxn(UArg arg0, UArg arg1);
Void hwiFxn();


// create two tasks
Task_Struct inputTaskStruct, lcdTaskStruct;
Char inputTaskStack[TASKSTACKSIZE], lcdTaskStack[TASKSTACKSIZE];
// create clock
Clock_Struct clkStruct;
Clock_Handle clkHandle;
// create two events
Event_Struct readingAvailableEventStruct, setpointDisplayEventStruct;
Event_Handle readingAvailableEventHandle, setpointDisplayEventHandle;
// create SWI
Swi_Struct swiStruct;
Swi_Handle swiHandle;
// create HWI
Hwi_Struct hwiStruct;
Hwi_Handle hwiHandle;

uint8_t temperature = 0; //global temperature variable


int main()
{

    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    // setup lcd
    LCD_Config();
    LCD_clear();
    LCD_home();
    LCD_contrast(10);

    // init functions
    Hwi_enable();
    initADC();

    /* Construct BIOS Objects */
    Task_Params taskParams;
    Clock_Params clkParams;
    Swi_Params swiParams;
    Hwi_Params hwiParams;

    /* Call driver init functions */
    Board_init();

    /* Construct Task threads */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.priority = 3; // update internal temperature task is less important than interrupt methods
                             // but more important than lcd task
    taskParams.stack = &inputTaskStack;
    Task_construct(&inputTaskStruct, (Task_FuncPtr)inputTask, &taskParams, NULL);

    taskParams.priority = 4; // lcd update is least important
    taskParams.stack = &lcdTaskStack;
    Task_construct(&lcdTaskStruct, (Task_FuncPtr)lcdTask, &taskParams, NULL);

    /* Obtain event handlers */
    Event_construct(&readingAvailableEventStruct, NULL);
    readingAvailableEventHandle = Event_handle(&readingAvailableEventStruct);

    Event_construct(&setpointDisplayEventStruct, NULL);
    setpointDisplayEventHandle = Event_handle(&setpointDisplayEventStruct);

    /* setup clock */
    Clock_Params_init(&clkParams);
    clkParams.startFlag = TRUE;
    Clock_construct(&clkStruct, (Clock_FuncPtr)clkFxn,
                    CLKPERIOD*2, &clkParams);
    clkHandle = Clock_handle(&clkStruct);
    Clock_setPeriod(clkHandle, CLKPERIOD);

    /* setup SWI */
    Swi_Params_init(&swiParams);
    swiParams.arg0 = 1;
    swiParams.arg1 = 0;
    swiParams.priority = 2; // priority after HWI
    swiParams.trigger = 0;

    Swi_construct(&swiStruct, (Swi_FuncPtr)swiFxn, &swiParams, NULL);
    swiHandle = Swi_handle(&swiStruct);

    /* setup HWI */
    Hwi_Params_init(&hwiParams);
    hwiParams.eventId = 40;
    hwiParams.priority = 1; // top priority
    Hwi_construct(&hwiStruct, 40, hwiFxn, &hwiParams, NULL);
    hwiHandle = Hwi_handle(&hwiStruct);

    /* start the BIOS */
    BIOS_start();
    return(0);
}

//Initialize ADC6 on pin 4.7
//Interrupt upon completed conversion
void initADC(){
    //setup pins 4.6 and 4.7 in analog mode
    P4->SEL0 |= 0b1<<7;
    P4->SEL1 |= 0b1<<7;
    // start sampling on SC bit, source a timer, use SMCLK, single-channel single conversion mode
    // 96 sample/hold time, turn core on
    ADC14->CTL0 &= 0x0;
    ADC14->CTL0 |= (1<<26) |  (1<<21) | (0b101<<12) | (0b101<<8) | (1<<4);
    // 12 bit resolution and use memory location 4 to start
    ADC14->CTL1 &= 0xF0000000;
    ADC14->CTL1 |= (0b10<<4) | (4<<16);
    ADC14->MCTL[4] |= 0x6;  // input on A6
    ADC14->IER0 |= (1<<4);  // enable interrupt for mem[4]
    ADC14->CTL0 |= 0b10;  // enable

}

/*
 * Clock function triggers an ADC conversion every ~0.5 sec
 */
Void clkFxn(UArg arg0)
{
    ADC14->CTL0 |= 1; // start ADC conversions
}


/*
 * This task updates the LCD with the stored temperature value
 * whenever an event is triggered
 */
Void lcdTask(UArg arg0, UArg arg1)
{
    UInt posted;

    while(1){
        //wait for event
        posted = Event_pend(setpointDisplayEventHandle,
           Event_Id_00,
           Event_Id_NONE,
           BIOS_WAIT_FOREVER);
        if (posted == 0) {
           System_printf("Timeout expired for Event_pend()\n");
           break;
        }
        // update the LCD if the correct event trigger was posted.
        if (posted & Event_Id_00) {
            LCD_goto_xy(0,0);
            LCD_print_str("Setpt:");
            LCD_print_udec3(temperature);
            LCD_print_str(" F");

        }
    }
}

/*
 * This task waits on a new temperature value to be obtained from the ADC
 * then determines if it is different than the current temperature displayed
 * and if it is different, posts and event to signal the LCD should update.
 */
Void inputTask(UArg arg0, UArg arg1)
{
    UInt posted;
    uint8_t old = 0;

   while(1) {
       // waits for the correct event to signal a new conversion is completed
       posted = Event_pend(readingAvailableEventHandle,
           Event_Id_00,
           Event_Id_NONE,
           BIOS_WAIT_FOREVER);

       if (posted == 0) {
           System_printf("Timeout expired for Event_pend()\n");
           break;
       }

       // signal to LCD task if the new temperature value is different than whats currently displayed
       if (posted & Event_Id_00) {
           if(temperature != old){
               old = temperature;
               Event_post(setpointDisplayEventHandle, Event_Id_00);
           }
       }
   }
   BIOS_exit(0);

}

/*
 * Hardware Interrupt function
 * called when a new ADC value is ready.
 * signals to a SWI when a conversion is ready
 */
Void hwiFxn(){
    uint32_t adc_val = ADC14->MEM[4]; //clear the interrupt by reading the value
    Swi_post(swiHandle); // signal to the SWI that a new conversion is ready
}

/*
 * Software interrupt function
 * Reads the ADC value, converts it to the temperature range 50-90F
 * uses an event to signal to the inputTask that a new conversion has been read
 */
Void swiFxn(UArg arg0, UArg arg1)
{
    temperature = (40-(uint8_t)((((float)ADC14->MEM[4])/ADC_RANGE)*40))+50;
    Event_post(readingAvailableEventHandle, Event_Id_00);
}


