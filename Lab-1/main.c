// main.c
// Runs on MSP432
// Lab 1 - Tool Time
// Draven Schilling
// 12/7/19
// display course number at the top of the lcd
// when toggle pushbutton, turn on led and display last name
// when pushbutton off, display first name and led off
//  **************************************************

#include <stdint.h>
#include "msp.h"
#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_delay.h"


// initialize port P1.1 GPIO IN for pushbutton
// initialize port 1.0 GPIO OUT for led
void init_gpio(void);

int main(void){

  if(Clock_Init_48MHz())
      P3->OUT |= BIT5;  // ******* debug

  // setup lcd
  init_gpio();
  LCD_Config();
  LCD_clear();
  LCD_home();
  LCD_contrast(10);

  LCD_print_str("EE4930");
  uint8_t pushbutton = 0;

  while(1){
      pushbutton = (P1->IN & 0b10) >> 1;

      if(pushbutton){
          //print to lcd
          LCD_goto_xy(0,1);
          LCD_print_str("OFF         ");
          LCD_goto_xy(0,2);
          LCD_print_str("Draven      ");
          P1->OUT &= ~0b1; // led off
      } else {
          //print to lcd
          LCD_goto_xy(0,1);
          LCD_print_str("ON          ");
          LCD_goto_xy(0,2);
          LCD_print_str("Schilling   ");
          P1->OUT |= 0b1; // led on
      }
  }
}

void init_gpio(void)
{
    // P1.0
    P1->SEL0 &= ~0b1; // use GPIO function
    P1->SEL1 &= ~0b1;
    P1->DIR |= 0b1;   // make output
    P1->OUT &= ~0b1;  // setup output low to start
    // P1.1
    P1->SEL0 &= ~0b10; // use GPIO function
    P1->SEL1 &= ~0b10;
    P1->DIR &= ~0b10;   // make input
    P1->REN |= 0b10; // allow pull up/down
    P1->OUT |= 0b10;  // setup as pull up
    return;
}
