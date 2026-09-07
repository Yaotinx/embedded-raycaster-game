/*
 * LED.c
 *
 *  Created on: Nov 5, 2023
 *      Author:
 */
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
// LaunchPad.h defines all the indices into the PINCM table

// initialize your LEDs
void LED_Init(void){
    // write this

  IOMUX->SECCFG.PINCM[36] = 0x81;
  IOMUX->SECCFG.PINCM[37] = 0x81;
  IOMUX->SECCFG.PINCM[38] = 0x81;
  GPIOA->DOE31_0 |= 0x00038000;;
}
// data specifies which LED to turn on
void LED_On(uint32_t data){
    // write this
    // use DOUTSET31_0 register so it does not interfere with other GPIO
   GPIOA->DOUTSET31_0 = data;
}

// data specifies which LED to turn off
void LED_Off(uint32_t data){
    // write this
    // use DOUTCLR31_0 register so it does not interfere with other GPIO
   GPIOA->DOUTCLR31_0 = data;
}

// data specifies which LED to toggle
void LED_Toggle(uint32_t data){
    // write this
    // use DOUTTGL31_0 register so it does not interfere with other GPIO
    GPIOA->DOUTTGL31_0 = data;
}
