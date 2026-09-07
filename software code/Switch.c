/*
 * Switch.c
 *
 *  Created on: January 12, 2026
 *      Author:
 */
#include <ti/devices/msp/msp.h>
#include "../inc/LaunchPad.h"
// LaunchPad.h defines all the indices into the PINCM table
void Switch_Init(void){
    // write this
  IOMUX->SECCFG.PINCM[53] = 0x00040081;
  IOMUX->SECCFG.PINCM[54] = 0x00040081;
  IOMUX->SECCFG.PINCM[58] = 0x00040081;
  IOMUX->SECCFG.PINCM[59] = 0x00040081;

}
// return current state of switches
uint32_t Switch_In(void){
    // write this
    uint32_t input = GPIOA->DIN31_0;
    input = ((input >>24) & 0x0F);
  return input; // replace this line
}
