// Sound.c
// Runs on MSPM0
// Sound assets in sounds/sounds.h
// your name
// your data 
#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include "Sound.h"
#include "sounds/sounds.h"
#include "../inc/DAC5.h"
#include "../inc/Timer.h"
#include "../inc/DAC5.h"
#include "LED.h"

void SysTick_IntArm(uint32_t period, uint32_t priority){
  // write this
  SysTick->CTRL = 0x00;      
  SysTick->LOAD = period-1;  
  SCB->SHP[1] = (SCB->SHP[1]&(~0xC0000000))|(priority<<30); 
  SysTick->VAL = 0;        
  SysTick->CTRL = 0x07;
}
const uint8_t *SoundPtr; // Pointer to the current sound buffer
uint32_t SoundCount = 0; // Samples remaining to be played
// initialize a 11kHz SysTick, however no sound should be started
// initialize any global variables
// Initialize the 5-bit DAC
void Sound_Init(void){
   DAC5_Init();
  SoundPtr = 0;
  SoundCount = 0;
  // For 80MHz clock, 11.025kHz is exactly 7256 cycles.
  // We initialize SysTick, but it will only "do" something when SoundCount > 0
  SysTick_IntArm(7256, 2); 
}
void SysTick_Handler(void){
  if(SoundCount > 0){
    DAC5_Out((*SoundPtr)); 
    SoundPtr++;
    SoundCount--;
  } else {
    DAC5_Out(16); // Mid-point (0-31) to let the speaker relax
  }
}


//******* Sound_Start ************
// This function does not output to the DAC. 
// Rather, it sets a pointer and counter, and then enables the SysTick interrupt.
// It starts the sound, and the SysTick ISR does the output
// feel free to change the parameters
// Sound should play once and stop
// Input: pt is a pointer to an array of DAC outputs
//        count is the length of the array
// Output: none
// special cases: as you wish to implement
void Sound_Start(const uint8_t *pt, uint32_t count){
  // Critical section: disable interrupts so SoundPtr and SoundCount 
  // don't get out of sync if a sound is already playing.
  __disable_irq();
  SoundPtr = pt;
  SoundCount = count;
  __enable_irq();
}

void Sound_shoot(void)
{
  Sound_Start(shoot, 4080);
}

void Sound_boss(void)
{
  Sound_Start(boss, 105605);
}

void Sound_explode(void)
{
  Sound_Start(explode, 8731);
}
