// Stop Go C/ASM Mix Example
// Hemanta Lawaju

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "tm4c123gh6pm.h"

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
#define PUSH_BUTTON_MASK 16

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
extern void waitPbPress(void);

void wait1Second(void)
{
        __asm(".const");
        __asm("name .field 10000000");
        __asm(" LDR R0, name");
        __asm("HERE:    SUB R0, #1");
        __asm("         CBZ R0, END");
        __asm("         B HERE");
        __asm("END:");
}

// Initialize Hardware
void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;   // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;               // bit 4 is an input
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | GREEN_LED_MASK | RED_LED_MASK;
                                                         // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	//initHw();

	wait1Second();

	/*
    // Turn on red LED, turn off green LED
    RED_LED = 1;
    GREEN_LED = 0;

    // Wait for PB press
    waitPbPress();

    // Turn off red LED
    RED_LED = 0;

    // Turn on green LED
    __asm("            PUSH {R0, R1}");
    __asm("            B    next");
    __asm("GREEN_LED:  .field 0x424A7F8C");
    __asm("next:       LDR R0, GREEN_LED");
    __asm("            MOV R1, #1");
    __asm("            STR R1, [R0]");
    __asm("            POP {R0, R1}");

    // Endless loop
     *

     */
    while(true);
}
