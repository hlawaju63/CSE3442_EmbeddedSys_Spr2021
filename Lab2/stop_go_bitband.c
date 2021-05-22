// Stop Go C Example (Bitbanding)
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
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 2*4)))

#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4)))
#define YELLOW_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4)))

#define PUSH_BUTTON_1 (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define PUSH_BUTTON_2 (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))


// PortB masks
#define RED_LED_MASK 8
#define BLUE_LED_MASK 4

// PortE masks
#define GREEN_LED_MASK 1
#define YELLOW_LED_MASK 2

// PortD masks
#define PUSH_BUTTON_MASK_1 1
#define PUSH_BUTTON_MASK_2 2

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
void waitPb1Press(void)
{
	while(PUSH_BUTTON_1);
}

void waitPb2Press(void)
{
    while(!PUSH_BUTTON_2);
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// Initialize Hardware
void initHw(void)
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R3 | SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTB_DIR_R |= RED_LED_MASK | BLUE_LED_MASK;   // bits 2 and 3 are outputs, other pins are inputs
    GPIO_PORTE_DIR_R |= GREEN_LED_MASK | YELLOW_LED_MASK ;   // bits 0 and 1 are outputs, other pins are inputs

    GPIO_PORTD_DIR_R &= ~PUSH_BUTTON_MASK_1;           // bit 0 is an input
    GPIO_PORTD_DIR_R &= ~PUSH_BUTTON_MASK_2;            // bit 1 is an input

    GPIO_PORTB_DR2R_R |= RED_LED_MASK | BLUE_LED_MASK;  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DR2R_R |= GREEN_LED_MASK | YELLOW_LED_MASK;
    GPIO_PORTD_DR2R_R |= PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 ;

    //Digital enable to read or write inputs the following pins of the port
    GPIO_PORTB_DEN_R |= RED_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTE_DEN_R |= GREEN_LED_MASK | YELLOW_LED_MASK ;
    GPIO_PORTD_DEN_R |= PUSH_BUTTON_MASK_1 | PUSH_BUTTON_MASK_2 ;

                                                         // enable LEDs and pushbuttons
    GPIO_PORTD_PUR_R |= PUSH_BUTTON_MASK_1;                // enable internal pull-up for push button
    GPIO_PORTD_PDR_R |= PUSH_BUTTON_MASK_2;                // enable internal pull-up for push button
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();

    RED_LED = 0;
    BLUE_LED = 0;
    GREEN_LED = 1;
    YELLOW_LED = 1;

    RED_LED = 1;

    waitPb2Press();

    RED_LED = 0;
    GREEN_LED = 0;

    waitMicrosecond(1000000);

    BLUE_LED = 1;

    waitPb1Press();

    while(true)
    {
        waitMicrosecond(500000);
        YELLOW_LED ^= 1;
    }

    /*
    // Turn off green LED, turn on red LED
    GPIO_PORTF_DATA_R &= ~GREEN_LED_MASK;
    GPIO_PORTF_DATA_R |= RED_LED_MASK;

    // Wait for PB press
    waitPbPress();

    // Turn off red LED, turn on green LED
    GPIO_PORTF_DATA_R &= ~RED_LED_MASK;
    GPIO_PORTF_DATA_R |= GREEN_LED_MASK;

    // Endless loop
     */

}
