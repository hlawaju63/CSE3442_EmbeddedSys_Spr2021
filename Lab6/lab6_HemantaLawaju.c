// Frequency Counter / Timer Example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for sprintf)

// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Blue LED:
//   PF2 drives an NPN transistor that powers the blue LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Frequency counter and timer input:
//   FREQ_IN on PC6 (WT1CCP0)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "wait.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

// PortC masks
#define FREQ_IN_MASK_1 64

//PortD masks
#define FREQ_IN_MASK_2 16
#define GPIO_SLEEP 64

// PortF masks
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void setCounterUp()
{
	WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
	WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
	WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
	WTIMER1_TAMR_R &= ~TIMER_TAMR_TACMR;
    WTIMER1_CTL_R = 0;                               // Positive Edge
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
	WTIMER1_TAV_R = 0;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

/*
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge count mode, count up
    WTIMER0_TAMR_R &= ~TIMER_TAMR_TACMR;
    WTIMER0_CTL_R = 0;                               // Positive Edge
    WTIMER0_IMR_R = 0;                               // turn-off interrupts
    WTIMER0_TAV_R = 0;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
*/

}

void setCounterDown()
{
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER0_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_CAP; // configure for edge count mode, count up
    WTIMER0_TAMR_R &= ~TIMER_TAMR_TACMR;
    WTIMER0_CTL_R = 0;                               //
    WTIMER0_IMR_R = 0;                               // turn-off interrupts
    WTIMER0_TAV_R = 40000000;                               // zero counter for first period
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter

/*
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER1_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER1_TAMR_R = TIMER_TAMR_TAMR_CAP; // configure for edge count mode, count up
    WTIMER1_TAMR_R &= ~TIMER_TAMR_TACMR;
    WTIMER1_CTL_R = 0;                               //
    WTIMER1_IMR_R = 0;                               // turn-off interrupts
    WTIMER1_TAV_R = 40000000;                               // zero counter for first period
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
*/
}

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    //  on two Timer module
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;


    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R3 ;
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | BLUE_LED_MASK;  // bits 1 and 2 are outputs, other pins are inputs
    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;               // bit 4 is an input
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
                                                         // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;                // enable internal pull-up for push button

    // Configure FREQ_IN for frequency counter for R2 i.e PORTC
	GPIO_PORTC_AFSEL_R |= FREQ_IN_MASK_1;              // select alternative functions for FREQ_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC6_M;           // map alt fns to FREQ_IN
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC6_WT1CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK_1;                // enable bit 6 for digital input




    // Configure FREQ_IN for frequency counter for R2 i.e PORTC
    GPIO_PORTC_AFSEL_R |=FREQ_IN_MASK_2;        // select alternative functions for FREQ_IN pin
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;      // map alt fns to FREQ_IN_2
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_WT0CCP0;
    GPIO_PORTC_DEN_R |= FREQ_IN_MASK_2;        // enable bit 6 for digital input

    //turning on pull up resistor for hall effect senseor
    GPIO_PORTC_PUR_R |= FREQ_IN_MASK_1 | FREQ_IN_MASK_2;

    // Turning PD6 on (high)
    //GPIO_PORTD_DATA_R  |= GPIO_SLEEP;

    setCounterUp();
    setCounterDown();
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

void setEncoderPosition(uint8_t id, int32_t position)
{
    if(id == 0)//left
    {
           WTIMER0_TAV_R = position;
    }
    else if(id == 1)//right
    {
           WTIMER1_TAV_R = position;
    }
}

int32_t getEncoderPosition(uint8_t id)
{
    if(id == 0)//left
    {
           return WTIMER0_TAV_R;
    }
    else if(id == 1)//right
    {
           return WTIMER1_TAV_R;
    }

}


void selectEncoderIncMode(uint8_t id)
{
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;

    if(id == 0)//left
     {
         WTIMER0_TAMR_R |=  TIMER_TAMR_TACDIR;
     }
     else if(id == 1)//right
     {
         WTIMER1_TAMR_R |=  TIMER_TAMR_TACDIR;
     }

    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void selectEncoderDecMode(uint8_t id)
{
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;

    if(id == 0)//left
     {
         WTIMER0_TAMR_R &=  ~TIMER_TAMR_TACDIR;
     }
     else if(id == 1)//right
     {
         WTIMER1_TAMR_R &=  ~TIMER_TAMR_TACDIR;
     }

    WTIMER0_CTL_R |= TIMER_CTL_TAEN;
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
}


/*
int main(void)
{
    // Initialize hardware
    initHw();
    initUart0();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);


    // Use blue LED to show mode
    uint32_t x = 0;
    uint32_t y = 0;

    while (true)
    {
        x = WTIMER0_TAV_R;
        y = WTIMER1_TAV_R;

    }



    setEncoderPosition(0, 0);
    setEncoderPosition(1, 0);

    selectEncoderIncMode(0);
    selectEncoderIncMode(1);

    while (true)
    {
        //a-print the value of counter
        char str1[80];
        char str2[80];
        sprintf(str1, "counter0 = %d, counter1 = %d\n", getEncoderPosition(0), getEncoderPosition(1) );

        putsUart0(str1);

        //b-configure both counter to increment mode
        //setCounterUp();

        waitMicrosecond(1000000);

        //c if count reaches 10
        if(getEncoderPosition(0) == 10 || getEncoderPosition(1) == 10)
        {
            //set counters to decrement
            selectEncoderDecMode(0);
            selectEncoderDecMode(1);
        }

        //c if count reaches -10
        if( getEncoderPosition(0) - 10 == -10 || getEncoderPosition(1) - 10 == -10)
        {
               //set counter to increment
            selectEncoderIncMode(0);
            selectEncoderIncMode(1);
        }


        //when id == 0 & counter reaches 10
        if( getEncoderPosition(0) == 10 )
        {
            //set 0 left wheel to dec mode
            selectEncoderDecMode(0);
        }

    }
}
*/
