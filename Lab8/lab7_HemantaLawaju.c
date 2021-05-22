// RGB Backlight PWM Example
// Hemanta Lawaju

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   M0PWM3 (PB5) drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   M0PWM5 (PE5) drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   M0PWM4 (PE4) drives an NPN transistor that powers the blue LED
// ST7565R Graphics LCD Display Interface:
//   MOSI on PD3 (SSI1Tx)
//   SCLK on PD0 (SSI1Clk)
//   ~CS on PD1 (SSI1Fss)
//   A0 connected to PD2

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "graphics_lcd.h"
#include "backlight.h"
#include "wait.h"
#include "tm4c123gh6pm.h"

//PortB Masks
#define YELLOW_BL_LED_MASK 16
#define RED_BL_LED_MASK 32


//PortE masks
#define BLUE_BL_LED_MASK 16
#define GREEN_BL_LED_MASK 32

#define LOAD_VAL        1024

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
}

void initializePwm(void)
{
    //we r using gen 1 and 2 of PWM0

    //ENABLE clock
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R4;
    _delay_cycles(3);

    //Configure four backlight LEDs
    GPIO_PORTB_DIR_R |= RED_BL_LED_MASK | YELLOW_BL_LED_MASK; //make bit 5 and 4 output
    GPIO_PORTB_DR2R_R |=RED_BL_LED_MASK | YELLOW_BL_LED_MASK; //set drive strength to 2 mA
    GPIO_PORTB_DEN_R |= RED_BL_LED_MASK | YELLOW_BL_LED_MASK; //enable digital
    GPIO_PORTB_AFSEL_R |=RED_BL_LED_MASK | YELLOW_BL_LED_MASK; //select auxillary function
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB5_M | GPIO_PCTL_PB4_M);    //enable PWM
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB4_M0PWM2;

    GPIO_PORTE_DIR_R |= GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;  // make bits 4 and 5 outputs
    GPIO_PORTE_DR2R_R |= GREEN_BL_LED_MASK | BLUE_BL_LED_MASK; // set drive strength to 2mA
    GPIO_PORTE_DEN_R |= GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;  // enable digital
    GPIO_PORTE_AFSEL_R |= GREEN_BL_LED_MASK | BLUE_BL_LED_MASK;// select auxilary function
    GPIO_PORTE_PCTL_R &= ~(GPIO_PCTL_PE4_M | GPIO_PCTL_PE5_M);    // enable PWM
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE4_M0PWM4 | GPIO_PCTL_PE5_M0PWM5;

    //Configure PWM module 0 to drive RGBY backlight
    //RED on M0PWM3 (PB5), M0PWM1b
    //YELLOW on M0PWM2 (PB4), M0PWM1a
    //BLUE on M0PWM4(PE4), M0PWM2a
    //GREEN on M0PWM5(PE5), M0PWM2b

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R0;       //reset PWM0 module
    SYSCTL_SRPWM_R = 0;                     // leave the reset state
    PWM0_1_CTL_R = 0;                       //turn-off PWM0 generator 1 (drive outputs 2 and 3)
    PWM0_2_CTL_R = 0;                       //turn-off PWM0 generator 2(drives output 3 and 4)

    PWM0_1_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; //output 2 on PWM0, gen 1a, cmpa
    PWM0_1_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; //output 3 on PWM0, gen 1b, cmpb

    PWM0_2_GENA_R = PWM_0_GENA_ACTCMPAD_ZERO | PWM_0_GENA_ACTLOAD_ONE; //output 4 on PWM0, gen 1a, cmpa
    PWM0_2_GENB_R = PWM_0_GENB_ACTCMPBD_ZERO | PWM_0_GENB_ACTLOAD_ONE; //output 5 on PWM0, gen 1b, cmpb


    PWM0_1_LOAD_R = LOAD_VAL;               //set frequency to 40 MHz sys clocl / 2 / 1024 = 19.53125
    PWM0_2_LOAD_R = LOAD_VAL;               //set frequency to 40 MHz sys clocl / 2 / 1024 = 19.53125

    //Invert outputs so duty cycle increase with increase with increasing comapre values
    PWM0_INVERT_R = PWM_INVERT_PWM2INV | PWM_INVERT_PWM3INV | PWM_INVERT_PWM4INV | PWM_INVERT_PWM5INV;

    PWM0_1_CMPA_R = 0;                                //yellow
    PWM0_1_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM0_2_CMPB_R = 0;                               // green off
    PWM0_2_CMPA_R = 0;                               // blue off

    PWM0_1_CTL_R = PWM_0_CTL_ENABLE;        //turn on PWM0 generator 1
    PWM0_2_CTL_R = PWM_0_CTL_ENABLE;        //turn-on PWM0 generator 2
    PWM0_ENABLE_R = PWM_ENABLE_PWM2EN | PWM_ENABLE_PWM3EN | PWM_ENABLE_PWM4EN | PWM_ENABLE_PWM5EN; //enable out puts

}

void setPwmDutyCycle(uint8_t id, uint16_t pwmA, uint16_t pwmB)
{
    if(id == 0)
    {
        PWM0_1_CMPB_R = (pwmA * LOAD_VAL) / 100; // (cmp * 100) /1024 = DC
        PWM0_1_CMPA_R = (pwmB * LOAD_VAL) / 100; // (cmp\loadval) * 100  = DutyCycle
    }
    if(id == 1)
    {
        PWM0_2_CMPB_R = (pwmA * LOAD_VAL) / 100; // (cmp * 100) /1024 = DC
        PWM0_2_CMPA_R = (pwmB * LOAD_VAL) / 100; // (cmp\loadval) * 100  = DutyCycle

    }

}

/*
void setBacklightRgbColor(uint16_t red, uint16_t yellow, uint16_t green, uint16_t blue)
{

    PWM0_1_CMPB_R = red;
    PWM0_1_CMPA_R = yellow;
    PWM0_2_CMPA_R = blue;
    PWM0_2_CMPB_R = green;
}
*/


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();
    initGraphicsLcd();
    //initBacklight();
    initializePwm();
    // Turn on all pixels for maximum light transmission
    //drawGraphicsLcdRectangle(0, 0, 128, 64, SET);

    // Cycle through colors
	int16_t i = 0;
	while(true)
	{

	    //a> 80% output on pwmA, low on pwmB
	    setPwmDutyCycle(0, 80, 0);
	    setPwmDutyCycle(1, 80, 0);
	    waitMicrosecond(1000000);

	    //b>  50% output on pwmA, low on pwmB
	    setPwmDutyCycle(0, 50, 0);
	    setPwmDutyCycle(1, 50, 0);
	    waitMicrosecond(1000000);
	    //c> 80% output on pwmA, high on pwmB
	    setPwmDutyCycle(0, 80, 100);
	    setPwmDutyCycle(1, 80, 100);
	    waitMicrosecond(1000000);
	    //d> Low output on pwmA, 80% on pwmB
	    setPwmDutyCycle(0, 0, 80);
	    setPwmDutyCycle(1, 0, 80);
	    waitMicrosecond(1000000);
	    //e> Low output on pwmA, 50% on pwmB
	    setPwmDutyCycle(0, 0, 50);
	    setPwmDutyCycle(1, 0, 50);
	    waitMicrosecond(1000000);
	    //f> High output on pwmA, 80% on pwmB
	    setPwmDutyCycle(0, 100, 80);
	    setPwmDutyCycle(1, 100, 80);
	    waitMicrosecond(1000000);

	}
}
