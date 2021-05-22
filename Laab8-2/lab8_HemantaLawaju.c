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

//id 1 is left id 0 is right
//setPwmDutyCycle(0, 80, 0); //counter-clockwise - backward
//setPwmDutyCycle(1, 80, 0); //clockwise -forward

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


//PortB Masks
#define YELLOW_BL_LED_MASK 16
#define RED_BL_LED_MASK 32

//PortE masks
#define BLUE_BL_LED_MASK 16
#define GREEN_BL_LED_MASK 32

#define LOAD_VAL        1024

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
    GPIO_PORTB_AFSEL_R |=RED_BL_LED_MASK | YELLOW_BL_LED_MASK  ; //select auxillary function
    GPIO_PORTB_PCTL_R &= ~(GPIO_PCTL_PB5_M | GPIO_PCTL_PB4_M | GPIO_PCTL_PB3_M);    //enable PWM
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_M0PWM3 | GPIO_PCTL_PB4_M0PWM2;

    // Turning PD6 on (high)
    GPIO_PORTD_DATA_R  |= GPIO_SLEEP;

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


//lab 8 methods
//makes robot go forward num of magnetic rotation on each wheel
void forward(uint16_t dist_cm)
{
    //diameter = 6.476cm, radius (r1) = 3.238cm , 2pir1 = 46 count , N cm = (46/2pir1) * N count
    float val = ( 46 / (2 * 3.141592 * 2.238)  ) * dist_cm;   //N cm = (46/2pir) * N count
    int count = val;

    setEncoderPosition(0, 0);
    setEncoderPosition(1, 0);

    setPwmDutyCycle(0, 0, 99); //counter clockwise
    setPwmDutyCycle(1, 99, 0); //clockwise

    while(true)
    {
        if(getEncoderPosition(0) == count || getEncoderPosition(1) == count)
        {
            //sleep if reaches count num of rotation on small magnet(hall effect sensor)
            setPwmDutyCycle(0, 0, 0); //counter clockwise - forward
            setPwmDutyCycle(1, 0, 0); //clockwise - forward
            break;
        }

    }
}


//makes robot go reverse num of magnetic rotation on each wheel
void reverse(uint16_t dist_cm)
{
    //diameter = 6.476cm, radius (r1) = 3.238cm , 2pir1 = 46 count , N cm = (46/2pir1) * N count
    float val = ( 46 / (2 * 3.141592 * 2.238)  ) * dist_cm;   //N cm = (46/2pir1) * N count
    int count = val;

    setEncoderPosition(0, 0);
    setEncoderPosition(1, 0);

    setPwmDutyCycle(0, 99, 0); //counter clockwise -backward
    setPwmDutyCycle(1, 0, 99); //clockwise - backward

    while(true)
    {
        if(getEncoderPosition(0)  == count || getEncoderPosition(1)  == count)
        {
            //sleep if reaches count num of rotation on small magnet(hall effect sensor)
            setPwmDutyCycle(0, 0, 0); //counter clockwise
            setPwmDutyCycle(1, 0, 0); //clockwise
            break;
        }

    }
}

void cw(uint16_t degrees)
{
        //radius of frame (r2) = 8.5 cm,  radius (r1) = 3.238cm , 2pir1 = 46 count , N cm = (46/2pir) * N count
        // count = (46*r2*degree) / for given degree

        float val = (46*8.5*degrees)/(3.283*360);
        int count = val;

        setEncoderPosition(0, 0);
        setEncoderPosition(1, 0);

        setPwmDutyCycle(0, 99, 0); // clockwise - forward
        setPwmDutyCycle(1, 99, 0); // clockwise - backward

        while(true)
        {
            if(getEncoderPosition(0)  == count || getEncoderPosition(1)  == count)
            {
                //sleep if reaches count num of rotation on small magnet(hall effect sensor)
                setPwmDutyCycle(0, 0, 0); //counter clockwise
                setPwmDutyCycle(1, 0, 0); //clockwise
                break;
            }

        }

}

void ccw(uint16_t degrees)
{
    //radius of frame (r2) = 8.5 cm,  radius (r1) = 3.238cm , 2pir1 = 46 count , N cm = (46/2pir) * N count
        // count = (46*r2*degree) / for given degree
        float val  = (46*8.5*degrees)/(3.283*360);
        int count = val;

        setEncoderPosition(0, 0);
        setEncoderPosition(1, 0);

        setPwmDutyCycle(0, 0, 99); //counter clockwise - backward
        setPwmDutyCycle(1, 0, 99); //counter clockwise - forward

        while(true)
        {
            if(getEncoderPosition(0)  == count || getEncoderPosition(1)  == count)
            {
                //sleep if reaches count num of rotation on small magnet(hall effect sensor)

                setPwmDutyCycle(0, 0, 0); //counter clockwise
                setPwmDutyCycle(1, 0, 0); //counternclockwise
                break;

            }

        }

}

void waitpb()
{
    while(PUSH_BUTTON);

}



void pause(uint32_t us)
{
    GPIO_PORTD_DATA_R  &= ~GPIO_SLEEP;
    waitMicrosecond(us);
    GPIO_PORTD_DATA_R  |=  GPIO_SLEEP;
}

void stop()
{

    GPIO_PORTD_DATA_R  &= ~GPIO_SLEEP;
    setPwmDutyCycle(0, 0, 0); //counter clockwise
    setPwmDutyCycle(1, 0, 0); //counternclockwise

}

int Odometer(void)
{
    // Initialize hardware
    initHw();
    initUart0();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    setEncoderPosition(0, 0);
    setEncoderPosition(1, 0);

    selectEncoderIncMode(0);
    selectEncoderIncMode(1);

}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
 {
    // Initialize hardware
    //Odometer();
    initHw();
    // Initialize hardware
    initializePwm();




    waitpb();

    forward(30);

    reverse(40);

    pause(5000000);

    stop();

    cw(90);

    forward(150);

    ccw(90);


}

