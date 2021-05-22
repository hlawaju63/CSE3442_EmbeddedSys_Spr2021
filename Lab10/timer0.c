// Hemanta Lawaju
// Final Robot Project file

#include <stdint.h>
#include <stdbool.h>
#include "timer0.h"
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"


//PORT D PD0 N PD2
#define TRIG_MASK 4
#define ECHO_MASK 1

//bitbanding PD2
#define TRIG      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
//bitbanding PD0
#define ECHO      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))




//***************************************
//*****************************************
//***********lab10************************
//*****************************************
//*****************************************
//**************************************

void initTimer0()
{

    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
    _delay_cycles(3);


    //Configure four GPIO for sonar senesor
    GPIO_PORTD_DIR_R |= TRIG_MASK ; //make bit 1 output for trig
    GPIO_PORTD_DIR_R &=  ~(ECHO_MASK) ; //make bit 0 input echo
    GPIO_PORTD_DR2R_R |=TRIG_MASK ; //set drive strength to 2 mA
    GPIO_PORTD_DEN_R |= TRIG_MASK | ECHO_MASK; //enable digital

    // Configure Timer 1 as the time base
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_1_SHOT | TIMER_TAMR_TACDIR;          // configure for periodic mode (count down)
    // TIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    // NVIC_EN0_R |= 1 << (INT_TIMER0A-16);             // turn-on interrupt 37 (TIMER1A)
}

void startTimer0()
{
    TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

void stopTimer0()
{
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                  // turn-on timer
}



int getdistance()
{
        TRIG = 0;

        //putsUart0("Example\n");
        TIMER0_TAV_R = 0;
        //ECHO = 0;
        //turn on the trig
        TRIG = 1;
        waitMicrosecond(15);
        TRIG = 0;
        while(!ECHO);
        //startTimer

        startTimer0();
        while(ECHO);
        int val = TIMER0_TAV_R;
        stopTimer0();

        int distance = (val * 0.025) / 58;

/*
        char out[50];
        sprintf(out, "distance %d\n", distance);
        putsUart0(out);

        waitMicrosecond(100000);
*/

        return distance;
}









int main(void)
{

    // Initialize hardware
    //initHw();
    //initializePwm();

    initTimer0();
    initUart0();

    setUart0BaudRate(115200, 40e6);

    //putsUart0("Type commands\n");

    int distance = 0;


    while(true)
    {
        distance = getdistance();
        char out[50];
        sprintf(out, "distance %d\n", distance);
        putsUart0(out);
        waitMicrosecond(100000);

    }





}



