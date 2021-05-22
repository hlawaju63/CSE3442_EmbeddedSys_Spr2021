// Hemanta Lawaju
// Final Robot Project file

#include <stdint.h>
#include <stdbool.h>
//#include "timer0.h"
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"


#include <string.h>


#include "wait.h"

//PORT D PD0 N PD1
#define TRIG_MASK 4
#define ECHO_MASK 1

//bitbanding PD2
#define TRIG      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
//bitbanding PD0
#define ECHO      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))



#define MAX_CHARS 80
#define MAX_FIELDS 5


//PORT D PD0 N PD1
#define TRIG_MASK 4
#define ECHO_MASK 1

//bitbanding PD2
#define TRIG      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))
//bitbanding PD0
#define ECHO      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))


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


typedef struct _instruction
{
    uint8_t command;
    uint8_t subcommand;
    uint16_t argument;
}instruction;


typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

bool timeMode = false;
uint32_t frequency = 0;
uint32_t time = 0;
int distance = 0;
int stopDist = 50;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

/*
// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

}
*/

//***************************************
//*****************************************
//***********lab10************************
//*****************************************
//*****************************************
//**************************************

void initTimer0()
{

    //initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;
    //SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;
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

/*
void setTimerLoadValue(uint32_t loadValue)
{
    TIMER0_TAILR_R = loadValue;
}

void clearTimer0InterruptFlag()
{
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;
}

*/

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




//lab-9 code
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************


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


void getsUart0(USER_DATA *data)
{

    uint8_t count = 0;


    while(1)
    {
        char c = getcUart0(); // continuously getting getting charac from Uart0

        //if user press backspace
        if( (c  == 8 || c == 127) & count > 0 )
        {
            count--;
        }
        //if user press null
        else if(c == 13 )
        {
            data->buffer[count] = '\0';
            return;
        }
        //is char
        else if(c >=32 )
        {
            if(count == MAX_CHARS)
            {
                data->buffer[count] = '\0';
                return;
            }
            data->buffer[count] = c;
            count++;
        }
    }
}

bool isAlpha(char c)
{

    if( (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')   )
        return true;
    else
        return false;

}

bool isNum(char c)
{
    if( (c >= 48 && c <= 57)   )
          return true;
    else
          return false;
}

/*
bool isPerHyph(char c )
{
    if( (c >= 45 && c <= 46)   )
          return true;
    else
          return false;
}
*/

void parseFields(USER_DATA *data)
{

    int pos = 0; //pos is used to go through each char in buffer
    int fieldPos = 0; //it represents the place where it changes from delimeter to alpha or num
    int found = 0;
    while(data->buffer[pos] != '\0')
    {
        if(pos == MAX_CHARS)
           return;


        if(pos == 0)
        {
           if( isAlpha( data->buffer[pos] ) || isNum( data->buffer[pos] ) )
           {
               found = 1;

                  data->fieldPosition[fieldPos] = pos;

                  if( isAlpha( data->buffer[pos])  )
                  {
                      data->fieldType[fieldPos] = 'a';
                  }
                  else
                  {
                      data->fieldType[fieldPos] = 'n';
                  }

                  data->fieldCount = ++fieldPos;
           }//inner if
           else
           {
                   data->buffer[pos] = '\0';  //converting all delimeters into  before returning

           }
        }//outer if

        else
        {
           if( (isAlpha( data->buffer[pos] ) || isNum( data->buffer[pos] ) ) )
           {

               if(found == 0)
               {

                  found = 1 ;

                  data->fieldPosition[fieldPos] = pos;

                  if( isAlpha( data->buffer[pos] ) )
                  {
                      data->fieldType[fieldPos] = 'a';
                  }
                  else
                  {
                      data->fieldType[fieldPos] = 'n';
                  }

                  data->fieldCount = ++fieldPos;

               }


           }//inner if
           else
           {
               found = 0;
               data->buffer[pos] = '\0';     //converting all delimeters into  before returning
           }


        }//outer if
        pos++;
    }//while



}//method


char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{
    if(fieldNumber >= data->fieldCount )
        return NULL;
    else
    {
        return &data->buffer[ data->fieldPosition[ fieldNumber] ]  ;
    }
}


int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    if(  fieldNumber >= data->fieldCount )
    {
        return 0;
    }

    else
    {
        char *c = &data->buffer[ data->fieldPosition[ fieldNumber] ];

        int num = 0;

        for( c ;  *c != '\0' ;  ++c)
        {
            if(*c >= '0' && *c <= '9')
            {
                num = (*c - '0') + num*10;
            }
            else
            {
                return 0;
            }

        }
        return num;

    }

}


bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    char *c = &data->buffer[ data->fieldPosition[0] ];

    if(minArguments <= ( data->fieldCount - 1 ) )
    {
        while( *c != '\0' || *strCommand != '\0')
        {

            if ( *c  == *strCommand )
            {
                c++;
                strCommand++;
            }
            else
            {
            return false;
            }


        }
        return true;
    }

    else
    {
        return false;
    }

}

//lab-8 code
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************
//***************************-----------------------***************************



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

    setPwmDutyCycle(0, 0, 80); //counter clockwise
    setPwmDutyCycle(1, 99, 0); //clockwise

    while(true)
    {
       distance = getdistance();
       //char out[50];
       //sprintf(out, "distance %d\n", distance);
       //putsUart0(out);
       //waitMicrosecond(100000);

        //int x = getDistance();
        //|| (distance >= stopDist)
        if(getEncoderPosition(0) == count || getEncoderPosition(1) == count || (distance < stopDist) )
        {
            //distance = getDistance();
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

    setPwmDutyCycle(0, 80, 0); //counter clockwise -backward
    setPwmDutyCycle(1, 0, 99); //clockwise - backward

    while(true)
    {
        //distance = getDistance();
        //stopDist in reverse mode has to be modified
        //|| (distance >= stopDist)
        if(getEncoderPosition(0)  == count || getEncoderPosition(1)  == count )
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

//this function is for comparing pb and distance after wait command,
bool isCommandPbDist(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    //so when user inputs wait pb we can use pb as function for wait
    //and  when user inputs wait distance 10(cm) we take distance to compare if the ultrasonic
    //distance is 10 and make robot stop
    // only index inside field position  is changed
    char *c = &data->buffer[ data->fieldPosition[1] ];

    if(minArguments <= ( data->fieldCount - 1 ) )
    {
        while( *c != '\0' || *strCommand != '\0')
        {

            if ( *c  == *strCommand )
            {
                c++;
                strCommand++;
            }
            else
            {
            return false;
            }


        }
        return true;
    }

    else
    {
        return false;
    }

}





int main(void)
{

    // Initialize hardware
    initHw();
    initializePwm();

    initTimer0();
    initUart0();

    setUart0BaudRate(115200, 40e6);

    //putsUart0("Type commands\n");

    //int distance = 0;

/*
    while(true)
    {
        distance = getdistance();
        char out[50];
        sprintf(out, "distance %d\n", distance);
        putsUart0(out);
        waitMicrosecond(100000);

    }

    */


    //struct USER_DATA
        USER_DATA data;
        instruction robotInstr[50];  // creating an array to store instruction for robot
        int count = 0;
        int insertArrayIndex = 0;
        int x = 0;
        bool insertCmd = false;
        char buffer [50];


        while(true)
           {
               bool valid = false;


               // ---------------------------------------------------
               // Get the string from the user
               getsUart0(&data);
               // Echo back to the user of the TTY interface for testing


               // Parse fields
               parseFields(&data);

               if(insertCmd)
               {
                   x = count; //stores the last location of array where we should insert new cmd
                   count = insertArrayIndex;
               }
               else
               {
                   count = x;
               }


               if (isCommand(&data, "forward", 0))
               {
                   // do something with this information
                   if(data.fieldCount > 1)
                   {
                       robotInstr[count].argument = getFieldInteger(&data, 1);
                   }
                   else
                   {
                       robotInstr[count].argument = 0xFFFF;
                   }
                   robotInstr[count].command = 0;
                   valid = true;

               }
               else if(isCommand(&data, "reverse", 0))
               {

                   if(data.fieldCount > 1)
                   {
                       robotInstr[count].argument = getFieldInteger(&data, 1);
                   }
                   else
                   {
                       robotInstr[count].argument = 0xFFFF;
                   }
                   robotInstr[count].command = 1;
                   valid = true;
               }
               else if(isCommand(&data, "cw", 0))
               {

                   if(data.fieldCount > 1)
                   {
                       robotInstr[count].argument = getFieldInteger(&data, 1);
                   }
                   else
                   {
                       robotInstr[count].argument = 0xFFFF;
                   }
                   robotInstr[count].command = 2;
                   valid = true;
               }
               else if(isCommand(&data, "ccw", 0))
               {
                   // do something with this information
                   if(data.fieldCount > 1)
                   {
                       robotInstr[count].argument = getFieldInteger(&data, 1);
                   }
                   else
                   {
                       robotInstr[count].argument = 0xFFFF;
                   }
                   robotInstr[count].command = 3;
                   valid = true;
               }
               else if(isCommand(&data, "wait", 1))
               {

                   //if wait pb is the command
                   if(isCommandPbDist(&data, "pb", 0))
                   {
                       //robotInstr[count].argument = 0;
                       robotInstr[count].subcommand = 0;
                       robotInstr[count].command = 4;
                       valid = true;

                   }//if wait distance dist_cm is the command
                   //where dist_cm is to be met by ultrasonic sensor
                   else if(isCommandPbDist(&data, "distance", 0))
                   {
                       robotInstr[count].argument = getFieldInteger(&data, 2);
                       stopDist = robotInstr[count].argument;
                       robotInstr[count].subcommand = 1;
                       robotInstr[count].command = 4;
                       valid = true;
                   }



               }
               else if(isCommand(&data, "pause", 1))
               {
                   robotInstr[count].argument = getFieldInteger(&data, 1);
                   robotInstr[count].command = 5;

                   valid = true;
               }
               else if(isCommand(&data, "stop", 0))
               {

                   robotInstr[count].argument = 0;
                   robotInstr[count].command = 6;
                   valid = true;
               }
               else if(isCommand(&data, "list", 0))
               {
                   //print all the instruction of the array
                   int i = 0;


                   for(i ; i < count ; i++)
                   {
                       if(robotInstr[i].command == 0)
                       {
                           sprintf (buffer, "%d \t forward\t %d ", i+1, robotInstr[i].argument);
                           putsUart0(buffer);
                           putcUart0('\n');
                       }
                       else if(robotInstr[i].command == 1)
                       {
                           sprintf (buffer, "%d \t reverse\t %d ", i+1, robotInstr[i].argument);
                           putsUart0(buffer);
                           putcUart0('\n');

                       }
                       else if(robotInstr[i].command == 2)
                       {
                           sprintf (buffer, "%d \t cw\t %d ", i+1, robotInstr[i].argument);
                           putsUart0(buffer);
                           putcUart0('\n');
                       }
                       else if(robotInstr[i].command == 3)
                       {
                           sprintf (buffer, "%d \t ccw\t %d ", i+1, robotInstr[i].argument);
                           putsUart0(buffer);
                           putcUart0('\n');
                       }
                       else if(robotInstr[i].command == 4)
                       {
                           if(robotInstr[i].subcommand == 0)
                           {
                               sprintf (buffer, "%d \t wait \t pb  ", i+1);
                               putsUart0(buffer);
                               putcUart0('\n');
                           }
                           else if(robotInstr[i].subcommand == 1)
                           {
                               sprintf (buffer, "%d \t wait\t distance \t %d  ", i+1, robotInstr[i].argument);
                               putsUart0(buffer);
                               putcUart0('\n');
                           }
                       }
                       else if(robotInstr[i].command == 5)
                       {
                           sprintf (buffer, "%d \t pause\t %d ", i+1, robotInstr[i].argument);
                           putsUart0(buffer);
                           putcUart0('\n');
                       }
                       else if(robotInstr[i].command == 6)
                       {
                           sprintf (buffer, "%d \t stop\t ", i+1);
                           putsUart0(buffer);
                           putcUart0('\n');
                       }
                   }
                   valid = true;
                   count--;
               }
               else if(isCommand(&data, "delete", 1))
               {

                   if(data.fieldCount > 1)
                   {
                       int lineNum = getFieldInteger(&data, 1);
                       if(lineNum <= count)
                       {
                           int i = lineNum - 1 ;
                           for(i ; i < count - 1; i++)
                           {
                               robotInstr[i] = robotInstr[i+1];
                           }
                           count--;
                       }
                       valid = true;
                   }

               }

               else if(isCommand(&data, "insert", 1))
               {
                   // do something with this information
                   if(data.fieldCount > 1)
                   {
                       int lineNum = getFieldInteger(&data, 1);
                       if(lineNum <= count)
                       {
                           int i = count - 1 ;
                           for(i ; i >= (lineNum -1) ; i--)
                           {
                               robotInstr[i + 1] = robotInstr[i];
                           }
                           //making insertCmd variable true
                           insertCmd = true;
                           count++;
                           insertArrayIndex = i+1;
                           continue;// go to top of loop

                           valid = true;

                       }

                   }
               }
               else if(isCommand(&data, "run", 0))
               {
                   // do something with this information
                   int i = 0;

                   for(i ; i < count ; i++)
                   {
                       if(robotInstr[i].command == 0) {
                           forward(robotInstr[i].argument);
                       }
                       else if(robotInstr[i].command == 1) {
                           reverse(robotInstr[i].argument);
                       }
                       else if(robotInstr[i].command == 2) {
                           cw(robotInstr[i].argument);
                       }
                       else if(robotInstr[i].command == 3) {
                           ccw(robotInstr[i].argument);
                       }
                       else if(robotInstr[i].command == 4) {
                           // Wait push button
                           //pb
                           if(robotInstr[i].subcommand == 0)
                           {
                               waitpb();
                           }//distance
                           /*
                           else if(robotInstr[i].subcommand == 1)
                           {
                                //do nthing since we handle it by setting stopDist = dist_cm value from wait Dist dist_cm
                               waitpb();
                           }
                           */

                       }
                       else if(robotInstr[i].command == 5) {
                           pause(robotInstr[i].argument);
                       }
                       else if(robotInstr[i].command == 6) {
                           stop();
                       }
                   }
                   valid = true;

               }


               if (!valid)
                   putsUart0("Invalid command\n");


               count++;


               if(!insertCmd)
               {
                   x = count;
               }
               else
               {
                   count = x;
                   insertCmd = false;
               }



           }


    while(true);
}



