// Serial C/ASM Mix Example
// Jason Losh

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
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "lab5_HemantaLawaju.h"


// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs
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
    if( (c >= 49 && c <= 57)   )
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

    if(minArguments >= ( data->fieldCount - 1 ) )
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

#define DEBUG

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
	// Initialize hardware
	initHw();
	initUart0();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);

    //struct USER_DATA
    USER_DATA data;
    bool valid = false;

    // ---------------------------------------------------
    // Get the string from the user
    getsUart0(&data);
    // Echo back to the user of the TTY interface for testing
    #ifdef DEBUG
    putsUart0(data.buffer);
    putcUart0('\n');
    #endif

    // Parse fields
    parseFields(&data);

    // Echo back the parsed field data (type and fields)
    #ifdef DEBUG
    uint8_t i;
    for (i = 0; i < data.fieldCount; i++)
    {
    putcUart0(data.fieldType[i]);
    putcUart0('\t');
    uint8_t pos = data.fieldPosition[i];
    putsUart0(&data.buffer[pos]);
    putcUart0('\n');
    }
    #endif

    // Command evaluation
    // set add, data add and data are integers
    if (isCommand(&data, "set", 2))
    {
    int32_t add = getFieldInteger(&data, 1);
    int32_t data = getFieldInteger(&data, 2);
    valid = true;
    // do something with this information
    }
    // alert ON|OFFalert ON or alert OFF are the expected commands
    if (isCommand(&data, "alert", 1))
    {
    char* str = getFieldString(&data, 1);
    valid = true;
    // process the string with your custom strcmp instruction, then do something
    }
    // Process other commands here
    // Look for error
    if (!valid)
    putsUart0("Invalid command\n");

    while(true);
}
