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

#define MAX_CHARS 80
#define MAX_FIELDS 5


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
/*
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
                    }
                    else if(robotInstr[i].subcommand == 1)
                    {

                        waitpb();
                    }

                }
                else if(robotInstr[i].command == 5) {
                    pause(robotInstr[i].argument);
                }
                else if(robotInstr[i].command == 6) {
                    stop();
                }
            }

*/
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
}


/*
        #ifdef DEBUG
        putsUart0(data.buffer);
        putcUart0('\n');
        #endif
*/


/*
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
*/
