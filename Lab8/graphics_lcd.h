// Graphics LCD Driver
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// ST7565R Graphics LCD Display Interface:
//   MOSI on PD3 (SSI1Tx)
//   SCLK on PD0 (SSI1Clk)
//   ~CS on PD1 (SSI1Fss)
//   A0 connected to PD2

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef GRAPHICS_LCD_H_
#define GRAPHICS_LCD_H_

enum operation
{
    CLEAR,
    SET,
    INVERT
};

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void clearGraphicsLcd();
void initGraphicsLcd();
void drawGraphicsLcdPixel(uint8_t x, uint8_t y, enum operation op);
void drawGraphicsLcdRectangle(uint8_t xul, uint8_t yul, uint8_t dx, uint8_t dy, enum operation op);
void setGraphicsLcdTextPosition(uint8_t x, uint8_t page);
void putcGraphicsLcd(char c);
void putsGraphicsLcd(char str[]);

#endif

