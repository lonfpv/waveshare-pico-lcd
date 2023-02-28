//
// LCD_Api.c
//
// SPDX-License-Identifier: MIT
//
// Title: Waveshare LCD API
//
// Abstract:
// API for the Waveshare 1.3" LCD.  Based on Waveshare's original
// LCN_1in3.c.

#include "LCD_Api.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Constants
 */
#define MAX_NUMERIC_STRING_LENGTH 255


/*
 * Typedef: LCD_Attributes
 *  Store LCD-specific attributes.
 */
typedef struct {
    UWORD        displayWidth;
    UWORD        displayHeight;
    UWORD        memoryWidth;
    UWORD        memoryHeight;
    UBYTE        memoryAccessRegister;
    UWORD        byteWidth;
    UWORD        byteHeight;
    LCD_Rotation rotation;
    LCD_Flip     flip;
    UWORD        scale;
} LCD_Attributes;


/*
 * Static variables.
 */
LCD_Attributes lcdAttributes;


/*
 * Static prototypes.
 */
static void LCD_Reset(void);
static void LCD_SendCommand(UBYTE Reg);
static void LCD_SendData_8Bit(UBYTE Data);
static void LCD_SendData_16Bit(UWORD Data);
static void LCD_InitRegisters(void);
static void LCD_SetAttributes(UWORD displayWidth, UWORD displayHeight, UWORD bytesPerPixel, 
        LCD_Rotation rotation, LCD_Flip flip);


/*
 * Function: LCD_Init()
 *  Initialize the LCD.  Call this function first for the LCD API.
 */
void LCD_Init(UWORD pixelWidth, UWORD pixelHeight, UWORD bytesPerPixel, LCD_Rotation rotation, LCD_Flip flip)
{
    DEV_SET_PWM(90);

    // Reset LCD hardware
    LCD_Reset();

    // Set the LCD resolution and scanning method
    LCD_SetAttributes(pixelWidth, pixelHeight, bytesPerPixel, rotation, flip);
    
    // Initialization the LCD configuration registers
    LCD_InitRegisters();
}


/*
 * Function: LCD_Reset()
 *  Reset the LCD using the reset input.
 */
static void LCD_Reset(void)
{
    DEV_Digital_Write(LCD_RST_PIN, 1);
    DEV_Delay_ms(100);
    DEV_Digital_Write(LCD_RST_PIN, 0);
    DEV_Delay_ms(100);
    DEV_Digital_Write(LCD_RST_PIN, 1);
    DEV_Delay_ms(100);
}


/*
 * Function: LCD_SendCommand()
 *  Send a command to the LCD.
 */
static void LCD_SendCommand(UBYTE Reg)
{
    DEV_Digital_Write(LCD_DC_PIN, 0);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(LCD_CS_PIN, 1);
}


/*
 * Function: LCD_SendData_8Bit()
 *  Send an 8-bit value to the LCD.
 * 
 * Parameters:
 *  Data -- 8-bit data to send
 */
static void LCD_SendData_8Bit(UBYTE Data)
{
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(LCD_CS_PIN, 1);
}


/*
 * Function: LCD_SendData_16Bit()
 *  Send a 16-bit value to the LCD
 * 
 * Parameters:
 *  Data -- 16-bit data to send
 */
static void LCD_SendData_16Bit(UWORD Data)
{
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    DEV_SPI_WriteByte((Data >> 8) & 0xFF);
    DEV_SPI_WriteByte(Data & 0xFF);
    DEV_Digital_Write(LCD_CS_PIN, 1);
}


/*
 * Function: LCD_InitRegisters()
 *  Initialize the LCD configuration registers.
 */
static void LCD_InitRegisters(void)
{
    LCD_SendCommand(0x3A);
    LCD_SendData_8Bit(0x05);

    LCD_SendCommand(0xB2);
    LCD_SendData_8Bit(0x0C);
    LCD_SendData_8Bit(0x0C);
    LCD_SendData_8Bit(0x00);
    LCD_SendData_8Bit(0x33);
    LCD_SendData_8Bit(0x33);

    LCD_SendCommand(0xB7);  // Gate control
    LCD_SendData_8Bit(0x35);

    LCD_SendCommand(0xBB);  // VCOM setting
    LCD_SendData_8Bit(0x19);

    LCD_SendCommand(0xC0);  // LCM control     
    LCD_SendData_8Bit(0x2C);

    LCD_SendCommand(0xC2);  // VDV and VRH command enable
    LCD_SendData_8Bit(0x01);
    LCD_SendCommand(0xC3);  // VRH set
    LCD_SendData_8Bit(0x12);
    LCD_SendCommand(0xC4);  // VDV set
    LCD_SendData_8Bit(0x20);

    LCD_SendCommand(0xC6);  // Frame rate control in normal mode
    LCD_SendData_8Bit(0x0F);
    
    LCD_SendCommand(0xD0);  // Power control 1
    LCD_SendData_8Bit(0xA4);
    LCD_SendData_8Bit(0xA1);

    LCD_SendCommand(0xE0);  // Positive voltage gamma control
    LCD_SendData_8Bit(0xD0);
    LCD_SendData_8Bit(0x04);
    LCD_SendData_8Bit(0x0D);
    LCD_SendData_8Bit(0x11);
    LCD_SendData_8Bit(0x13);
    LCD_SendData_8Bit(0x2B);
    LCD_SendData_8Bit(0x3F);
    LCD_SendData_8Bit(0x54);
    LCD_SendData_8Bit(0x4C);
    LCD_SendData_8Bit(0x18);
    LCD_SendData_8Bit(0x0D);
    LCD_SendData_8Bit(0x0B);
    LCD_SendData_8Bit(0x1F);
    LCD_SendData_8Bit(0x23);

    LCD_SendCommand(0xE1);  // Negative voltage gamma control
    LCD_SendData_8Bit(0xD0);
    LCD_SendData_8Bit(0x04);
    LCD_SendData_8Bit(0x0C);
    LCD_SendData_8Bit(0x11);
    LCD_SendData_8Bit(0x13);
    LCD_SendData_8Bit(0x2C);
    LCD_SendData_8Bit(0x3F);
    LCD_SendData_8Bit(0x44);
    LCD_SendData_8Bit(0x51);
    LCD_SendData_8Bit(0x2F);
    LCD_SendData_8Bit(0x1F);
    LCD_SendData_8Bit(0x1F);
    LCD_SendData_8Bit(0x20);
    LCD_SendData_8Bit(0x23);

    LCD_SendCommand(0x21);  // Display inversion on

    LCD_SendCommand(0x11);  // Sleep out

    LCD_SendCommand(0x29);  // Display on
}

/*
 * Function: LCD_SetAttributes()
 *  Set LCD attributes.
 * 
 * Parameters:
 *  displayWidth -- LCD viewport width
 *  displayHeight -- LCD viewport height
 *  bytesPerPixel -- number of bytes per LCD pixel
 *  rotation -- viewport angle relative to physical LCD
 *  flip -- viewport flip (mirror) axis or axes
 */
static void LCD_SetAttributes(UWORD displayWidth, UWORD displayHeight, UWORD bytesPerPixel,
        LCD_Rotation rotation, LCD_Flip flip)
{
    // Set general attributes
    lcdAttributes.rotation = rotation;
    lcdAttributes.flip = flip;
    lcdAttributes.scale = 65;
    lcdAttributes.displayWidth = displayWidth;
    lcdAttributes.displayHeight = displayHeight;

    //Get GRAM and LCD width and height
    if (rotation == ROTATION_0 || rotation == ROTATION_180) {
        lcdAttributes.memoryHeight = displayWidth;
        lcdAttributes.memoryWidth  = displayHeight;
        lcdAttributes.memoryAccessRegister = 0X70;
    } else {
        lcdAttributes.memoryHeight = displayHeight;
        lcdAttributes.memoryWidth  = displayWidth;
        lcdAttributes.memoryAccessRegister = 0X00;
    }

    lcdAttributes.byteWidth = lcdAttributes.memoryWidth * bytesPerPixel;
    lcdAttributes.byteHeight = lcdAttributes.memoryHeight;

    // Set the read / write scan direction of the frame memory
    LCD_SendCommand(0x36); //MX, MY, RGB mode
    LCD_SendData_8Bit(lcdAttributes.memoryAccessRegister);	//0x08 set RGB
}


/*
 * Function: LCD_SetWindow()
 *  Specify a window into the LCD display image memory.
 *
 * Parameters:
 *  Xstart -- starting X-coordinate (left column)
 *  Ystart -- starting Y-coordinate (top row)
 *  Xend -- ending X-coordintate (right column)
 *  Yend -- ending Y-coordinate (bottom row)
 */
void LCD_SetWindow(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend)
{
    // Set the X coordinate
    LCD_SendCommand(0x2A);
    LCD_SendData_8Bit(0x00);
    LCD_SendData_8Bit(Xstart);
	LCD_SendData_8Bit(0x00);
    LCD_SendData_8Bit(Xend-1);

    // Set the Y coordinate
    LCD_SendCommand(0x2B);
    LCD_SendData_8Bit(0x00);
	LCD_SendData_8Bit(Ystart);
	LCD_SendData_8Bit(0x00);
    LCD_SendData_8Bit(Yend-1);

    LCD_SendCommand(0X2C);
}


/*
 * Function: LCD_Clear()
 *  Set all pixels on the LCD to a specified color.
 * 
 * Parameter:
 *  color -- color for all pixels
 */
void LCD_Erase(UWORD color)
{
    UWORD *blankRow;
    UWORD row, column;

    // Create row frame buffer
    blankRow = (UWORD *) malloc(480);

    // Change color from little-endian to big-endian
    color = (color << 8) | (color >> 8);

    // Set all pixels in the frame buffer to the specified color
    for (column = 0; column < lcdAttributes.memoryWidth; column++) {
        blankRow[column] = color;
    }
    
    // Copy row frame buffer to the LCD one row at a time
    LCD_SetWindow(0, 0, lcdAttributes.memoryWidth, lcdAttributes.memoryHeight);
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    for (row = 0; row < lcdAttributes.memoryHeight; row++){
        DEV_SPI_Write_nByte((uint8_t *) blankRow, lcdAttributes.byteWidth);
    }
    DEV_Digital_Write(LCD_CS_PIN, 1);
}


/*
 * Function: LCD_DisplayImage()
 *  Display an image in a specified window on the LCD
 * 
 * Parameters:
 *  Xstart -- starting X-coordinate (left column)
 *  Ystart -- starting Y-coordinate (top row)
 *  Xend -- ending X-coordintate (right column)
 *  Yend -- ending Y-coordinate (bottom row)
 */
void LCD_DisplayImage(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend, UWORD *Image)
{
    UDOUBLE Addr = 0;
    UWORD row;

    LCD_SetWindow(Xstart, Ystart, Xend , Yend);
    DEV_Digital_Write(LCD_DC_PIN, 1);
    DEV_Digital_Write(LCD_CS_PIN, 0);
    for (row = Ystart; row < Yend - 1; row++) {
        Addr = Xstart + row * lcdAttributes.memoryWidth;
        DEV_SPI_Write_nByte((uint8_t *)&Image[Addr], (Xend-Xstart)*2);
    }
    DEV_Digital_Write(LCD_CS_PIN, 1);
}


/*
 * Function: LCD_DisplayPoint()
 *  Display a point on the LCD
 *
 * Parameters:
 *  X -- X (column) coordinate
 *  Y -- Y (row) coordinate
 */
void LCD_DisplayPoint(UWORD X, UWORD Y, UWORD Color)
{
    LCD_SetWindow(X, Y, X, Y);
    LCD_SendData_16Bit(Color);
}


/*
 * Function: LCD_DrawChar()
 *  Display an ASCII character on the LCD.
 * 
 * Parameters:
 *  startColumn -- starting X-coordinate (left column)
 *  startRow -- starting Y-coordinate (top row)
 *  character -- character to display
 *  pFont -- character font
 *  foregroundColor -- foreground color
 *  backgroundColor -- background color
 */
void LCD_DrawChar(UWORD startColumn, UWORD startRow, const char character,
                    sFONT *pFont, UWORD foregroundColor, UWORD backgroundColor)
{
    UWORD rowOffset, columnOffset;

    if (startColumn > lcdAttributes.memoryWidth || startRow > lcdAttributes.memoryHeight) {
//      Debug("LCD_DrawChar input exceeds the normal display range\r\n");
        return;
    }

    uint32_t charIndex = (character - ' ') * pFont->Height * (pFont->Width / 8 + (pFont->Width % 8 ? 1 : 0));
    const unsigned char *pCharImage = &pFont->table[charIndex];

    for (rowOffset = 0; rowOffset < pFont->Height; rowOffset++) {
        for (columnOffset = 0; columnOffset < pFont->Width; columnOffset++) {
            if (*pCharImage & (0x80 >> (columnOffset % 8))) {
                LCD_DisplayPoint(startColumn + columnOffset, startRow + rowOffset, backgroundColor);
            } else {
                LCD_DisplayPoint(startColumn + columnOffset, startRow + rowOffset, foregroundColor);
            }
            if (columnOffset % 8 == 7)
                pCharImage++;
        }
        if (pFont->Width % 8 != 0)
            pCharImage++;
    } 
}


/*
 * Function: LCD_DrawString()
 *  Display a string on the LCD.
 * 
 * Parameters:
 *  startColumn -- starting X-coordinate (left column)
 *  startRow -- starting Y-coordinate (top row)
 *  pString -- string to display
 *  pFont -- character font
 *  foregroundColor -- foreground color
 *  backgroundColor -- background color
 */
void LCD_DrawString(UWORD startColumn, UWORD startRow, const char *pString,
                         sFONT *pFont, UWORD foregroundColor, UWORD backgroundColor)
{
    UWORD currentColumn = startColumn;
    UWORD currentRow = startRow;

    if (startColumn > lcdAttributes.memoryWidth || startRow > lcdAttributes.memoryHeight) {
//      Debug("Paint_DrawString_EN Input exceeds the normal display range\r\n");
        return;
    }

    while (*pString != '\0') {
        // If the current row is filled, reposition to (startColumn, nextRow)
        // nextRow is the current row plus the character height
        if ((currentColumn + pFont->Width) > lcdAttributes.memoryWidth) {
            currentColumn = startColumn;
            currentRow += pFont->Height;
        }

        // If all rows are full, start over
        if ((currentRow + pFont->Height) > lcdAttributes.memoryHeight) {
            currentColumn = startColumn;
            currentRow = startRow;
        }
        LCD_DrawChar(currentColumn, currentRow, *pString, pFont, backgroundColor, foregroundColor);

        // Increment string pointer to The next character
        pString++;

        // Increment X coordinate (column) by the width of a character
        currentColumn += pFont->Width;
    }
}


/*
 * Function: LCD_DrawNumber()
 *  Display a number on the LCD.
 * 
 * Parameters:
 *  startColumn -- starting X-coordinate (left column)
 *  startRow -- starting Y-coordinate (top row)
 *  value -- floating point value to display
 *  pFont -- character font
 *  foregroundColor -- foreground color
 *  backgroundColor -- background color
 */
void LCD_DrawNumber(UWORD startColumn, UWORD startRow, double value, UWORD minWidth, UWORD precision, 
                         const char *units, sFONT *pFont, UWORD foregroundColor, UWORD backgroundColor)
{
    UWORD currentColumn = startColumn;
    UWORD currentRow = startRow;
    char numericString[MAX_NUMERIC_STRING_LENGTH];

    sprintf(numericString, "%*.*lf", minWidth, precision, value);
    if (units) {
        strcat(numericString, units);
    }
    LCD_DrawString(startColumn, startRow, numericString, pFont, foregroundColor, backgroundColor);
}


#ifdef __cplusplus
}
#endif
