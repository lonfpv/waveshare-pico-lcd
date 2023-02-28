//
// LCD_Api.h
//
// SPDX-License-Identifier: MIT
//
// Title: Waveshare LCD API
//
// Abstract:
// API for the Waveshare 1.3" LCD.  Based on Waveshare's original
// LCN_1in3.c.

#ifndef __LCD_API_H
#define __LCD_API_H

#include <stdio.h>
#include "DEV_Config.h"
#include "fonts.h"

#ifdef __cplusplus
extern "C" {
#endif


/*
 * Constants
 */

/* Colors */
#define WHITE          0xFFFF
#define BLACK          0x0000
#define BLUE           0x001F
#define BRED           0XF81F
#define GRED           0XFFE0
#define GBLUE          0X07FF
#define RED            0xF800
#define MAGENTA        0xF81F
#define GREEN          0x07E0
#define CYAN           0x7FFF
#define YELLOW         0xFFE0
#define BROWN          0XBC40
#define BRRED          0XFC07
#define GRAY           0X8430

/* Default colors */
#define IMAGE_BACKGROUND    BLACK
#define FONT_FOREGROUND     YELLOW
#define FONT_BACKGROUND     BLACK


/*
 * Typedef: LCD_Rotation
 *  LCD rotation.
 */
typedef enum {
    ROTATION_0,
    ROTATION_90,
    ROTATION_180,
    ROTATION_270
} LCD_Rotation;


/*
 * Typedef: LCD_Flip
 *  LCD flip axis or axes.
 */
typedef enum {
    FLIP_NONE,
    FLIP_HORIZONTAL,
    FLIP_VERTICAL,
    FLIP_BOTH
} LCD_Flip;


/*
 * Function: LCD_Init()
 *  Initialize the LCD.  Call this function first for the LCD API.
 */
extern void LCD_Init(UWORD pixelWidth, UWORD pixelHeight, UWORD bytesPerPixel, LCD_Rotation rotation, LCD_Flip flip);


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
extern void LCD_SetWindow(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend);


/*
 * Function: LCD_Erase()
 *  Set all pixels on the LCD to a specified color.
 * 
 * Parameter:
 *  color -- color for all pixels
 */
extern void LCD_Erase(UWORD color);


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
extern void LCD_DisplayImage(UWORD Xstart, UWORD Ystart, UWORD Xend, UWORD Yend, UWORD *Image);


/*
 * Function: LCD_DisplayPoint()
 *  Display a point on the LCD
 *
 * Parameters:
 *  X -- X (column) coordinate
 *  Y -- Y (row) coordinate
 */
extern void LCD_DisplayPoint(UWORD X, UWORD Y, UWORD Color);


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
extern void LCD_DrawChar(UWORD startColumn, UWORD startRow, const char character,
                        sFONT *pFont, UWORD foregroundColor, UWORD backgroundColor);


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
extern void LCD_DrawString(UWORD startColumn, UWORD startRow, const char *pString,
                        sFONT *pFont, UWORD foregroundColor, UWORD backgroundColor);


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
extern void LCD_DrawNumber(UWORD startColumn, UWORD startRow, double value, UWORD minWidth, UWORD precision, 
                        const char *units, sFONT *pFont, UWORD foregroundColor, UWORD backgroundColor);


#ifdef __cplusplus
}
#endif

#endif  // __LCD_API_H
