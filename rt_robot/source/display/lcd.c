/*
 * lcd.c
 *
 *  Created on: 15 de jul de 2019
 *      Author: Matheus_Pinto
 */

#include "lcd.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "delayer/delayer.h"



static uint8_t _displayfunction;
static uint8_t _displaycontrol;
static uint8_t _displaymode;
static uint8_t _numlines;
static uint8_t _row_offsets[4];
static lcdBus_t _lcdBus;


/* waiting macros */
#define Waitns(x) \
        delayer_Waitns(x)                 /* Wait x ns */
#define Waitus(x) \
        delayer_Waitus(x)                 /* Wait x us */
#define Waitms(x) \
        delayer_Waitms(x)                 /* Wait x ms */

/* macros for the RS pin */
#define ClrRS() \
        GPIO_WritePinOutput(_lcdBus.rs.base, _lcdBus.rs.pin, 0)                    /* RS=0: command mode */
#define SetRS() \
        GPIO_WritePinOutput(_lcdBus.rs.base, _lcdBus.rs.pin, 1)                    /* RS=1: data mode */

/* macros for the EN pin */
#define ClrEN() \
        GPIO_WritePinOutput(_lcdBus.en.base, _lcdBus.en.pin, 0)                    /* EN=0 */
#define SetEN() \
		GPIO_WritePinOutput(_lcdBus.en.base, _lcdBus.en.pin, 1)                    /* EN=1 */


/* Internal method prototypes */
static void EnablePulse(void);

static void WriteBits(uint8_t value);

static void SetRowOffsets(int row0, int row1, int row2, int row3);

/*
** ===================================================================
**     Method      :  EnablePulse (component LCDHTA)
**
**     Description :
**         Creates a pulse to transfer data and/or commands
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
static void EnablePulse(void)
{
	ClrEN();
	Waitus(1);
	SetEN();
	Waitus(1);    // enable pulse must be >450ns
	ClrEN();
	Waitus(100);   // commands need > 37us to settle
}


static void SetRowOffsets(int row0, int row1, int row2, int row3)
{
  _row_offsets[0] = row0;
  _row_offsets[1] = row1;
  _row_offsets[2] = row2;
  _row_offsets[3] = row3;
}


void lcd_Init(lcdConfig_t *config)
{
#ifdef LCD_4BITMODE
	_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	 for (int i = 0; i < 4; i++)
	 {
		 _lcdBus.data[i].base = config->bus.data[i].base;
		 _lcdBus.data[i].pin = config->bus.data[i].pin;
	 }
#else
	_displayfunction = LCD_8BITMODE | LCD_1LINE | LCD_5x8DOTS;
	 for (int i = 0; i < 8; i++)
	 {
		 _lcdBus.data[i].base = config->bus.data[i].base;
		 _lcdBus.data[i].pin = config->bus.data[i].pin;
	 }
#endif
	_lcdBus.rs.base = config->bus.rs.base;
	_lcdBus.rs.pin = config->bus.rs.pin;
	_lcdBus.en.base = config->bus.en.base;
	_lcdBus.en.pin = config->bus.en.pin;

    _numlines = config->lines;

    if (_numlines > 1) {
      _displayfunction |= LCD_2LINE;
    }

    SetRowOffsets(0x00, 0x40, 0x00 + config->cols, 0x40 + config->cols);

    // for some 1 line displays you can select a 10 pixel high font
    if ((config->charsize != LCD_5x8DOTS) && (config->lines == 1)) {
      _displayfunction |= LCD_5x10DOTS;
    }

    // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
    // according to datasheet, we need at least 40ms after power rises above 2.7V
    // before sending commands. Arduino can turn on way before 4.5V so we'll wait 50
    delayer_Waitms(60);
    ClrRS();
    ClrEN();

    //put the LCD into 4 bit or 8 bit mode
#ifdef LCD_4BITMODE
    // this is according to the hitachi HD44780 datasheet
    // figure 24, pg 46
    // we start in 8bit mode, try to set 4 bit mode
    WriteBits(0x03);
    Waitus(5000); // wait min 4.1ms
    // second try
    WriteBits(0x03);
    Waitus(5000); // wait min 4.1ms
    // third go!
    WriteBits(0x03);
    Waitus(200);
    // finally, set to 4-bit interface
    WriteBits(0x02);
#else
    // this is according to the hitachi HD44780 datasheet
    // page 45 figure 23

    // Send function set command sequence
    lcd_Command(LCD_FUNCTIONSET | _displayfunction);
    Waitus(4500); // wait min 4.1ms

    // second try
    lcd_Command(LCD_FUNCTIONSET | _displayfunction);
    Waitus(150);

    // third go
    lcd_Command(LCD_FUNCTIONSET | _displayfunction);
#endif
    // finally, set # lines, font size, etc.
    lcd_Command(LCD_FUNCTIONSET | _displayfunction);

    // turn the display on with no cursor or blinking default
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcd_Display();

    // clear it off
    lcd_Clear();

    // Initialize to default text direction (for romance languages)
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    // set the entry mode
    lcd_Command(LCD_ENTRYMODESET | _displaymode);

    Waitus(500);
}


/********** high level commands, for the user! */
void lcd_Clear()
{
  lcd_Command(LCD_CLEARDISPLAY);  // clear display, set cursor position to zero
  Waitus(5000);  // this command takes a long time!
}

void lcd_Home()
{
  lcd_Command(LCD_RETURNHOME);  // set cursor position to zero
  Waitus(2000);  // this command takes a long time!
}

void lcd_SetCursor(uint8_t col, uint8_t row)
{
  const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
  if( row >= max_lines )
  {
    row = max_lines - 1;    // we count rows starting w/0
  }
  if( row >= _numlines )
  {
    row = _numlines - 1;    // we count rows starting w/0
  }

  lcd_Command(LCD_SETDDRAMADDR | (col + _row_offsets[row]));
}


// Turn the display on/off (quickly)
void lcd_NoDisplay()
{
  _displaycontrol &= ~LCD_DISPLAYON;
  lcd_Command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcd_Display()
{
  _displaycontrol |= LCD_DISPLAYON;
  lcd_Command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void lcd_NoCursor()
{
  _displaycontrol &= ~LCD_CURSORON;
  lcd_Command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcd_Cursor()
{
  _displaycontrol |= LCD_CURSORON;
  lcd_Command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void lcd_NoBlink()
{
  _displaycontrol &= ~LCD_BLINKON;
  lcd_Command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void lcd_Blink()
{
  _displaycontrol |= LCD_BLINKON;
  lcd_Command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void lcd_ScrollDisplayLeft(void)
{
  lcd_Command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void lcd_ScrollDisplayRight(void)
{
  lcd_Command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void lcd_LeftToRight(void)
{
  _displaymode |= LCD_ENTRYLEFT;
  lcd_Command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void lcd_RightToLeft(void)
{
  _displaymode &= ~LCD_ENTRYLEFT;
  lcd_Command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void lcd_Autoscroll(void)
{
  _displaymode |= LCD_ENTRYSHIFTINCREMENT;
  lcd_Command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void lcd_NoAutoscroll(void)
{
  _displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
  lcd_Command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void lcd_CreateChar(uint8_t location, uint8_t charmap[])
{
  location &= 0x7; // we only have 8 locations 0-7
  lcd_Command(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    lcd_Write(charmap[i]);
  }
}


/*********** mid level commands, for sending data/cmds */

inline void lcd_Command(uint8_t value)
{
	ClrRS();
	Waitus(200);
#ifdef LCD_8BITMODE
    WriteBits(value);
#else
    WriteBits(value >> 4);
    WriteBits(value);
#endif
}

inline size_t lcd_Write(uint8_t value)
{
	SetRS();
	Waitus(200);
#ifdef LCD_8BITMODE
    WriteBits(value);
#else
    WriteBits(value >> 4);
    WriteBits(value);
#endif
    return 1; // assume sucess
}


/************ low level data pushing commands **********/

static void WriteBits(uint8_t value)
{
#ifdef LCD_4BITMODE
	uint8_t count = 4;
#else
	uint8_t count = 8;
#endif
  for (int i = 0; i < count; i++)
  {
 	GPIO_WritePinOutput(_lcdBus.data[i].base, _lcdBus.data[i].pin, (value >> i) & 0x01);
  }

  delayer_Waitus(10);
  EnablePulse();
}

void lcd_WriteString(char *str)
{
  while (*str != '\0'){
    lcd_Write(*str);
    str++;
  }
}

void lcd_WriteLn(void)
{
	lcd_Command(LCD_SECOND_LINE_CMD);
}
