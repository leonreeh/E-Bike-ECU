#include "liquidcrystal_i2c.h"
#include <stdio.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t dpFunction;
uint8_t dpControl;
uint8_t dpMode;
uint8_t dpRows;
uint8_t dpBacklight;

static void SendCommand(uint8_t);
static void SendChar(uint8_t);
static void Send(uint8_t, uint8_t);
static void Write4Bits(uint8_t);
static void ExpanderWrite(uint8_t);
static void PulseEnable(uint8_t);
static void DelayInit(void);
static void DelayUS(uint32_t);

uint8_t special1[8] = {
        0b00000,
        0b11001,
        0b11011,
        0b00110,
        0b01100,
        0b11011,
        0b10011,
        0b00000
};

uint8_t special2[8] = {
        0b11000,
        0b11000,
        0b00110,
        0b01001,
        0b01000,
        0b01001,
        0b00110,
        0b00000
};

void HD44780_Init(uint8_t rows)
{
  dpRows = rows;

  dpBacklight = LCD_BACKLIGHT;

  dpFunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;

  if (dpRows > 1)
  {
    dpFunction |= LCD_2LINE;
  }
  else
  {
    dpFunction |= LCD_5x10DOTS;
  }

  /* Wait for initialization */
  DelayInit();
  HAL_Delay(50);

  ExpanderWrite(dpBacklight);
  HAL_Delay(1000);

  /* 4bit Mode */
  Write4Bits(0x03 << 4);
  DelayUS(4500);

  Write4Bits(0x03 << 4);
  DelayUS(4500);

  Write4Bits(0x03 << 4);
  DelayUS(4500);

  Write4Bits(0x02 << 4);
  DelayUS(100);

  /* Display Control */
  SendCommand(LCD_FUNCTIONSET | dpFunction);

  dpControl = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
  HD44780_Display();
  HD44780_Clear();

  /* Display Mode */
  dpMode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
  DelayUS(4500);

  HD44780_CreateSpecialChar(0, special1);
  HD44780_CreateSpecialChar(1, special2);

  HD44780_Home();
}

void HD44780_Clear()
{
  SendCommand(LCD_CLEARDISPLAY);
  DelayUS(2000);
}

void HD44780_Home()
{
  SendCommand(LCD_RETURNHOME);
  DelayUS(2000);
}

/**
  * @brief Sets the cursor position on the HD44780 LCD display
  * @param col    = Column position (0-based)
  * @param row    = Row position (0-based)
  * @retval void
  */
void HD44780_SetCursor(uint8_t col, uint8_t row)
{
  int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
  if (row >= dpRows)
  {
    row = dpRows-1;
  }
  SendCommand(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void HD44780_NoDisplay()
{
  dpControl &= ~LCD_DISPLAYON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Display()
{
  dpControl |= LCD_DISPLAYON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_NoCursor()
{
  dpControl &= ~LCD_CURSORON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Cursor()
{
  dpControl |= LCD_CURSORON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_NoBlink()
{
  dpControl &= ~LCD_BLINKON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_Blink()
{
  dpControl |= LCD_BLINKON;
  SendCommand(LCD_DISPLAYCONTROL | dpControl);
}

void HD44780_ScrollDisplayLeft(void)
{
  SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}

void HD44780_ScrollDisplayRight(void)
{
  SendCommand(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

void HD44780_LeftToRight(void)
{
  dpMode |= LCD_ENTRYLEFT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_RightToLeft(void)
{
  dpMode &= ~LCD_ENTRYLEFT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_AutoScroll(void)
{
  dpMode |= LCD_ENTRYSHIFTINCREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_NoAutoScroll(void)
{
  dpMode &= ~LCD_ENTRYSHIFTINCREMENT;
  SendCommand(LCD_ENTRYMODESET | dpMode);
}

void HD44780_CreateSpecialChar(uint8_t location, uint8_t charmap[])
{
  location &= 0x7;
  SendCommand(LCD_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++)
  {
    SendChar(charmap[i]);
  }
}

void HD44780_PrintSpecialChar(uint8_t index)
{
  SendChar(index);
}

void HD44780_LoadCustomCharacter(uint8_t char_num, uint8_t *rows)
{
  HD44780_CreateSpecialChar(char_num, rows);
}

/**
  * @brief Prints a string to the HD44780 LCD display
  * @param c    = Pointer to a null-terminated string to display
  * @retval void
  */
void HD44780_PrintStr(const char c[])
{
  while(*c) SendChar(*c++);
}

void HD44780_SetBacklight(uint8_t new_val)
{
  if(new_val) HD44780_Backlight();
  else HD44780_NoBacklight();
}

void HD44780_NoBacklight(void)
{
  dpBacklight=LCD_NOBACKLIGHT;
  ExpanderWrite(0);
}

void HD44780_Backlight(void)
{
  dpBacklight=LCD_BACKLIGHT;
  ExpanderWrite(0);
}

static void SendCommand(uint8_t cmd)
{
  Send(cmd, 0);
}

static void SendChar(uint8_t ch)
{
  Send(ch, RS);
}

static void Send(uint8_t value, uint8_t mode)
{
  uint8_t highnib = value & 0xF0;
  uint8_t lownib = (value<<4) & 0xF0;
  Write4Bits((highnib)|mode);
  Write4Bits((lownib)|mode);
}

static void Write4Bits(uint8_t value)
{
  ExpanderWrite(value);
  PulseEnable(value);
}

static void ExpanderWrite(uint8_t _data)
{
  uint8_t data = _data | dpBacklight;
  HAL_I2C_Master_Transmit(&hi2c1, DEVICE_ADDR, (uint8_t*)&data, 1, 10);
}

static void PulseEnable(uint8_t _data)
{
  ExpanderWrite(_data | ENABLE);
  DelayUS(20);

  ExpanderWrite(_data & ~ENABLE);
  DelayUS(20);
}

static void DelayInit(void)
{
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;

  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  DWT->CYCCNT = 0;

  /* 3 NO OPERATION instructions */
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
}

static void DelayUS(uint32_t us) {
  uint32_t cycles = (SystemCoreClock/1000000L)*us;
  uint32_t start = DWT->CYCCNT;
  volatile uint32_t cnt;

  do
  {
    cnt = DWT->CYCCNT - start;
  } while(cnt < cycles);
}

/**
  * @brief Initializes the lcd_ar struct with default cursor positions and display values
  * @param lcd    = Pointer to an lcd_ar struct to initialize
  * @retval void
  */
void Init_lcd_ar(lcd_ar* lcd){
	//Load cursor positions
	lcd ->cur_volt[0] =15;
	lcd ->cur_volt[1] =0;

	lcd ->cur_temp[0]=0;
	lcd ->cur_temp[1]=0;

	lcd ->cur_amp[0] =15;
	lcd ->cur_amp[1] =1;

	lcd ->cur_speed[0]=9;
	lcd ->cur_speed[1]=3;

	lcd ->cur_erpm[0]=0;
	lcd ->cur_erpm[1]=3;

	lcd ->cur_pwm[0]=1;
	lcd ->cur_pwm[1]=2;

	//Set up Display for array mode
	//Init Volt
	HD44780_SetCursor(15,0);
	HD44780_PrintStr("42.0V");
	//Init Temp
	HD44780_SetCursor(0,0);
	HD44780_PrintStr("069C");
	//Init Current
	HD44780_SetCursor(15,1);
	HD44780_PrintStr("02.5A");
	//Init Speed
	HD44780_SetCursor(9,3);
	HD44780_PrintStr("10KM/H");
	//Init ERPM
	HD44780_SetCursor(0,3);
	HD44780_PrintStr("4000");
	//Iinit PWM
	//HD44780_SetCursor(1,2);
	//HD44780_PrintStr("00%");
}

/**
  * @brief Updates the lcd_ar struct and LCD display with new sensor values
  * @param ar     = Pointer to an lcd_ar struct containing cursor positions
  * @param val    = Array of float values to update [Voltage, Current, Temperature, Speed, (optional: PWM)]
  * @retval void
  */
void update_lcd_val(lcd_ar* ar, float val[]){
	//Set Voltage
	snprintf(ar->volt, 5, "%04.1f", val[0]);
	HD44780_SetCursor(ar->cur_volt[0],ar->cur_volt[1]);
	HD44780_PrintStr(ar->volt);
	//Set Current
	snprintf(ar->amp, 5, "%04.1f", val[1]);
	HD44780_SetCursor(ar->cur_amp[0],ar->cur_amp[1]);
	HD44780_PrintStr(ar->amp);
	//Set Temp
	snprintf(ar->temp, 4, "%03.0f", val[2]);
	HD44780_SetCursor(ar->cur_temp[0],ar->cur_temp[1]);
	HD44780_PrintStr(ar->temp);
	//Set Speed
	snprintf(ar->speed, 3, "%02.0f",val[3] );
	HD44780_SetCursor(ar->cur_speed[0],ar->cur_speed[1]);
	HD44780_PrintStr(ar->speed);
	//Set ERPM
	snprintf(ar->erpm, 5, "%04.0f",rpm);
	HD44780_SetCursor(ar->cur_erpm[0],ar->cur_erpm[1]);
	HD44780_PrintStr(ar->erpm);
	//Set PWm
	//snprintf(ar->pwm,3,"%02.0f",val[4] );
	//HD44780_SetCursor(ar->cur_pwm[0],ar->cur_pwm[1]);
	//HD44780_PrintStr(ar->pwm);

}

