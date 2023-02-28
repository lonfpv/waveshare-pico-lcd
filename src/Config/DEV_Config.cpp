/*****************************************************************************
* | File      	:   DEV_Config.cpp
* | Author      :   
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2021-03-16
* | Info        :   
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of theex Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
******************************************************************************/

#include <Arduino.h>
// #include <SPI.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>
// #include <Wire.h>
#include "DEV_Config.h"

#define SPI_PORT spi1
#define I2C_PORT i2c0

// Default Wire (I2C) object: I2C0 SDA = GP4, I2C1 SCL = GP5, I2C0 peripheral
#define WIRE1_SDA       2  // Use GP2 as I2C1 SDA
#define WIRE1_SCL       3  // Use GP3 as I2C1 SCL
// arduino::MbedI2C Wire1(WIRE1_SDA, WIRE1_SCL);

// Default Serial1 object: UART0, TX = GP0, RX = GP1
#define SERIAL2_TX      4  // Use GP4 as UART1 TX
#define SERIAL2_RX      5  // Use GP5 as UART1 RX
// arduino::UART Serial2(SERIAL2_TX, SERIAL2_RX, NC, NC);

// Default SPI at MISO = GP16, SS = GP17, SCLK = GP18, MOSI = GP19
// SS/CS is software controlled, doesn't matter which pin
#define SPI1_MISO 12
#define SPI1_MOSI 11
#define SPI1_SCLK 10
// arduino::MbedSPI SPI1(SPI1_MISO, SPI1_MOSI, SPI1_SCLK);

uint slice_num;


/*
 * GPIO read and write
 */
void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
//  gpio_put(Pin, Value);
    digitalWrite(Pin, Value ? HIGH : LOW);
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
//  return gpio_get(Pin);
    return digitalRead(Pin);
}


/*
 * SPI
 */
void DEV_SPI_WriteByte(uint8_t Value)
{
    spi_write_blocking(SPI_PORT, &Value, 1);
}

void DEV_SPI_Write_nByte(uint8_t pData[], uint32_t Len)
{
    spi_write_blocking(SPI_PORT, pData, Len);
}


/*
 * I2C
 */
void DEV_I2C_Write(uint8_t addr, uint8_t reg, uint8_t Value)
{
    uint8_t data[2] = {reg, Value};
    i2c_write_blocking(I2C_PORT, addr, data, 2, false);
}

void DEV_I2C_Write_nByte(uint8_t addr, uint8_t *pData, uint32_t Len)
{
    i2c_write_blocking(I2C_PORT, addr, pData, Len, false);
}

uint8_t DEV_I2C_ReadByte(uint8_t addr, uint8_t reg)
{
    uint8_t buf;
    i2c_write_blocking(I2C_PORT,addr,&reg,1,true);
    i2c_read_blocking(I2C_PORT,addr,&buf,1,false);
    return buf;
}


/*
 * GPIO Mode
 */
void DEV_GPIO_Mode(UWORD Pin, UWORD Mode)
{
//  _gpio_init(Pin);
    if(Mode == 0 /* || Mode == 0 */ /* GPIO_IN */) {
//      gpio_set_dir(Pin, GPIO_IN);
        pinMode(Pin, INPUT);
    } else {
//      gpio_set_dir(Pin, GPIO_OUT);
        pinMode(Pin, OUTPUT);
    }
}


/*
 * KEY Config
 */
void DEV_KEY_Config(UWORD Pin)
{
//  gpio_init()
//  gpio_init(Pin);
//	gpio_pull_up(Pin);
//  gpio_set_dir(Pin, GPIO_IN);
    pinMode(Pin, INPUT_PULLUP);
}

/*
 * Millisecond delay
 */
void DEV_Delay_ms(UDOUBLE xms)
{
    sleep_ms(xms);
}


/*
 * Microsecond delay
 */
void DEV_Delay_us(UDOUBLE xus)
{
    sleep_us(xus);
}


/*
 * GPIO initialization
 */
void DEV_GPIO_Init(void)
{
    DEV_GPIO_Mode(LCD_RST_PIN, 1);
    DEV_GPIO_Mode(LCD_DC_PIN, 1);
    DEV_GPIO_Mode(LCD_CS_PIN, 1);
    DEV_GPIO_Mode(LCD_BL_PIN, 1);
//  DEV_GPIO_Mode(LCD_TEST_PIN, 1); // RB: added for hardware debugging
    
    DEV_GPIO_Mode(LCD_CS_PIN, 1);
    DEV_GPIO_Mode(LCD_BL_PIN, 1);

    DEV_Digital_Write(LCD_CS_PIN, 1);
    DEV_Digital_Write(LCD_DC_PIN, 0);
    DEV_Digital_Write(LCD_BL_PIN, 1);
//  DEV_Digital_Write(LCD_TEST_PIN, 1); // RB: added for hardware debugging
}


/*
 * Function: Dev_Module_Init()
 * Initialize Pico I/O pins.
 */
UBYTE DEV_Module_Init(void)
{
//  stdio_init_all();   

    // I2C Config
//  i2c_init(i2c1,300*1000);
//* gpio_set_function(SENSOR_SDA_PIN, GPIO_FUNC_I2C);
//* gpio_set_function(SENSOR_SCL_PIN, GPIO_FUNC_I2C);
//* gpio_pull_up(SENSOR_SDA_PIN);
//* gpio_pull_up(SENSOR_SCL_PIN);
//  Wire.begin();  // I2C0
//  Wire1.begin(); // I2C1

    // SCI (UART) Config
    Serial.begin(115200);  // USB CDC serial
    Serial1.begin(115200); // UART0 serial at TX=GP0, RX=GP1
//  Serial2.begin(115200); // UART1 serial at TX=GP4, RX=GP5

    // SPI Config
//  spi_init(SPI_PORT, 10000 * 1000);
    _spi_init(SPI_PORT, 5000 * 1000);
//  SPI.begin();  // SPI0
// TBD -- SPI1.begin(); // SPI1 

    gpio_set_function(LCD_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(LCD_MOSI_PIN, GPIO_FUNC_SPI);
//  gpio_set_function(p10,GPIO_FUNC_SPI);
//  gpio_set_function(p11,GPIO_FUNC_SPI);
//  gpio_set_function(p12,GPIO_FUNC_SPI);

    // GPIO Config
    DEV_GPIO_Init();
    
    // PWM Config
    // gpio_set_function(LCD_BL_PIN, GPIO_FUNC_PWM);
    pinMode(LCD_BL_PIN, OUTPUT);
    // slice_num = pwm_gpio_to_slice_num(LCD_BL_PIN);
    // pwm_set_wrap(slice_num, 100);
    // pwm_set_chan_level(slice_num, PWM_CHAN_B, 1);
    // pwm_set_clkdiv(slice_num,50);
    // pwm_set_enabled(slice_num, true);
    analogWriteResolution(12);
    // PWM value can be 0 to 4095 -- set to 2047 for 50%
    analogWrite(LCD_BL_PIN, 2047);
    
    printf("DEV_Module_Init OK \r\n");
    return 0;
}


// Set PWM value from 0 to 100%
void DEV_SET_PWM(uint8_t Value){
//  if (Value > 100) {
//      printf("DEV_SET_PWM Error \r\n");
//  }else {
        // pwm_set_chan_level(slice_num, PWM_CHAN_B, Value);
        analogWrite(LCD_BL_PIN, min(41 * (int) Value, 4095));
//  }
}

/******************************************************************************
function:	Module exits, closes SPI and BCM2835 library
parameter:
Info:
******************************************************************************/
void DEV_Module_Exit(void)
{

}
