#ifndef __CONFIG_H__
#define __CONFIG_H__


/*
*   LCD GPIO Config
*/
// #define HW_LCD_USE_8080
// #define HW_LCD_USE_3SPI
#define HW_LCD_USE_4SPI

#define HW_LCD_RST  4

#define HW_LCD_TE  13
#define HW_LCD_CS  12
#define HW_LCD_DC  11
#define HW_LCD_MOSI     46
#define HW_LCD_MISO     3
#define HW_LCD_CLK      10

#define HW_LCD_WRX      HW_LCD_CLK
#define HW_LCD_RDX      9
#define HW_LCD_DB0      8
#define HW_LCD_DB1      18
#define HW_LCD_DB2      17
#define HW_LCD_DB3      16
#define HW_LCD_DB4      15
#define HW_LCD_DB5      7
#define HW_LCD_DB6      6
#define HW_LCD_DB7      5

#define HW_LCD_IM1      41
#define HW_LCD_IM2      40
#define HW_LCD_BKL      42

/*
*   I2C GPIO Config
*/
#define HW_I2C_SDA  14
#define HW_I2C_SCL  21

/*
*   GT911 GPIO Config
*/
#define HW_TP_RST   48
#define HW_TP_INT   47

/*
*   Haptic GPIO Config
*/
#define HW_HT_EN    38
#define HW_HT_TRIG  39



#endif
