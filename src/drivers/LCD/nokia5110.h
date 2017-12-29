#ifndef _NOKIA5110_H_
#define _NOKIA5110_H_

//#include <hidef.h>      /* common defines and macros */
//#include "derivative.h"      /* derivative-specific definitions */

#include "common.h"
#include "gpio.h"



#define uchar unsigned char  //自己加的
#define uint unsigned int   //自己加的

//位操作定义
#define LCD_RST  PTA13_OUT 
#define LCD_CE   PTA14_OUT 
#define LCD_DC   PTA16_OUT 
#define SDIN     PTA15_OUT 
#define SCLK     PTA19_OUT 

//函数声明
void Delay_us(uint ut);
void LCD_Init(void);
void LCD_clear(void);
void LCD_write_byte(uchar, uchar);
void LCD_set_XY(uchar , uchar );      
void LCD_write_char(uchar );
void LCD_Write_Char(uchar ,uchar ,uchar);
void LCD_Write_Num(uchar ,uchar ,uint,uchar);
void LCD_write_english_string(uchar ,uchar ,char *); 
void LCD_write_chinese(uchar , uchar , char *);
void LCD_write_chinese_string(uchar , uchar ,char *);
void LCD_Write_String(uchar , uchar ,char *);
void LCD_draw_bmp_pixel(uchar ,uchar ,uchar *, uchar ,uchar );

#endif