#include "oled.h"
#include "spi.h"

#define X_WIDTH 		 128
#define Y_WIDTH 		 64
#define PAGE_SIZE    8

#define XLevelL		0x00
#define XLevelH		0x10
#define YLevel       0xB0

//OLED显存总共分为8页
//每页8行，一行128个像素点
//OLED的显存
//存放格式如下.
//[0]0 1 2 3 ... 127 (0~7)行	   
//[1]0 1 2 3 ... 127 (8~15)行	
//[2]0 1 2 3 ... 127 (16~23)行	
//[3]0 1 2 3 ... 127 (24~31)行	
//[4]0 1 2 3 ... 127 (32~39)行	
//[5]0 1 2 3 ... 127 (40~47)行	
//[6]0 1 2 3 ... 127 (48~55)行	
//[7]0 1 2 3 ... 127 (56~63)行			   

//数组每个bit存储OLED每个像素点的颜色值(1-亮(白色),0-灭(黑色))
//每个数组元素表示1列8个像素点，一共128列
static unsigned char OLED_buffer[1024] = 
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

//常用ASCII表
//偏移量32
//ASCII字符集
//偏移量32
//大小:6*8
//逐行式，顺向（高位在前）
/************************************6*8的点阵************************************/
const unsigned char F6x8[][8] =		
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, // sp
	0x10,0x10,0x10,0x10,0x00,0x10,0x00,0x00, // !
	0x28,0x28,0x28,0x00,0x00,0x00,0x00,0x00, // "
	0x28,0x28,0x7C,0x28,0x7C,0x28,0x28,0x00, // #
	0x10,0x3C,0x50,0x38,0x14,0x78,0x10,0x00, // $
	0x00,0x4C,0x2C,0x10,0x08,0x60,0x60,0x00, // %
	0x30,0x48,0x50,0x20,0x54,0x48,0x34,0x00, // &
	0x30,0x10,0x20,0x00,0x00,0x00,0x00,0x00, // '
	0x08,0x10,0x20,0x20,0x20,0x10,0x08,0x00, // (
	0x20,0x10,0x08,0x08,0x08,0x10,0x20,0x00, // )
	0x00,0x10,0x54,0x38,0x54,0x10,0x00,0x00, // *
	0x00,0x10,0x10,0x7C,0x10,0x10,0x00,0x00, // +
	0x00,0x00,0x00,0x00,0x00,0x18,0x08,0x10, // ,
	0x00,0x00,0x00,0x00,0x7C,0x00,0x00,0x00, // -
	0x00,0x00,0x00,0x00,0x00,0x30,0x30,0x00, // .
	0x00,0x04,0x08,0x10,0x20,0x40,0x00,0x00, // /
	0x38,0x44,0x4C,0x54,0x64,0x44,0x38,0x00, // 0
	0x10,0x30,0x10,0x10,0x10,0x10,0x38,0x00, // 1
	0x38,0x44,0x04,0x08,0x10,0x20,0x7C,0x00, // 2
	0x7C,0x08,0x10,0x08,0x04,0x44,0x38,0x00, // 3 
	0x08,0x18,0x28,0x48,0x7C,0x08,0x08,0x00, // 4
	0x7C,0x40,0x78,0x04,0x04,0x44,0x38,0x00, // 5
	0x18,0x20,0x40,0x78,0x44,0x44,0x38,0x00, // 6
	0x7C,0x04,0x08,0x10,0x20,0x20,0x20,0x00, // 7
	0x38,0x44,0x44,0x38,0x44,0x44,0x38,0x00, // 8
	0x38,0x44,0x44,0x3C,0x04,0x08,0x30,0x00, // 9
	0x00,0x30,0x30,0x00,0x30,0x30,0x00,0x00, // :
	0x00,0x30,0x30,0x00,0x30,0x10,0x20,0x00, // ;
	0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x00, // <
	0x00,0x00,0x7C,0x00,0x7C,0x00,0x00,0x00, // =
	0x20,0x10,0x08,0x04,0x08,0x10,0x20,0x00, // >
	0x38,0x44,0x04,0x08,0x10,0x00,0x10,0x00, // ?
	0x38,0x44,0x04,0x34,0x5C,0x44,0x38,0x00, // @
	0x10,0x28,0x44,0x44,0x7C,0x44,0x44,0x00, // A
	0x78,0x44,0x44,0x78,0x44,0x44,0x78,0x00, // B
	0x38,0x44,0x40,0x40,0x40,0x44,0x38,0x00, // C
	0x70,0x48,0x44,0x44,0x44,0x48,0x70,0x00, // D
	0x7C,0x40,0x40,0x78,0x40,0x40,0x7C,0x00, // E
	0x7C,0x40,0x40,0x78,0x40,0x40,0x40,0x00, // F
	0x38,0x44,0x40,0x5C,0x44,0x44,0x3C,0x00, // G
	0x44,0x44,0x44,0x7C,0x44,0x44,0x44,0x00, // H
	0x38,0x10,0x10,0x10,0x10,0x10,0x38,0x00, // I
	0x1C,0x08,0x08,0x08,0x08,0x48,0x30,0x00, // J
	0x44,0x48,0x50,0x60,0x50,0x48,0x44,0x00, // K
	0x40,0x40,0x40,0x40,0x40,0x40,0x7C,0x00, // L
	0x44,0x6C,0x54,0x54,0x44,0x44,0x44,0x00, // M
	0x44,0x44,0x64,0x54,0x4C,0x44,0x44,0x00, // N
	0x38,0x44,0x44,0x44,0x44,0x44,0x38,0x00, // O
	0x78,0x44,0x44,0x78,0x40,0x40,0x40,0x00, // P
	0x38,0x44,0x44,0x44,0x54,0x48,0x34,0x00, // Q
	0x78,0x44,0x44,0x78,0x50,0x48,0x44,0x00, // R
	0x3C,0x40,0x40,0x38,0x04,0x04,0x78,0x00, // S
	0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x00, // T
	0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00, // U
	0x44,0x44,0x44,0x44,0x44,0x28,0x10,0x00, // V
	0x44,0x44,0x44,0x54,0x54,0x54,0x28,0x00, // W
	0x44,0x44,0x28,0x10,0x28,0x44,0x44,0x00, // X
	0x44,0x44,0x44,0x28,0x10,0x10,0x10,0x00, // Y
	0x7C,0x04,0x08,0x10,0x20,0x40,0x7C,0x00, // Z
	0x38,0x20,0x20,0x20,0x20,0x20,0x38,0x00, // [
	0x00,0x40,0x20,0x10,0x08,0x04,0x00,0x00, // '\'
	0x38,0x08,0x08,0x08,0x08,0x08,0x38,0x00, // ]
	0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00, // ^
	0x00,0x00,0x00,0x00,0x00,0x00,0x7C,0x00, // _
	0x20,0x10,0x08,0x00,0x00,0x00,0x00,0x00, // '
	0x00,0x00,0x38,0x04,0x3C,0x44,0x3C,0x00, // a
	0x40,0x40,0x58,0x64,0x44,0x44,0x78,0x00, // b
	0x00,0x00,0x38,0x40,0x40,0x44,0x38,0x00, // c
	0x04,0x04,0x34,0x4C,0x44,0x44,0x3C,0x00, // d
	0x00,0x00,0x38,0x44,0x7C,0x40,0x38,0x00, // e
	0x38,0x24,0x20,0x70,0x20,0x20,0x20,0x00, // f
	0x00,0x00,0x3C,0x44,0x44,0x3C,0x04,0x38, // g
	0x40,0x40,0x58,0x64,0x44,0x44,0x44,0x00, // h
	0x10,0x00,0x30,0x10,0x10,0x10,0x38,0x00, // i
	0x08,0x00,0x18,0x08,0x08,0x08,0x48,0x30, // j
	0x40,0x40,0x48,0x50,0x60,0x50,0x48,0x00, // k
	0x30,0x10,0x10,0x10,0x10,0x10,0x38,0x00, // l
	0x00,0x00,0x68,0x54,0x54,0x44,0x44,0x00, // m
	0x00,0x00,0x58,0x64,0x44,0x44,0x44,0x00, // n
	0x00,0x00,0x38,0x44,0x44,0x44,0x38,0x00, // o
	0x00,0x00,0x58,0x64,0x64,0x58,0x40,0x40, // p
	0x00,0x00,0x34,0x4C,0x4C,0x34,0x04,0x04, // q
	0x00,0x00,0x58,0x64,0x40,0x40,0x40,0x00, // r
	0x00,0x00,0x3C,0x40,0x38,0x04,0x78,0x00, // s
	0x20,0x20,0x70,0x20,0x20,0x24,0x18,0x00, // t
	0x00,0x00,0x44,0x44,0x44,0x4C,0x34,0x00, // u
	0x00,0x00,0x44,0x44,0x44,0x28,0x10,0x00, // v
	0x00,0x00,0x44,0x44,0x54,0x54,0x28,0x00, // w
	0x00,0x00,0x44,0x28,0x10,0x28,0x44,0x00, // x
	0x00,0x00,0x44,0x44,0x44,0x3C,0x04,0x38, // y
	0x00,0x00,0x7C,0x08,0x10,0x20,0x7C,0x00, // z
	0x10,0x20,0x20,0x40,0x20,0x20,0x10,0x00, // {
	0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x00, // |
	0x20,0x10,0x10,0x08,0x10,0x10,0x20,0x00, // }
	0x00,0x00,0x00,0x20,0x54,0x08,0x00,0x00, // ~
};


void OLED_Init_GPIO(void)
{
	gpio_init_type gpio_init_struct;

  /* enable the led clock */
	crm_periph_clock_enable(OLED_GPIO_CRM_CLK, TRUE);

  /* set default parameter */
  gpio_default_para_init(&gpio_init_struct);

  /* configure the led gpio */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = OLED_RST_PIN | OLED_DC_PIN | OLED_CS_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(OLED_GPIO, &gpio_init_struct);
	
	OLED_GPIO->scr = OLED_RST_PIN | OLED_DC_PIN | OLED_CS_PIN;
}

void OLED_Reset(void)
{
	OLED_GPIO->scr = OLED_RST_PIN;
	dwt_delay_ms(100);
	OLED_GPIO->clr = OLED_RST_PIN;
	dwt_delay_ms(100);
	OLED_GPIO->scr = OLED_RST_PIN;
}

//向OLED写字节，若cmd=0写入数据，cmd=1写入命令
void OLED_WR_Byte(u8 dat, u8 cmd)
{
	if (cmd)
	{
		OLED_GPIO->scr = OLED_DC_PIN;
	}
	else 
	{
		OLED_GPIO->clr = OLED_DC_PIN;
	}
	
	OLED_GPIO->clr = OLED_CS_PIN;
	SPI1_WriteByte(dat);
	OLED_GPIO->scr = OLED_CS_PIN;
}

/* OLED画面填充, 可用于清屏 */
void OLED_Fill(u8 bmp_data)
{
	u8 y,x;
	
	for(y=0;y<8;y++)
	{
		OLED_WR_Byte(0xb0+y, OLED_CMD);	// 设置行起始地址
		OLED_WR_Byte(0x01, OLED_CMD);		// 设置低列起始地址
		OLED_WR_Byte(0x10, OLED_CMD);		// 设置高列起始地址
		for(x=0;x<X_WIDTH;x++)
		{
			OLED_WR_Byte(bmp_data, OLED_DAT);
		}
	}
}

/*
	设置起始地址
  *Y轴是按8格递进的，y轴0~63，只能按8格的倍数显示，
  *因为列行式只能按8个字节进行
*/
void OLED_Set_Pos(u8 x, u8 y)
{ 
  OLED_WR_Byte(0xb0+(y>>3), OLED_CMD);				// 设置行起始地址
  OLED_WR_Byte(((x&0xf0)>>4)|0x10, OLED_CMD);// 设置列起始地址
  OLED_WR_Byte((x&0x0f)|0x01, OLED_CMD); 
} 

void OLED_Init(void)
{
	SPI1_Init(); 		  //初始化SPI1
 	OLED_Init_GPIO(); //初始化GPIO
 	dwt_delay_ms(200);
	OLED_Reset();     //复位OLED

/**************初始化SSD1306*****************/	
	OLED_WR_Byte(0xAE, OLED_CMD); /*display off*/
	OLED_WR_Byte(0x00, OLED_CMD); /*set lower column address*/
	OLED_WR_Byte(0x10, OLED_CMD); /*set higher column address*/
	OLED_WR_Byte(0x40, OLED_CMD); /*set display start line*/ 
	OLED_WR_Byte(0xB0, OLED_CMD); /*set page address*/
	OLED_WR_Byte(0x81, OLED_CMD); /*contract control*/ 
	OLED_WR_Byte(0xFF, OLED_CMD); /*128*/
	OLED_WR_Byte(0xA1, OLED_CMD); /*set segment remap*/ 
	OLED_WR_Byte(0xA6, OLED_CMD); /*normal / reverse*/
	OLED_WR_Byte(0xA8, OLED_CMD); /*multiplex ratio*/ 
	OLED_WR_Byte(0x3F, OLED_CMD); /*duty = 1/64*/
	OLED_WR_Byte(0xC8, OLED_CMD); /*Com scan direction*/
	OLED_WR_Byte(0xD3, OLED_CMD); /*set display offset*/ 
	OLED_WR_Byte(0x00, OLED_CMD);
	OLED_WR_Byte(0xD5, OLED_CMD); /*set osc division*/ 
	OLED_WR_Byte(0x80, OLED_CMD);
	OLED_WR_Byte(0xD9, OLED_CMD); /*set pre-charge period*/ 
	OLED_WR_Byte(0XF1, OLED_CMD);
	OLED_WR_Byte(0xDA, OLED_CMD); /*set COM pins*/ 
	OLED_WR_Byte(0x12, OLED_CMD);
	OLED_WR_Byte(0xDB, OLED_CMD); /*set vcomh*/ 
	OLED_WR_Byte(0x30, OLED_CMD);
	OLED_WR_Byte(0x8D, OLED_CMD); /*set charge pump disable*/ 
	OLED_WR_Byte(0x14, OLED_CMD);
	OLED_WR_Byte(0xAF, OLED_CMD); /*display ON*/
	
}  

void OLED_Set_Pixel(unsigned char x, unsigned char y,unsigned char color)
{
	if(color)
	{
		OLED_buffer[(y/PAGE_SIZE)*X_WIDTH+x]|= (1<<(y%PAGE_SIZE))&0xff;
	}
	else
	{
		OLED_buffer[(y/PAGE_SIZE)*X_WIDTH+x]&= ~((1<<(y%PAGE_SIZE))&0xff);
	}
}		   		

void OLED_Display(void)
{
	u8 i,n;		    
	for(i=0;i<PAGE_SIZE;i++)  
	{  
		OLED_WR_Byte (YLevel+i, OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte (XLevelL,  OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte (XLevelH,  OLED_CMD);      //设置显示位置—列高地址   
		for(n=0;n<X_WIDTH;n++)
		{
			OLED_WR_Byte(OLED_buffer[i*X_WIDTH+n], OLED_DAT); 
		}
	}   //更新显示
}

void GUI_ShowChar(u8 x,u8 y,u8 chr,u8 mode)
{      	
	unsigned char c=0,i=0,tmp,j=0;	
	c=chr-' ';//得到偏移后的值			
	if(x>X_WIDTH-1){x=0;y=y+2;}
	
	for(i=0;i<8;i++)
	{
		if(mode)
		{
			tmp = F6x8[c][i];
		}
		else
		{
			tmp = ~(F6x8[c][i]);
		}
		for(j=0;j<8;j++)
		{
			if(tmp&(0x80>>j))
			{
				OLED_Set_Pixel(x+j, y+i,1);
			}
			else
			{
				OLED_Set_Pixel(x+j, y+i,0);
			}
		}
	}
	OLED_Display();
}

void GUI_ShowString(u8 x,u8 y,u8 *chr,u8 mode)
{
	unsigned char j=0;

	while (chr[j]!='\0')
	{		
		GUI_ShowChar(x,y,chr[j],mode);
		x+=6;
		if(x>120)
		{
			x=0;
			y+=8;
		}
		j++;
	}
}

void GUI_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2,u8 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 

	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; 
	uCol=y1; 
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴 
	else distance=delta_y; 
	for(t=0;t<=distance+1;t++ )//画线输出 
	{ 
		OLED_Set_Pixel(uRow,uCol,color);
		xerr+=delta_x ; 
		yerr+=delta_y ; 
		if(xerr>distance) 
		{ 
			xerr-=distance; 
			uRow+=incx; 
		} 
		if(yerr>distance) 
		{ 
			yerr-=distance; 
			uCol+=incy; 
		} 
	}  
	OLED_Display();
} 

