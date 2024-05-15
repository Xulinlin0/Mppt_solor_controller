#ifndef __LCD_H
#define __LCD_H		

#include "at32f403a_407_board.h"

#include "oled.h"

/******************** define gpio but spi ********************/
#define OLED_GPIO_CS            GPIOA
#define OLED_GPIO_CS_CRM_CLK    CRM_GPIOA_PERIPH_CLOCK

#define OLED_GPIO            GPIOB
#define OLED_GPIO_CRM_CLK    CRM_GPIOB_PERIPH_CLOCK

#define OLED_RST_PIN         GPIO_PINS_7 //复位	PB7
#define OLED_DC_PIN          GPIO_PINS_6	//数据/命令 PB6
#define OLED_CS_PIN          GPIO_PINS_15 //片选

/******************** define sendByte cmd ********************/
#define OLED_CMD		0
#define OLED_DAT		1

void OLED_WR_Byte(u8 dat, u8 cmd);
void OLED_Fill(u8 bmp_data);
void OLED_Set_Pos(u8 x, u8 y);
void OLED_Init(void); 
void GUI_ShowString(u8 x,u8 y,u8 *chr,u8 mode);
void GUI_DrawLine(u8 x1, u8 y1, u8 x2, u8 y2,u8 color);
#endif
