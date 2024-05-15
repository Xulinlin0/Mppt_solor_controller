#ifndef __SOFT_IIC_H
#define __SOFT_IIC_H

#include "at32f403a_407.h"
#include "at32f403a_407_board.h"

//IO方向设置
#define SDA_IN()  {GPIOB->cfglr&=0X0FFFFFFF;GPIOB->cfglr|=(u32)8<<28;}
#define SDA_OUT() {GPIOB->cfglr&=0X0FFFFFFF;GPIOB->cfglr|=(u32)3<<28;}

//IO操作函数	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	  //((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) ,addr = GPIOB_ODR_Addr (GPIOB_BASE+12) //0x40010C0C 
#define READ_SDA   PBin(7)  //输入SDA 

/** @addtogroup AT32F403A_407_middlewares_i2c_application_library
  * @{
  */
  
#define I2Cx_SCL_PIN                     GPIO_PINS_6
#define I2Cx_SCL_GPIO_PORT               GPIOB
#define I2Cx_SCL_GPIO_CLK                CRM_GPIOB_PERIPH_CLOCK

#define I2Cx_SDA_PIN                     GPIO_PINS_7
#define I2Cx_SDA_GPIO_PORT               GPIOB
#define I2Cx_SDA_GPIO_CLK                CRM_GPIOB_PERIPH_CLOCK

//SCL SDA
#define I2C_SCL_HIGH()                   gpio_bits_set(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN)
#define I2C_SCL_LOW()                    gpio_bits_reset(I2Cx_SCL_GPIO_PORT, I2Cx_SCL_PIN)

#define I2C_SDA_HIGH()                   gpio_bits_set(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN)
#define I2C_SDA_LOW()                    gpio_bits_reset(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN)

#define I2C_SDA_READ()                   (gpio_input_data_bit_read(I2Cx_SDA_GPIO_PORT, I2Cx_SDA_PIN) == SET)

//function statement
void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);


#endif

