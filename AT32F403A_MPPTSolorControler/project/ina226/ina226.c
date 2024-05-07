#include "ina226.h"
#include "soft_iic.h"

#include "at32f403a_407_board.h"

u16 INA226_Read2Byte(u8 reg_addr)
{
	u16 reg_data=0;
	u16 temp=0;
	IIC_Start();
	IIC_Send_Byte(WRITE_ADDR);
	if(IIC_Wait_Ack()) return 0;
	IIC_Send_Byte(reg_addr);
	if(IIC_Wait_Ack()) return 0;
	IIC_Start();
	IIC_Send_Byte(READ_ADDR);
	if(IIC_Wait_Ack()) return 0;
	reg_data=IIC_Read_Byte(1);
	reg_data=(reg_data<<8)&0xFF00;
	temp=IIC_Read_Byte(0);
	IIC_Stop();
	reg_data|=temp;
	return reg_data;
}

u16 INA226_Write2Byte(u8 reg_addr,u16 reg_data)
{
	u8 data_H=(u8)((reg_data&0xFF00)>>8);
	u8 data_L=(u8)reg_data&0x00FF;
	IIC_Start();
	IIC_Send_Byte(WRITE_ADDR);
	if(IIC_Wait_Ack()) return 0;
	IIC_Send_Byte(reg_addr);
	if(IIC_Wait_Ack()) return 0;
	IIC_Send_Byte(data_H);
	if(IIC_Wait_Ack()) return 0;
	IIC_Send_Byte(data_L);
	if(IIC_Wait_Ack()) return 0;
	IIC_Stop();
	dwt_delay_ms(2);
	return 1 ;

}

void INA226_Init(void)
{
	INA226_Write2Byte(Config_Reg,0x4527);
	INA226_Write2Byte(Calib_Reg,0x5000);
}

