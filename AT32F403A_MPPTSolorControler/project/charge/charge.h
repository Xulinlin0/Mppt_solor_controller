#ifndef __CHARGE_H
#define __CHARGE_H

#include "at32f403a_407_board.h"

/* =================== define ================== */


/* ==================== typedef ================= */
enum Charge_States
{
	charge_idle,        //电池空闲
	charge_fault,       //电池故障
	charge_done,        //电池充电完毕
	charge_precharge,   //电池预充状态
	charge_charging,	  //电池cc-cv充电
	charge_float        //电池浮充状态
};

enum charger_fault
{
	none_err,			//无错误
	pre_timeout,  //预充超时
	ing_timeout,  //充电阶段超时
	ing_Vocutoff, //充电阶段出现Vo低于最低电压
	float_timeout,//浮充阶段充电超时
};

typedef struct _ChargePara{
	enum Charge_States		states_charge; //充电状态
	enum charger_fault		err_code;           //故障代码
	
	float  charging_capacity;		  //电池容量
	
	float  charging_Vo;						//电池充电电压
	float  floating_Vo;						//电池浮充电压
	float  cutoff_Vo;							//电池截止电压
		
	float  precharg_Io;						//电池预充电流
	float  charging_Io;						//电池充电电流
	float  floating_Io;						//电池浮充电流
	float  stop_Io;								//电池停止电流
	
	float  charge_voltage;      	//输出充电电压
	float  charge_current;      	//输出充电电流
	
	unsigned int  precharge_time;	//预充时间
	unsigned int  charge_time;		//充电时间
	unsigned int  floating_time;	//浮充时间

}BattryPara_TypeDef;

/* ====================function declaration================= */
uint8_t ChargeStateCalc(BattryPara_TypeDef *bat);


#endif
