#include "charge.h"
#include "CtlLoop.h"

/**************** extern **********************/
extern __IO uint16_t adc1_ordinary_valuetab[4];
extern struct _DAC_eff dac_eff;
extern struct  _ADI SADC;

/**************** global **********************/
/* 电池参数定义	*/
BattryPara_TypeDef    gMy_Battry = {
	
  .states_charge     = charge_idle,    //充电状态 
  .err_code          = none_err,    	 //故障代码
  
  .charging_capacity = 20.0f, //电池容量，单位：AH
	
  .charging_Vo    	 = 14.4f, //电池电压：单位：V
  .floating_Vo    	 = 13.8f, //浮充电压：单位：V
  .cutoff_Vo      	 = 10.8f, //最低电压：单位：V
	
  .precharg_Io     	 = 0.50f, //预充电流：单位：A
  .charging_Io       = 2.00f, //充电电流：单位：A
  .floating_Io       = 0.30f, //浮充电流：单位：A
  .stop_Io           = 0.30f, //停止电流：单位：A

  .precharge_time    = 6000,  //预充时间(10分钟,按10HZ中断计算)
  .charge_time       = 36000, //充电时间(60分钟,按10HZ中断计算)
  .floating_time     = 12000 //浮充时间(20分钟,按10HZ中断计算)
};

/**
 **********************************************************
 *     Funtion Name : uint8_t ChargeStateCalc(BattryPara_TypeDef *bat)
 *     Description :	电池三段式充电实现
 *     Parameters  :
 *     Returns     :	电池当前状态
 **********************************************************
**/
uint8_t ChargeStateCalc(BattryPara_TypeDef *bat)
{
	static uint16_t swiStaTime = 0;
	static uint16_t precharge_time = 0;
	static uint16_t charge_time = 0;
	
	float IoTemp = ((int32_t)adc1_ordinary_valuetab[3] - dac_eff.Iout_offset) *dac_eff.Iout_eff_k;
	float VoTemp = adc1_ordinary_valuetab[2] * dac_eff.Vout_eff_k + dac_eff.Vout_eff_b;
	
	switch(bat->states_charge)
	{
		/* 电池空闲阶段 */
		case charge_idle:		
				bat->charge_current = bat->floating_Io;//浮充电流
				bat->charge_voltage = bat->charging_Vo;//充电电压
				
				//(Io>=停止电流或Vo<充电电压),进入预充阶段
				swiStaTime = (IoTemp >= bat->stop_Io || VoTemp < bat->charging_Vo) ? (swiStaTime + 1) : 0;
				if (swiStaTime > 10)
				{
					bat->states_charge = charge_precharge;
				}
				
				precharge_time = 0;
				charge_time = 0;
				break;
		/* 电池预充阶段 */
		case charge_precharge: 
				bat->charge_current = bat->precharg_Io;//预充电流
				bat->charge_voltage = bat->charging_Vo;//充电电压
				
				//Vo>最低电压， 进入充电阶段
				swiStaTime = (VoTemp > bat->cutoff_Vo) ? (swiStaTime + 1) : 0;
				if (swiStaTime > 10)
				{
					bat->states_charge = charge_charging;
				}
				
				if (precharge_time > bat->precharge_time)//预充超时，进入故障模式
				{
					bat->err_code = pre_timeout;
					bat->states_charge = charge_fault;
				}
				precharge_time++;
				break;
		/* cc-cv充电中阶段 */
		case charge_charging:
				bat->charge_current = bat->charging_Io;//充电电流
				bat->charge_voltage = bat->charging_Vo;//充电电压
				
				//Vo>浮充电压&&Io<停止电流，进入浮充模式
				swiStaTime = (VoTemp > bat->floating_Vo && IoTemp < bat->stop_Io) ? (swiStaTime + 1) : 0;
				if (swiStaTime > 10)
				{
					bat->states_charge = charge_fault;
				}
				
				if (charge_time > bat->charge_time || VoTemp < bat->cutoff_Vo) //充电超时或Vo<最低电压，视作充电故障
				{
					bat->err_code = (charge_time > bat->charge_time) ? ing_timeout : ing_Vocutoff;
					bat->states_charge = charge_fault;
				}
				break;
		/* 电池浮充阶段 */
		case charge_float:
				bat->charge_current = bat->floating_Io;//浮充电流
				bat->charge_voltage = bat->floating_Vo;//浮充电压
				
				//Io<停止电流，直到浮充时间到，则充电完成
				swiStaTime = (IoTemp < bat->stop_Io) ? (swiStaTime + 1) : 0;		
				if (swiStaTime > bat->floating_time)
				{
					bat->states_charge = charge_done;
				}
				
				if (charge_time > bat->charge_time) //充电超时，视作故障
				{
					bat->err_code = float_timeout;
					bat->states_charge = charge_fault;
				}
				charge_time++;
				break;
		/* 充电完成阶段 */
		case charge_done:	
				bat->charge_current = bat->stop_Io;		 //停止电流
				bat->charge_voltage = bat->floating_Vo;//浮充电压
				if (VoTemp < 0.9f * bat->floating_Vo)  //if输出电压<0.9浮充电压，将电池置回空闲状态
				{
					bat->states_charge = charge_idle;
				}
				break;
		/* 充电故障阶段 */
		case charge_fault:	
				bat->charge_current = bat->stop_Io; 	 //停止电流
				bat->charge_voltage = bat->floating_Vo;//浮充电压
				swiStaTime = 0;
				break;
	}
	return bat->states_charge;
}



