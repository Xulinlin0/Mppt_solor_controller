#include "CtlLoop.h"
#include "adc.h"
#include "timer.h"
#include "filter.h"
#include "charge.h"
#include "esp8266.h"

#include "SEGGER_RTT.h"


/* v2_buck
void BuckBoard_init(void)
{
  gpio_init_type gpio_init_struct;
	
	crm_periph_clock_enable(ACQ_SD_GPIO_CLK, TRUE);
	crm_periph_clock_enable(ACQ_OPEN_GPIO_CLK, TRUE);
	
  gpio_init_struct.gpio_pins = ACQ_SD_PIN ;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(ACQ_SD_GPIO_PORT, &gpio_init_struct);
	
	gpio_init_struct.gpio_pins = ACQ_OPEN_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(ACQ_OPEN_GPIO_PORT, &gpio_init_struct);
	
	gpio_init_struct.gpio_pins = ACQ_LED1_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(ACQ_LED1_GPIO_PORT, &gpio_init_struct);
	
	gpio_init_struct.gpio_pins = ACQ_LED2_PIN;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(ACQ_LED2_GPIO_PORT, &gpio_init_struct);
	
	ACQ_OPEN_GPIO_PORT->scr = ACQ_OPEN_PIN;
	ACQ_LED1_GPIO_PORT->clr = ACQ_LED1_PIN;
	dwt_delay_ms(1);
	ACQ_SD_GPIO_PORT->scr = ACQ_SD_PIN;
}
*/

/**************** extern **********************/
extern __IO uint16_t adc1_ordinary_valuetab[4];
extern low_filter_struct gLowFilterVI[4];
extern uint8_t mppt_enable;
extern BattryPara_TypeDef   gMy_Battry;
extern char uart_tx_parm[150];
extern uint8_t	chargerMode;
extern uint8_t	mppt_enable;

/**************** global **********************/
struct  _ADI SADC = {0};			//输入输出参数采样值和平均值

pid_struct pid_ctrol[4] = {0};	//[0]-Vin [1]-Iin [2]-Vout [3]-Iout pid计算相关参数

struct _DAC_eff dac_eff =
{
	 CAL_VOUT_K / 4096.0f * 100,		CAL_VOUT_B, //Vout
	 CAL_VIN_K  / 4096.0f * 100,		CAL_VIN_B,	//Vin
	 0,						1 / 4096.0f * 20, 		//Iout
	 0,						1 / 4096.0f	* 20 		  //Iin
};

float pwm; //记录pid输出结果

enum loop_fault loop_err = none; //环路异常状态信息

/**************** function **********************/

int32_t abs(int32_t x)
{
	if(x >= 0)	return x;
	else	return -x;
}

float f_abs(float x)
{
	if (x < 0.0f) return -x;
	else return x;
};

float f_min(float x, float y)
{
	if (x < y) return x;
	else return y;
}

/* 电流零漂检测 
	零漂正校验返回0，检测成功返回1，失败返回-1
*/
int8_t ExcursionCheck(void)
{
	static uint16_t excu_count = 0,excu_FaultCnt = 0;
	static uint32_t IinSum=0, IoutSum=0;
	static int32_t IinErr=0, IoutErr=0;
	
	excu_count++;
	
	//输入输出电流偏置求和
  IinSum += adc1_ordinary_valuetab[1];
	IoutSum += adc1_ordinary_valuetab[3];
	
	if (excu_count == 256)	
	{
		IinErr = IinSum>>8;
		IoutErr = IoutSum>>8;
		
		excu_count =0;
		IinSum=0;
		IoutSum=0;
		
		if (abs(IinErr - DV_REF) < 80 && abs(IoutErr - DV_REF) < 80)
		{
			dac_eff.Iin_offset = IinErr;
			dac_eff.Iout_offset = IoutErr;
			return 1;
		}
		else if (excu_FaultCnt++ > 3)	//连续3次校验错误
		{
			excu_FaultCnt = 0;
			return -1;
		}
	}
	return 0;
}


/* 采样输出电压、输出电流、输入电压、输入电流 */
void ADCSample(uint8_t Iopen)
{
	//输入输出采样参数求和，用以计算平均值
	static uint32_t VinAvgSum=0,IinAvgSum=0,VoutAvgSum=0,IoutAvgSum=0;
	
	//从DMA缓冲器中获取数据
	SADC.Vin	= adc1_ordinary_valuetab[0];
	SADC.Vout = adc1_ordinary_valuetab[2];
	
	if (Iopen)
	{
		SADC.Iin = (int32_t)adc1_ordinary_valuetab[1] - dac_eff.Iin_offset;
		SADC.Iout =(int32_t)adc1_ordinary_valuetab[3] - dac_eff.Iout_offset;
		
		IinAvgSum = IinAvgSum + SADC.Iin - (IinAvgSum>>2);
		SADC.IinAvg = (float)((IinAvgSum>>2)*dac_eff.Iin_eff_k);
	
		IoutAvgSum = IoutAvgSum + SADC.Iout - (IoutAvgSum>>2);
		SADC.IoutAvg = (float)((IoutAvgSum>>2)*dac_eff.Iout_eff_k);
	}
	
	//计算各个采样值的平均值-滑动平均方式
	VinAvgSum = VinAvgSum + SADC.Vin - (VinAvgSum>>2);//求和，新增入一个新的采样值，同时减去之前的平均值。
	SADC.VinAvg =	(float)((VinAvgSum>>2)*dac_eff.Vin_eff_k+dac_eff.Vin_eff_b);//求平均
	
	VoutAvgSum = VoutAvgSum + SADC.Vout - (VoutAvgSum>>2);
	SADC.VoutAvg = (float)((VoutAvgSum>>2)*dac_eff.Vout_eff_k+dac_eff.Vout_eff_b);
			
}

/* 	pid初始化  */
void increPid_init(void)
{
	//VinPID控制环参数初始化	
	pid_ctrol[0].value = 18.0f;
	pid_ctrol[0].a0 = VIN_KP + VIN_KI + VIN_KD;
	pid_ctrol[0].a1 = VIN_KP + 2 * VIN_KD;
	pid_ctrol[0].a2 = VIN_KD;
	pid_ctrol[0].Ek_Dead = 0.01f;
	pid_ctrol[0].Output_min = 0;
	pid_ctrol[0].Output_max = 3;	//MPPT串Iout,输出为Iout
	pid_ctrol[0].Incem = 0;
	
	//VoutPID控制环参数初始化
	pid_ctrol[2].target = 14.4f;
	pid_ctrol[2].a0 = VOUT_KP + VOUT_KI + VOUT_KD;
	pid_ctrol[2].a1 = VOUT_KP + 2 * VOUT_KD;
	pid_ctrol[2].a2 = VOUT_KD;
	pid_ctrol[2].Ek_Dead = 0.01f;
	pid_ctrol[2].Output_min = 0;
	pid_ctrol[2].Output_max = 3;
	pid_ctrol[2].Incem = 0;
	
	//IoutPID控制环参数初始化
	pid_ctrol[3].target = 3.0f;
	pid_ctrol[3].a0 = IOUT_KP + IOUT_KI + IOUT_KD;
	pid_ctrol[3].a1 = IOUT_KP + 2 * IOUT_KD;
	pid_ctrol[3].a2 = IOUT_KD;
	pid_ctrol[3].Ek_Dead = 0.001f;
	pid_ctrol[3].Output_min = MIN_BUKC_DUTY;
	pid_ctrol[3].Output_max = MAX_BUCK_DUTY;
	pid_ctrol[3].Incem = 0;
}

/* 	增量式pid计算 	*/
float increPid_cal(pid_struct *pid_cot)
{
	pid_cot->Ek_0 = pid_cot->target - pid_cot->value;
	
	if (f_abs(pid_cot->Ek_0) < pid_cot->Ek_Dead)//误差小于设定死区就不进行PID计算，保持上一次输出
	{
		pid_cot->Incem = 0;
	}
	else
	{
		pid_cot->Incem = pid_cot->a0 * pid_cot->Ek_0 - pid_cot->a1 * pid_cot->Ek_1 + pid_cot->a2 * pid_cot->Ek_2;
	}
	
	pid_cot->Output += pid_cot->Incem;
	
	pid_cot->Ek_2	= pid_cot->Ek_1;	//保存k-2误差
	pid_cot->Ek_1 = pid_cot->Ek_0;	//保存k-1误差
	
	//环路输出最大最小占空比限制
	if (pid_cot->Output > pid_cot->Output_max)
	{
		pid_cot->Output = pid_cot->Output_max;
	}
	else if (pid_cot->Output < pid_cot->Output_min)
	{
		pid_cot->Output = pid_cot->Output_min;
	}
	
	return pid_cot->Output;
}

uint8_t CV_flag = 0;
/* 环路控制 */
void Buck_LoopControl(void)
{	
	static uint8_t v_loop_ctrcnt = 0;
	
	if (mppt_enable) //作为mppt计算功率的实时数据
	{
		gLowFilterVI[0].Input = adc1_ordinary_valuetab[0] * dac_eff.Vin_eff_k + dac_eff.Vin_eff_b;
		low_filter_calc(&gLowFilterVI[0]);//滤波
		
		gLowFilterVI[1].Input = ((int32_t)adc1_ordinary_valuetab[1] - dac_eff.Iin_offset) * dac_eff.Iin_eff_k;
		low_filter_calc(&gLowFilterVI[1]);
	}

	if (++v_loop_ctrcnt >= 10)
	{
		v_loop_ctrcnt = 0;
		/* Vout pid control*/
		gLowFilterVI[2].Input = adc1_ordinary_valuetab[2] * dac_eff.Vout_eff_k + dac_eff.Vout_eff_b;
		low_filter_calc(&gLowFilterVI[2]);
		pid_ctrol[2].value = gLowFilterVI[2].Output;
		
		pid_ctrol[3].target = increPid_cal(&pid_ctrol[2]);
		
		/* 若开启了mppt模式, Vin pid control */
		if (mppt_enable)
		{
			pid_ctrol[0].target = gLowFilterVI[0].Output; //得到Vin实时值
			
			increPid_cal(&pid_ctrol[0]);
			
			if (pid_ctrol[0].Output > pid_ctrol[2].Output)
			{
				CV_flag = 1;
			}
			else
			{
				CV_flag = 0;
			}
			pid_ctrol[3].target = f_min(pid_ctrol[0].Output, pid_ctrol[2].Output);
		}
	} 
	
	/* 由Iout设定pwm */
	gLowFilterVI[3].Input = ((int32_t)adc1_ordinary_valuetab[3] - dac_eff.Iout_offset) * dac_eff.Iout_eff_k;
	low_filter_calc(&gLowFilterVI[3]);
	pid_ctrol[3].value = gLowFilterVI[3].Output;
	
	increPid_cal(&pid_ctrol[3]);
	pwm = pid_ctrol[3].Output;	 ///111

	//更新对应寄存器
	TMR1->c1dt = (uint16_t)(pwm * 2398 );
	TMR1->c3dt = (uint16_t)(pwm * (2398 >> 1));
}

#ifdef TEST_DEBUG
void BuckVI_increControl(void)
{
	static uint8_t v_loop_ctrcnt = 0;
		
	/* Vin,Vout,Iout滤波 */
	if (mppt_enable)
	{
		low_filter_calc(&gLowFilterVI[0]);//滤波
		gLowFilterVI[0].Input = adc1_ordinary_valuetab[0] * dac_eff.Vin_eff_k + dac_eff.Vin_eff_b;
		pid_ctrol[0].value = gLowFilterVI[0].Output;
	}
	
	gLowFilterVI[2].Input = adc1_ordinary_valuetab[1] * dac_eff.Vout_eff_k + dac_eff.Vout_eff_b;
	low_filter_calc(&gLowFilterVI[2]);
	pid_ctrol[2].value = gLowFilterVI[2].Output;
	
	gLowFilterVI[3].Input = ((int32_t)adc1_ordinary_valuetab[3] - dac_eff.Iout_offset) * dac_eff.Iout_eff_k;
	low_filter_calc(&gLowFilterVI[3]);
	pid_ctrol[3].value = gLowFilterVI[3].Output;

	
	//电压环频率为电流环十分之一
	if (++v_loop_ctrcnt == 10)
	{
		increPid_cal(&pid_ctrol[2]);
		float p_pwm = pid_ctrol[2].Output;
		
		if (mppt_enable) 
		{
			increPid_cal(&pid_ctrol[0]);
			p_pwm = f_min(pid_ctrol[0].Output, p_pwm);
		}
		
		v_loop_ctrcnt = 0;
		
		pid_ctrol[3].target = f_min(p_pwm, pid_ctrol[3].target);
	}

	pwm = increPid_cal(&pid_ctrol[3]);
	
	//更新对应寄存器
	TMR1->c1dt = (uint16_t)(pwm * 2398 );
	TMR1->c3dt = (uint16_t)(pwm * (2398 >> 1));
}



uint8_t CV_flag = 0;

void charger3degreeCtrl(void)
{
	static float pwm1 = 0.1, pwm2 = 0.1;
	
	pid_ctrol[3].target = gMy_Battry.charge_current;
	pid_ctrol[2].target = gMy_Battry.charge_voltage;
	
	// Iout滤波
	gLowFilterVI[3].Input = ((int32_t)adc1_ordinary_valuetab[3] - dac_eff.Iout_offset) * dac_eff.Iout_eff_k;
	low_filter_calc(&gLowFilterVI[3]);
	pid_ctrol[3].value = gLowFilterVI[3].Output;
	
	// Vout滤波
	gLowFilterVI[2].Input = adc1_ordinary_valuetab[2] * dac_eff.Vout_eff_k + dac_eff.Vout_eff_b;
	low_filter_calc(&gLowFilterVI[2]);
	pid_ctrol[2].value = gLowFilterVI[2].Output;

	// Vin
	gLowFilterVI[0].Input = adc1_ordinary_valuetab[0] * dac_eff.Vin_eff_k + dac_eff.Vin_eff_b;
//	low_filter_calc(&gLowFilterVI[2]);
//	pid_ctrol[2].value = gLowFilterVI[2].Output;
	pwm1 = increPid_cal(&pid_ctrol[2]);
	pwm2 = increPid_cal(&pid_ctrol[3]);
	
	if (pwm1 < pwm2) CV_flag = 1;
	pwm = f_min(pwm1, pwm2);
	
	/*
	TMR1->c1dt = (uint16_t)(pwm * 4798 );
	TMR1->c3dt = (uint16_t)(pwm * (4798 >> 1));
	*/
	
	//更新对应寄存器
	TMR1->c1dt = (uint16_t)(pwm * 2398 );
	TMR1->c3dt = (uint16_t)(pwm * (2398 >> 1));
}

#endif


/* 环路故障检测 ,返回故障类型 */
//参数1：是否开启电流检测, 参数2：检测次数
uint8_t LoopFaultCheck(uint8_t Iopen, uint8_t cnt)
{
	static uint8_t fault_cnt=0;
	
	ADCSample(Iopen);
	
//	if ((SADC.Iout > MAX_SHORT_I) && (SADC.Vout < MIN_SHORT_V))
//	{
//		loop_err = iout_short;//短路检测
//	}
//	else 
	if (Iopen && (SADC.Iout > (IOUT_OVER_VAL + dac_eff.Iout_offset)))	//过流检测
	{
		if (fault_cnt++ > cnt)
		{
			fault_cnt = 0;
			loop_err = iout_over;
		}
	}
	else if (SADC.Vin < (VIN_UNDER_VAL - 8)) //输入欠压检测
	{
		if (fault_cnt++ > cnt)
		{
			fault_cnt = 0;
			loop_err = vin_under;
		}
	}
	else if (SADC.Vin > (VIN_OVER_VAL + 8)) //输入过压检测
	{
		if (fault_cnt++ > cnt)
		{
			fault_cnt = 0;
			loop_err = vin_over;
		}
	}
	
//	else if (SADC.Vout > VOUT_OVER_VAL) //输出过压检测
//	{
//		if (vout_over_cnt++ > 2)
//		{
//			vout_over_cnt = 0;
//			loop_err = vout_over;
//		}
//	}

	return loop_err;
}

/* 环路恢复检测 ,返回故障类型 */
uint8_t LoopFaultBack(uint8_t err, uint8_t cnt)
{
	static uint8_t back_cnt=0;
	
	ADCSample(1);
	
	switch(err)
	{
		case vin_under:	
		case vin_over:	if (SADC.Vin > (VIN_UNDER_VAL + 8) && SADC.Vin < (VIN_OVER_VAL - 8))
									  {
											if (back_cnt++ > cnt)
											{
												back_cnt = 0;
												loop_err = none;
											}
									  }
									  break;
	}
	return loop_err;
}


