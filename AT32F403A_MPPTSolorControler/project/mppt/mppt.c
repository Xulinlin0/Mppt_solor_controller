#include "mppt.h"
#include "filter.h"
#include "ctlLoop.h"
#include "at32f403a_407_board.h"

/**************** extern **********************/
extern uint16_t adc1_ordinary_valuetab[4];
extern low_filter_struct gLowFilterMpptPv[3];
extern low_filter_struct  gLowFilterVI[4];
extern struct _DAC_eff dac_eff;

/**************** global **********************/
float pv_Pin = 0;

/*
* ===================================================================
*  Funtion Name : float mpptmode_PVctr(cnt);
*  Description :  PMMT扰动观察法
*  Parameters  :  cnt=100/taskdelayTime,调用频率10HZ,周期100ms
*  Returns     :  Viref的设定值
* ===================================================================
*/
float mpptmode_PVctr(uint8_t cnt)
{
	static float  pv_Plast = 0;
	static uint8_t mppt_up_dn_flag  = 0;
	static float  pv_ref   = 0;
	static uint8_t mppt_run_count = 0;
	
	/* 对PV输入信号滤波 */
	//输入滤波
	gLowFilterMpptPv[0].Input = gLowFilterVI[0].Output;
	gLowFilterMpptPv[1].Input = gLowFilterVI[1].Output;
		
	low_filter_calc(&gLowFilterMpptPv[0]);
	low_filter_calc(&gLowFilterMpptPv[1]);
	
	/* 滤波, 计算输入功率 */
	gLowFilterMpptPv[2].Input = gLowFilterMpptPv[0].Input * gLowFilterMpptPv[1].Input;
	low_filter_calc(&gLowFilterMpptPv[2]);
	pv_Pin = gLowFilterMpptPv[2].Output;

	/*
		注意要考虑一下mppt算法的调用频率
	*/
	if (++mppt_run_count >= cnt) 
	{
		mppt_run_count = 0;
		 
		/* 功率降低，改变扰动方向 */
		if (pv_Plast - pv_Pin > 0.05f) 
		{
			mppt_up_dn_flag = !mppt_up_dn_flag;
		}
		
		if (mppt_up_dn_flag)//扰动方向为增大Vin
		{
			pv_ref = gLowFilterMpptPv[0].Output + MPPT_STEP;
		}
		else								//扰动方向为减小Vin
		{
			pv_ref = gLowFilterMpptPv[0].Output - MPPT_STEP;
		}
		
		pv_Plast = pv_Pin; //保存上一次的输入功率以用来判断扰动方向
		
		if (pv_Pin <= MIN_PV_PIN) //输入功率过小，不进入MPPT模式，Vin置为最小
		{
			mppt_up_dn_flag = 0x01;
			pv_ref = MIN_PV_VIN;
		}
		
		/* 输入电压设定值限幅 */
		if (pv_ref < MIN_PV_VIN)
		{
			mppt_up_dn_flag = 0x01;
			pv_ref = MIN_PV_VIN;
		}
		else if (pv_ref > MAX_PV_VIN)
		{
			mppt_up_dn_flag = 0x00;
			pv_ref = MAX_PV_VIN;
		}
	}	
	
	return pv_ref;
}


