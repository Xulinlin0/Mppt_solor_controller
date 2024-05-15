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
*  Description :  PMMT�Ŷ��۲취
*  Parameters  :  cnt=100/taskdelayTime,����Ƶ��10HZ,����100ms
*  Returns     :  Viref���趨ֵ
* ===================================================================
*/
float mpptmode_PVctr(uint8_t cnt)
{
	static float  pv_Plast = 0;
	static uint8_t mppt_up_dn_flag  = 0;
	static float  pv_ref   = 0;
	static uint8_t mppt_run_count = 0;
	
	/* ��PV�����ź��˲� */
	//�����˲�
	gLowFilterMpptPv[0].Input = gLowFilterVI[0].Output;
	gLowFilterMpptPv[1].Input = gLowFilterVI[1].Output;
		
	low_filter_calc(&gLowFilterMpptPv[0]);
	low_filter_calc(&gLowFilterMpptPv[1]);
	
	/* �˲�, �������빦�� */
	gLowFilterMpptPv[2].Input = gLowFilterMpptPv[0].Input * gLowFilterMpptPv[1].Input;
	low_filter_calc(&gLowFilterMpptPv[2]);
	pv_Pin = gLowFilterMpptPv[2].Output;

	/*
		ע��Ҫ����һ��mppt�㷨�ĵ���Ƶ��
	*/
	if (++mppt_run_count >= cnt) 
	{
		mppt_run_count = 0;
		 
		/* ���ʽ��ͣ��ı��Ŷ����� */
		if (pv_Plast - pv_Pin > 0.05f) 
		{
			mppt_up_dn_flag = !mppt_up_dn_flag;
		}
		
		if (mppt_up_dn_flag)//�Ŷ�����Ϊ����Vin
		{
			pv_ref = gLowFilterMpptPv[0].Output + MPPT_STEP;
		}
		else								//�Ŷ�����Ϊ��СVin
		{
			pv_ref = gLowFilterMpptPv[0].Output - MPPT_STEP;
		}
		
		pv_Plast = pv_Pin; //������һ�ε����빦���������ж��Ŷ�����
		
		if (pv_Pin <= MIN_PV_PIN) //���빦�ʹ�С��������MPPTģʽ��Vin��Ϊ��С
		{
			mppt_up_dn_flag = 0x01;
			pv_ref = MIN_PV_VIN;
		}
		
		/* �����ѹ�趨ֵ�޷� */
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


