#include "charge.h"
#include "CtlLoop.h"

/**************** extern **********************/
extern __IO uint16_t adc1_ordinary_valuetab[4];
extern struct _DAC_eff dac_eff;
extern struct  _ADI SADC;

/**************** global **********************/
/* ��ز�������	*/
BattryPara_TypeDef    gMy_Battry = {
	
  .states_charge     = charge_idle,    //���״̬ 
  .err_code          = none_err,    	 //���ϴ���
  
  .charging_capacity = 20.0f, //�����������λ��AH
	
  .charging_Vo    	 = 14.4f, //��ص�ѹ����λ��V
  .floating_Vo    	 = 13.8f, //�����ѹ����λ��V
  .cutoff_Vo      	 = 10.8f, //��͵�ѹ����λ��V
	
  .precharg_Io     	 = 0.50f, //Ԥ���������λ��A
  .charging_Io       = 2.00f, //����������λ��A
  .floating_Io       = 0.30f, //�����������λ��A
  .stop_Io           = 0.30f, //ֹͣ��������λ��A

  .precharge_time    = 6000,  //Ԥ��ʱ��(10����,��10HZ�жϼ���)
  .charge_time       = 36000, //���ʱ��(60����,��10HZ�жϼ���)
  .floating_time     = 12000 //����ʱ��(20����,��10HZ�жϼ���)
};

/**
 **********************************************************
 *     Funtion Name : uint8_t ChargeStateCalc(BattryPara_TypeDef *bat)
 *     Description :	�������ʽ���ʵ��
 *     Parameters  :
 *     Returns     :	��ص�ǰ״̬
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
		/* ��ؿ��н׶� */
		case charge_idle:		
				bat->charge_current = bat->floating_Io;//�������
				bat->charge_voltage = bat->charging_Vo;//����ѹ
				
				//(Io>=ֹͣ������Vo<����ѹ),����Ԥ��׶�
				swiStaTime = (IoTemp >= bat->stop_Io || VoTemp < bat->charging_Vo) ? (swiStaTime + 1) : 0;
				if (swiStaTime > 10)
				{
					bat->states_charge = charge_precharge;
				}
				
				precharge_time = 0;
				charge_time = 0;
				break;
		/* ���Ԥ��׶� */
		case charge_precharge: 
				bat->charge_current = bat->precharg_Io;//Ԥ�����
				bat->charge_voltage = bat->charging_Vo;//����ѹ
				
				//Vo>��͵�ѹ�� ������׶�
				swiStaTime = (VoTemp > bat->cutoff_Vo) ? (swiStaTime + 1) : 0;
				if (swiStaTime > 10)
				{
					bat->states_charge = charge_charging;
				}
				
				if (precharge_time > bat->precharge_time)//Ԥ�䳬ʱ���������ģʽ
				{
					bat->err_code = pre_timeout;
					bat->states_charge = charge_fault;
				}
				precharge_time++;
				break;
		/* cc-cv����н׶� */
		case charge_charging:
				bat->charge_current = bat->charging_Io;//������
				bat->charge_voltage = bat->charging_Vo;//����ѹ
				
				//Vo>�����ѹ&&Io<ֹͣ���������븡��ģʽ
				swiStaTime = (VoTemp > bat->floating_Vo && IoTemp < bat->stop_Io) ? (swiStaTime + 1) : 0;
				if (swiStaTime > 10)
				{
					bat->states_charge = charge_fault;
				}
				
				if (charge_time > bat->charge_time || VoTemp < bat->cutoff_Vo) //��糬ʱ��Vo<��͵�ѹ������������
				{
					bat->err_code = (charge_time > bat->charge_time) ? ing_timeout : ing_Vocutoff;
					bat->states_charge = charge_fault;
				}
				break;
		/* ��ظ���׶� */
		case charge_float:
				bat->charge_current = bat->floating_Io;//�������
				bat->charge_voltage = bat->floating_Vo;//�����ѹ
				
				//Io<ֹͣ������ֱ������ʱ�䵽���������
				swiStaTime = (IoTemp < bat->stop_Io) ? (swiStaTime + 1) : 0;		
				if (swiStaTime > bat->floating_time)
				{
					bat->states_charge = charge_done;
				}
				
				if (charge_time > bat->charge_time) //��糬ʱ����������
				{
					bat->err_code = float_timeout;
					bat->states_charge = charge_fault;
				}
				charge_time++;
				break;
		/* �����ɽ׶� */
		case charge_done:	
				bat->charge_current = bat->stop_Io;		 //ֹͣ����
				bat->charge_voltage = bat->floating_Vo;//�����ѹ
				if (VoTemp < 0.9f * bat->floating_Vo)  //if�����ѹ<0.9�����ѹ��������ûؿ���״̬
				{
					bat->states_charge = charge_idle;
				}
				break;
		/* �����Ͻ׶� */
		case charge_fault:	
				bat->charge_current = bat->stop_Io; 	 //ֹͣ����
				bat->charge_voltage = bat->floating_Vo;//�����ѹ
				swiStaTime = 0;
				break;
	}
	return bat->states_charge;
}



