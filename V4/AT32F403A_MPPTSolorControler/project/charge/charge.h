#ifndef __CHARGE_H
#define __CHARGE_H

#include "at32f403a_407_board.h"

/* =================== define ================== */


/* ==================== typedef ================= */
enum Charge_States
{
	charge_idle,        //��ؿ���
	charge_fault,       //��ع���
	charge_done,        //��س�����
	charge_precharge,   //���Ԥ��״̬
	charge_charging,	  //���cc-cv���
	charge_float        //��ظ���״̬
};

enum charger_fault
{
	none_err,			//�޴���
	pre_timeout,  //Ԥ�䳬ʱ
	ing_timeout,  //���׶γ�ʱ
	ing_Vocutoff, //���׶γ���Vo������͵�ѹ
	float_timeout,//����׶γ�糬ʱ
};

typedef struct _ChargePara{
	enum Charge_States		states_charge; //���״̬
	enum charger_fault		err_code;           //���ϴ���
	
	float  charging_capacity;		  //�������
	
	float  charging_Vo;						//��س���ѹ
	float  floating_Vo;						//��ظ����ѹ
	float  cutoff_Vo;							//��ؽ�ֹ��ѹ
		
	float  precharg_Io;						//���Ԥ�����
	float  charging_Io;						//��س�����
	float  floating_Io;						//��ظ������
	float  stop_Io;								//���ֹͣ����
	
	float  charge_voltage;      	//�������ѹ
	float  charge_current;      	//���������
	
	unsigned int  precharge_time;	//Ԥ��ʱ��
	unsigned int  charge_time;		//���ʱ��
	unsigned int  floating_time;	//����ʱ��

}BattryPara_TypeDef;

/* ====================function declaration================= */
uint8_t ChargeStateCalc(BattryPara_TypeDef *bat);


#endif
