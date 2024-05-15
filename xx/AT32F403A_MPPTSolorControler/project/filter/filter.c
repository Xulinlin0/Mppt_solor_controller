#include "filter.h"

/* ==================== global ================= */
/* PIDCtrl��ͨ�˲� */
/*
	[0]	-	Vin
	[1]	- Iin
	[2] - Vout
	[3] - Iout
*/
//��ֹƵ��5KHZ������Ƶ��20KHZ
low_filter_struct  gLowFilterVI[4] = 
{
	{5e3, 20e3, 0 },	
	{5e3, 20e3, 0 },  
	{5e3, 20e3, 0 },
	{5e3, 20e3, 0 }
};

/* MPPTCtrl��ͨ�˲� */
/*
	[0]	-	Vin
	[1]	- Iin
	[2] - Pin
*/
//��ֹƵ��10HZ������Ƶ��100HZ
low_filter_struct gLowFilterMpptPv[3] = 
{
	{10, 100, 0 },	
	{10, 100, 0 },  
	{10, 100, 0 },
};


/*
* ===================================================================
*  Funtion Name : void low_filter_init(low_filter_struct *p)
*  Description :  ��ͨ�˲�����ʼ��
*  Parameters  :  ����ʼ����ֹƵ�ʺͲ���Ƶ�ʵĵ�ͨ�˲����ṹ��
*  Returns     :  none
* ===================================================================
*/
void low_filter_init(low_filter_struct *p)
{
  float Tc;//ʱ�䳣����
  
  if( p->Fc <= 0.0f || p->Fs <= 0.0f )
  {
      p->Ka      = 1.0f;
      p->Kb      = 0.0f;
      return;
  }

  Tc         = 1.0f / (PI2 * p->Fc);

  p->Ka      = 1.0f / (1.0f + Tc * p->Fs);
  p->Kb      = 1.0f - p->Ka;
  p->Input   = 0.0f;
  p->Output  = 0.0f;
}

/*
* ===================================================================
*  Funtion Name : void low_filter_calc(low_filter_struct *p)
*  Description :  ��ͨ�˲�����
*  Parameters  :  ����ʼ����ֹƵ�ʺͲ���Ƶ�ʵĵ�ͨ�˲����ṹ��
*  Returns     :  none
* ===================================================================
*/
void low_filter_calc(low_filter_struct *p)
{
  if(p->Output == p->Input)return;

  p->Output = p->Ka * p->Input + p->Kb * p->Output;

}

