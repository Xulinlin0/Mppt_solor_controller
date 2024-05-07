#include "filter.h"

/* ==================== global ================= */
/* PIDCtrl低通滤波 */
/*
	[0]	-	Vin
	[1]	- Iin
	[2] - Vout
	[3] - Iout
*/
//截止频率5KHZ，采样频率20KHZ
low_filter_struct  gLowFilterVI[4] = 
{
	{5e3, 20e3, 0 },	
	{5e3, 20e3, 0 },  
	{5e3, 20e3, 0 },
	{5e3, 20e3, 0 }
};

/* MPPTCtrl低通滤波 */
/*
	[0]	-	Vin
	[1]	- Iin
	[2] - Pin
*/
//截止频率10HZ，采样频率100HZ
low_filter_struct gLowFilterMpptPv[3] = 
{
	{10, 100, 0 },	
	{10, 100, 0 },  
	{10, 100, 0 },
};


/*
* ===================================================================
*  Funtion Name : void low_filter_init(low_filter_struct *p)
*  Description :  低通滤波器初始化
*  Parameters  :  被初始化截止频率和采样频率的低通滤波器结构体
*  Returns     :  none
* ===================================================================
*/
void low_filter_init(low_filter_struct *p)
{
  float Tc;//时间常数τ
  
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
*  Description :  低通滤波处理
*  Parameters  :  被初始化截止频率和采样频率的低通滤波器结构体
*  Returns     :  none
* ===================================================================
*/
void low_filter_calc(low_filter_struct *p)
{
  if(p->Output == p->Input)return;

  p->Output = p->Ka * p->Input + p->Kb * p->Output;

}

