#ifndef __CTLLOOP_H
#define __CTLLOOP_H	 

#include "at32f403a_407_board.h"

/* V2_BUCK
#define ACQ_SD_PIN                     	 GPIO_PINS_8
#define ACQ_SD_GPIO_PORT               	 GPIOB
#define ACQ_SD_GPIO_CLK                	 CRM_GPIOB_PERIPH_CLOCK

#define ACQ_OPEN_PIN                     GPIO_PINS_9
#define ACQ_OPEN_GPIO_PORT               GPIOB
#define ACQ_OPEN_GPIO_CLK                CRM_GPIOB_PERIPH_CLOCK

#define ACQ_LED1_PIN                     GPIO_PINS_3
#define ACQ_LED1_GPIO_PORT               GPIOA
#define ACQ_LED1_GPIO_CLK                CRM_GPIOA_PERIPH_CLOCK

#define ACQ_LED2_PIN                     GPIO_PINS_4
#define ACQ_LED2_GPIO_PORT               GPIOA
#define ACQ_LED2_GPIO_CLK                CRM_GPIOA_PERIPH_CLOCK
*/

//(x>>n)  ==  x/2^(n)
/* =================== define ================== */
/* 定义偏置电压 */
#define DV_REF		2048 			//(a_Vin_ref*3.3)*Q12	---a_Vin_ref=1.65V

/* 占空比 */
#define MIN_BUKC_DUTY		0.05f		//BUCK最小占空比		10%
#define MAX_BUCK_DUTY 	0.90f	//BUCK最大占空比，	90%

/* 拟合参数 */
/* V2
#define CAL_VOUT_K	1.0095f			//Vin矫正K值
#define CAL_VOUT_B	0.2501f			//Vin矫正B值
#define CAL_VIN_K		0.9961f			//Vout矫正K值
#define CAL_VIN_B		0.2749f			//Vout矫正B值
*/

#define CAL_VOUT_K	 0.9989f		//Vin矫正K值
#define CAL_VOUT_B	 0.0348f
#define CAL_VIN_K		 1.0017f		//Vout矫正K值
#define CAL_VIN_B		-0.0812f		


/*------------ PID参数  ------------*/
//位置式
/*
#define BUCK_VKP    				0.0458f		//电压环PI环路P值，调试数据
#define BUCK_VKI     				0.00047f	//电压环PI环路I值，调试数据
#define BUCK_VKD						0.0f
*/

/* 增量式 */
/* V2 50KHZ的
#define VOUT_KP    				0.04468f		//电压环PI环路P值，调试数据
#define VOUT_KI     			0.0046f	  //电压环PI环路I值，调试数据
#define VOUT_KD						0.00000f
*/

//Vout	V2
/*
#define VOUT_KP    				0.06833f		//电压环PI环路P值，调试数据
#define VOUT_KI     			0.00025f	  //电压环PI环路I值，调试数据
#define VOUT_KD						0.00000f
*/

//Iout V2
/*
Ki 0.00689
#define IOUT_KP    				0.279f		 //电流环PI环路P值，调试数据
#define IOUT_KI     			0.0078f	 //电流环PI环路I值，调试数据
#define IOUT_KD						0.0022f
*/

//Vin 原Vin调参
//#define VIN_KP		  0.008f		
//#define VIN_KI			0.0001f	  
//#define VIN_KD			0.000f

//#define VIN_KP		  0.049f		
//#define VIN_KI			0.000f	  
//#define VIN_KD			0.000f

//#define VIN_KP		  0.00018f		
//#define VIN_KI			0.0000035f	  
//#define VIN_KD			0.000f

#define VIN_KP		  0.07f		
#define VIN_KI			0.00f	  
#define VIN_KD			0.000f

//Vout
#define VOUT_KP			0.05f			
#define VOUT_KI		  0.008f
#define VOUT_KD			0.00f

//Iout 
#define IOUT_KP    	0.013f	
#define IOUT_KI     0.00001f
#define IOUT_KD			0.00f

/*----------- 软件保护参数 -----------*/
//Q12减少运算
#define MAX_SHORT_I     		0		// 短路电流 I*4096/20
#define MIN_SHORT_V     		0   // 短路电压 V*4096/100
#define IOUT_OVER_VAL    		1638   // 过流电流 //8A
#define VOUT_OVER_VAL    		0   // 输出过压
//#define VIN_UNDER_VAL    		614   // 输入欠压 15V欠压点
#define VIN_UNDER_VAL    		491   // 输入欠压	12V欠压点
#define VIN_OVER_VAL				1433		// 输入过压 35V
#define MIN_UVP_VAL_RE  		0  
#define MAX_VIN_OVP_VAL    	0

/* ====================== typedef =================== */
/* 采样变量结构体 */
struct _ADI
{
	int32_t Iout;				//输出 电流
	float   IoutAvg;			//输出 电流 平均值
	int32_t Vout;				//输出 电压
	float   VoutAvg;			//输出 电压 平均值
	int32_t Iin;					//输入 电流
	float   IinAvg;			//输入 电流 平均值
	int32_t Vin;					//输入 电压
	float   VinAvg;			//输出 电压平均值
};

typedef struct _PID{
	float target;
	float value;
	
	float	a0;				//计算值:a0 = Kp(1.0 + T/Ti + Td/T) = Kp+Ki+Kd
	float	a1;				//计算值:a1 = Kp(1.0 + 2*Td/T) = Kp+2Kd
  float a2;       //计算值:a2 = Kp*Td/T = Kd
	
	float Ek_Dead; 	//误差死区
	
	float Ek_0;    //计算值:Error[k-0],本次误差
  float Ek_1;    //计算值:Error[k-1],前一次误差
  float Ek_2;    //计算值:Error[k-2],前二次误差
	
	float	Incem;		//电压增量值
	
	float Output_max;
	float Output_min;
	
	float Output;  //计算值:PID输出值
	
}pid_struct;

struct _DAC_eff
{
	float Vout_eff_k;
	float Vout_eff_b;
	float Vin_eff_k;
	float Vin_eff_b;
	int32_t Iout_offset;
	float Iout_eff_k;
	int32_t Iin_offset;	
	float Iin_eff_k;
};

//环路故障
enum loop_fault
{
	none,
	vin_under,  //输入欠压
	vin_over,	  //输入过压
	vout_over,  //输出过压
	iout_over,	//输出过流
	iout_short, //输出短路
	excu_fault, //零漂检测异常
};

/* ====================function declaration================= */
//void BuckBoard_init(void);
int8_t ExcursionCheck(void);
void ADCSample(uint8_t Iopen);
void increPid_init(void);

#ifdef TEST_DEBUG
	void charger3degreeCtrl(void);	 //电池三段式
#endif

void Buck_LoopControl(void); //环路控制
uint8_t LoopFaultCheck(uint8_t Iopen, uint8_t cnt);//环路保护
uint8_t LoopFaultBack(uint8_t err, uint8_t cnt);//环路检测恢复
	
float f_min(float x, float y);
#endif

