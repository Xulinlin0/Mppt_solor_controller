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
/* ����ƫ�õ�ѹ */
#define DV_REF		2048 			//(a_Vin_ref*3.3)*Q12	---a_Vin_ref=1.65V

/* ռ�ձ� */
#define MIN_BUKC_DUTY		0.05f		//BUCK��Сռ�ձ�		10%
#define MAX_BUCK_DUTY 	0.90f	//BUCK���ռ�ձȣ�	90%

/* ��ϲ��� */
/* V2
#define CAL_VOUT_K	1.0095f			//Vin����Kֵ
#define CAL_VOUT_B	0.2501f			//Vin����Bֵ
#define CAL_VIN_K		0.9961f			//Vout����Kֵ
#define CAL_VIN_B		0.2749f			//Vout����Bֵ
*/

//V3
//#define CAL_VOUT_K	 0.9989f		//Vout����Kֵ
//#define CAL_VOUT_B	 0.0348f
//#define CAL_VIN_K		 1.0017f		//Vin����Kֵ
//#define CAL_VIN_B		-0.0812f		

#define CAL_VOUT_K	  0.9875f		//Vout����Kֵ
#define CAL_VOUT_B	 -0.1314f
#define CAL_VIN_K		  0.9858f		//Vin����Kֵ
#define CAL_VIN_B		 -0.0483f		

/*------------ PID����  ------------*/
/*------ V1��	-------*/
//λ��ʽ
/*
#define BUCK_VKP    				0.0458f		//��ѹ��PI��·Pֵ����������
#define BUCK_VKI     				0.00047f	//��ѹ��PI��·Iֵ����������
#define BUCK_VKD						0.0f
*/

/*------ V2��	-------*/
/* ����ʽ */
/* V2 ����f��50KHZ �����pwm
#define VOUT_KP    				0.04468f		//��ѹ��PI��·Pֵ����������
#define VOUT_KI     			0.0046f	  //��ѹ��PI��·Iֵ����������
#define VOUT_KD						0.00000f
*/

//Vout	V2	�����pwm
/*
#define VOUT_KP    				0.06833f		//��ѹ��PI��·Pֵ����������
#define VOUT_KI     			0.00025f	  //��ѹ��PI��·Iֵ����������
#define VOUT_KD						0.00000f
*/

//Iout V2
/*
Ki 0.00689
#define IOUT_KP    				0.279f		 //������PI��·Pֵ����������
#define IOUT_KI     			0.0078f	 //������PI��·Iֵ����������
#define IOUT_KD						0.0022f
*/
/*------ V3��	-------*/
//Vin 	����f��100KHZ	�����pwm
/*
#define VIN_KP		  0.00018f		
#define VIN_KI			0.0000035f	  
#define VIN_KD			0.000f
*/

//Vout	����f��100KHZ	�����pwm
//#define VOUT_KP_S		0.05f			
//#define VOUT_KI_S		0.008f
//#define VOUT_KD_S		0.00f

/*------ V4��	-------*/
//Vout	����f��100KHZ	�����pwm
#define VOUT_KP_S		0.052f			//0.052 	12V
#define VOUT_KI_S		0.0012f			//0.001	12V
#define VOUT_KD_S		0.00f

//Vin		����f��100KHZ	�����Iout
#define VIN_KP		  0.32f		
#define VIN_KI			0.005f	  
#define VIN_KD			0.000f

//Vout	����f��100KHZ	�����Iout
#define VOUT_KP			0.098f			
#define VOUT_KI		  0.0013f
#define VOUT_KD			0.00f

//Iout	����f��100KHZ	�����pwm
//#define IOUT_KP    	0.013f	
#define IOUT_KP    	0.0131f	
#define IOUT_KI     0.000015f	
#define IOUT_KD			0.00f

/*----------- ����������� -----------*/
//Q12��������
#define MAX_SHORT_I     		0		// ��·���� I*4096/20
#define MIN_SHORT_V     		0   // ��·��ѹ V*4096/100
#define IOUT_OVER_VAL    		1638   // �������� //8A
#define VOUT_OVER_VAL    		0   // �����ѹ
//#define VIN_UNDER_VAL    		614   // ����Ƿѹ 15VǷѹ��
#define VIN_UNDER_VAL    		491   // ����Ƿѹ	12VǷѹ��
#define VIN_OVER_VAL				1638		// �����ѹ 40V
#define MIN_UVP_VAL_RE  		0  
#define MAX_VIN_OVP_VAL    	0

/* ====================== typedef =================== */
/* ���������ṹ�� */
struct _ADI
{
	int32_t Iout;				//��� ����
	float   IoutAvg;			//��� ���� ƽ��ֵ
	int32_t Vout;				//��� ��ѹ
	float   VoutAvg;			//��� ��ѹ ƽ��ֵ
	int32_t Iin;					//���� ����
	float   IinAvg;			//���� ���� ƽ��ֵ
	int32_t Vin;					//���� ��ѹ
	float   VinAvg;			//��� ��ѹƽ��ֵ
};

typedef struct _PID{
	float target;
	float value;
	
	float	a0;				//����ֵ:a0 = Kp(1.0 + T/Ti + Td/T) = Kp+Ki+Kd
	float	a1;				//����ֵ:a1 = Kp(1.0 + 2*Td/T) = Kp+2Kd
  float a2;       //����ֵ:a2 = Kp*Td/T = Kd
	
	float Ek_Dead; 	//�������
	
	float Ek_0;    //����ֵ:Error[k-0],�������
  float Ek_1;    //����ֵ:Error[k-1],ǰһ�����
  float Ek_2;    //����ֵ:Error[k-2],ǰ�������
	
	float	Incem;		//��ѹ����ֵ
	
	float Output_max;
	float Output_min;
	
	float Output;  //����ֵ:PID���ֵ
	
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

//��·����
enum loop_fault
{
	none,
	vin_under,  //����Ƿѹ
	vin_over,	  //�����ѹ
	vout_over,  //�����ѹ
	iout_over,	//�������
	iout_short, //�����·
	excu_fault, //��Ư����쳣
};

/* ====================function declaration================= */
//void BuckBoard_init(void);
int8_t ExcursionCheck(void);
void ADCSample(uint8_t Iopen);
void increPid_init(void);

void Buck_LoopControl_charger(void); //��س�绷·����
void Buck_LoopControl_supply(void); //�ɵ�ֱ��Դ�����·����
uint8_t LoopFaultCheck(uint8_t Iopen, uint8_t cnt);//��·����
uint8_t LoopFaultBack(uint8_t err, uint8_t cnt);//��·���ָ�
	
float f_min(float x, float y);
#endif

