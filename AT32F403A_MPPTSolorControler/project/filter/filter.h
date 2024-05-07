#ifndef __FILTER_H
#define __FILTER_H

/* =================== define ================== */
#define PI    3.14159265F // ��
#define PI2   6.28318530F // 2��

/* ==================== typedef ================= */
/*  ��ͨ�˲�������  */
typedef struct
{
  float  Fc;		//��ֹƵ��
  float  Fs;		//����Ƶ��
  float  Ka;		//a
  float  Kb;		//1-a
	
	float  Input; //�����ź�
  float  Output;//����ź�
	
}low_filter_struct;


/* ====================Function declaration================= */
void low_filter_init(low_filter_struct *p);
void low_filter_calc(low_filter_struct *p);

#endif

