#ifndef __FILTER_H
#define __FILTER_H

/* =================== define ================== */
#define PI    3.14159265F // π
#define PI2   6.28318530F // 2π

/* ==================== typedef ================= */
/*  低通滤波器数据  */
typedef struct
{
  float  Fc;		//截止频率
  float  Fs;		//采样频率
  float  Ka;		//a
  float  Kb;		//1-a
	
	float  Input; //输入信号
  float  Output;//输出信号
	
}low_filter_struct;


/* ====================Function declaration================= */
void low_filter_init(low_filter_struct *p);
void low_filter_calc(low_filter_struct *p);

#endif

