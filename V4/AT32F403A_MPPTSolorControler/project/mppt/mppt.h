#ifndef __MPPT_H
#define __MPPT_H

#include "at32f403a_407_board.h"

/* =================== define ================== */
#define MPPT_STEP		0.1f  //mppt步长,单位V 

#define MIN_PV_PIN	0.0f	 //最小PV功率
#define MIN_PV_VIN	12.0f	 //最小PV电压
#define MAX_PV_VIN	30.0f	 //最大PV电压

/* ==================== typedef ================= */




/* ====================Function declaration================= */
float mpptmode_PVctr(uint8_t cnt);


#endif

