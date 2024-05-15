#ifndef __MPPT_H
#define __MPPT_H

#include "at32f403a_407_board.h"

/* =================== define ================== */
#define MPPT_STEP		0.1f  //mppt����,��λV 

#define MIN_PV_PIN	0.0f	 //��СPV����
#define MIN_PV_VIN	12.0f	 //��СPV��ѹ
#define MAX_PV_VIN	30.0f	 //���PV��ѹ

/* ==================== typedef ================= */




/* ====================Function declaration================= */
float mpptmode_PVctr(uint8_t cnt);


#endif

