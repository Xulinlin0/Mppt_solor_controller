/* ---------------------- define ----------------------- */
#define RTT_DEBUG


/* ---------------------- include ----------------------- */
/* stander include*/
#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "string.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "printf-stdarg.h"

/* Demo Hardware includes. */
//#include "soft_iic.h"
#include "timer.h"
#include "adc.h"

/* Demo app includes. */
//#include "ina226.h"
#include "oled.h"
#include "CtlLoop.h"
#include "filter.h"
#include "mppt.h"
#include "charge.h"
#include "esp8266.h"

#ifdef RTT_DEBUG
	#include "SEGGER_RTT.h"
#endif

/* ---------------------- extern ----------------------- */
// 滤波器
extern low_filter_struct gLowFilterMpptPv[3];
extern low_filter_struct  gLowFilterVI[4];
// pid controller
extern pid_struct pid_ctrol[4];

extern BattryPara_TypeDef   gMy_Battry;
extern char uart_tx_parm[150];
extern enum loop_fault loop_err;
extern struct  _ADI SADC;
extern float pwm;

extern float pv_Pin;
extern uint8_t	CV_flag;

extern int8_t Netbutton;
/* ---------------------- global ----------------------- */
TaskHandle_t sStateInit_handler;
TaskHandle_t sStateFault_handler;
TaskHandle_t sStateRun_handler;

// mode flag
uint8_t	DeviceSwitch = 0;
uint8_t	chargerMode = 1;
uint8_t	mppt_enable = 1;

/*
	[0]-Vin		Pin
	[1]-Iin		Pout
	[2]-Vout	Peff
  [3]-Iout 
*/
char datTostr[4][6] = {0};

/* ---------------------- global function ---------------- */
void vApplicationIdleHook()
{
	static uint16_t i = 0xffff;
	if (0 == i--)
	{
		i = 0xffff;
		SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_BLUE"IDLE\r\n");
	}
}

/* ---------------------- task function ---------------- */

/*3 INIT*/
void sStateInit_task_function(void *pvParameters)
{
	int8_t excu_result = 0;
	
	vTaskDelay(100);
	while (KEY2 != Key_Scan())
	{
		if (Netbutton == 1)
		{
			Netbutton = -1;
			break;
		}
	};
	DeviceSwitch = 1;
	
	while (1)
	{	
		/* 零漂检测 */
		excu_result = ExcursionCheck();
		
		if (LoopFaultCheck(0, 60) != none) //环路保护
		{
			DeviceSwitch = 0;
			xTaskNotifyGive(sStateFault_handler);
			ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		}
				
		if (excu_result == 1)
		{
			/* 初始化PIDctrl低通滤波器 */
			for(uint8_t i=0; i<4; i++)
			{
				low_filter_init(&gLowFilterVI[i]);
			}
			
			/* 初始化mpptctrl低通滤波器 */
			for(uint8_t i=0; i<3; i++)
			{
				low_filter_init(&gLowFilterMpptPv[i]);
			}
			
			/* 初始化pid参数 */
			increPid_init();	
		
			tmr_channel_enable(TMR1,TMR_SELECT_CHANNEL_1, TRUE);	//打开TIMR1ch1，输出pwm
			tmr_channel_enable(TMR1,TMR_SELECT_CHANNEL_1C, TRUE);	//打开TIMR1ch1，输出pwm
			tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);				//打开TIMR1中断，开启PID控制

//			SEGGER_RTT_printf("Iin_offset= %d,Iout_offset = %d\r\n",dac_eff.Iin_offset, dac_eff.Iout_offset);
				
			xTaskNotifyGive(sStateRun_handler);
			ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		}
		else if(excu_result == -1)
		{
			loop_err = excu_fault;
			DeviceSwitch  = 0;
			xTaskNotifyGive(sStateFault_handler);
			ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		}
		
		vTaskDelay(5);
	}
}

/*2 FAULT*/
void sStateFault_task_function(void *pvParameters)
{
	uint8_t fault_back = 0;
	ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
	
	while (1)
	{
		at32_led_on(LED1);
		at32_led_off(LED2);
		
		tmr_interrupt_enable(TMR1, TMR_OVF_INT, FALSE);				//打开TIMR1中断，开启PID控制
		tmr_channel_enable(TMR1,TMR_SELECT_CHANNEL_1, FALSE);	//打开TIMR1ch1，输出pwm
		tmr_channel_enable(TMR1,TMR_SELECT_CHANNEL_1C, FALSE);	//打开TIMR1ch1，输出pwm
		
		SEGGER_RTT_printf(1,"loop_err = %d, chanrge_err = %d\r\n", loop_err, gMy_Battry.err_code);
		
		if (loop_err)
		{
			if (none == LoopFaultBack(loop_err, 3))
			{
				fault_back = 1;
			}
		}
		
		if ((KEY2 == Key_Scan()) || fault_back || (Netbutton == 1))//接收到了物联网启动按键发送信息/按键触发恢复故障/环路自检恢复
		{
			Netbutton = -1;
			DeviceSwitch = 1;
			fault_back = 0;
			loop_err = none;	
			gMy_Battry.err_code = none_err;
			
			TMR1->c1dt = (uint16_t)(0.1 * 2398);
			TMR1->c3dt = (uint16_t)(0.05 * 2398);
			
			xTaskNotifyGive(sStateInit_handler);
			ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		}	
		vTaskDelay(100);
	}
}

float target = 0;
/*1 RUN*/
/* sStateRun_task_function */
void sStateRun_task_function(void *pvParameters)
{
	ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
	
	while (1)
	{
		at32_led_on(LED2);
	  at32_led_off(LED1);
		
		//环路保护
		if (LoopFaultCheck(1, 3) != none)
		{
			DeviceSwitch = 0;
			xTaskNotifyGive(sStateFault_handler);
			ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		}
		
		if (chargerMode)//若为电池充电模式
		{
			if (charge_fault == ChargeStateCalc(&gMy_Battry))
			{
				DeviceSwitch = 0;
				xTaskNotifyGive(sStateFault_handler);
				ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
			}
			else
			{
				pid_ctrol[2].target = gMy_Battry.charge_voltage; 	 //停止电流
				pid_ctrol[2].Output_max = gMy_Battry.charge_current; 
			}
		}
		
		if (mppt_enable)
		{
			pid_ctrol[0].value = mpptmode_PVctr(1);	
			target = pid_ctrol[0].value;//1111
			pid_ctrol[0].Output_max = gMy_Battry.charge_current; 
		}
		
		SEGGER_RTT_printf(0,"%f\r\n", pid_ctrol[3].target);
		
		if ((Netbutton == 0) || KEY1 == Key_Scan())	//物联网传输关闭输出或按下KEY1
		{
			Netbutton = -1;
			DeviceSwitch = 0;
			xTaskNotifyGive(sStateFault_handler);
			ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		}
		
		vTaskDelay(100);//10KHZ调用频率
	}
}

/*1 SHOW */
void show_task_function(void *pvParameters)
{
	float pv_Pin = 0, buck_Pout = 0, P_eff = 0;
	
	tiny_sprintf(uart_tx_parm, "{\\\"DeviceSwitch\\\":0\\,\\\"loop_fault\\\":0\\,\\\"charger_fault\\\":0\\,\\\"mppt_enable\\\":%d\\,\\\"chargerMode\\\":%d}", mppt_enable, chargerMode);
	esp8266_pub_data(uart_tx_parm);
	vTaskDelay(200);
	
	flTstr_FirpTwo(gMy_Battry.floating_Io, datTostr[0]);
	flTstr_FirpTwo(gMy_Battry.charging_Io, datTostr[1]);
	flTstr_FirpTwo(gMy_Battry.precharg_Io, datTostr[2]);
	flTstr_FirpTwo(gMy_Battry.stop_Io, datTostr[3]);
	tiny_sprintf(uart_tx_parm, "{\\\"floating_Io\\\":%s\\,\\\"charging_Io\\\":%s\\,\\\"precharg_Io\\\":%s\\,\\\"stop_Io\\\":%s}", datTostr[0], datTostr[1], datTostr[2], datTostr[3]);
	esp8266_pub_data(uart_tx_parm);
	memset(datTostr, 0, sizeof(datTostr));
	vTaskDelay(200);
	
	flTstr_FirpTwo(gMy_Battry.cutoff_Vo, datTostr[0]);
	flTstr_FirpTwo(gMy_Battry.floating_Vo, datTostr[1]);
	flTstr_FirpTwo(gMy_Battry.charging_Vo, datTostr[2]);
	tiny_sprintf(uart_tx_parm, "{\\\"cutoff_Vo\\\":%s\\,\\\"floating_Vo\\\":%s\\,\\\"charging_Vo\\\":%s}",  datTostr[0], datTostr[1],  datTostr[2]);
	memset(datTostr, 0, sizeof(datTostr));
	esp8266_pub_data(uart_tx_parm);
	vTaskDelay(200);
	
  while(1)
  {	
		tiny_sprintf(uart_tx_parm, "{\\\"DeviceSwitch\\\":%d\\,\\\"loop_fault\\\":%d\\,\\\"charger_fault\\\":%d}", DeviceSwitch, loop_err, gMy_Battry.err_code);
		esp8266_pub_data(uart_tx_parm);
		vTaskDelay(600);
		
		/* oled show1 */
		GUI_DrawLine(0, 15, 127, 15,1);
		GUI_DrawLine(60, 0, 60, 63,1);
		if (mppt_enable)
		{
			GUI_ShowString(0, 0, "MODE:MPPT", 1);
		}
		else
		{
			GUI_ShowString(0, 0, "MODE:PWM ", 1);
		}
		
		if (flTstr_FirpTwo(SADC.VinAvg, datTostr[0]) && flTstr_FirpTwo(SADC.VoutAvg, datTostr[2]))
		{
			tiny_sprintf(uart_tx_parm, "Vin:%s  Vout:%s", datTostr[0], datTostr[2]);
			GUI_ShowString(0, 24, uart_tx_parm, 1);
		}
		if (flTstr_FirpTwo(SADC.IinAvg, datTostr[1]) && flTstr_FirpTwo(SADC.IoutAvg, datTostr[3]))
		{
			tiny_sprintf(uart_tx_parm, "Iin:%s  Iout:%s", datTostr[1], datTostr[3]);
			GUI_ShowString(0, 40, uart_tx_parm, 1);
		}
		
		/* LOT transmit1 */
		tiny_sprintf(uart_tx_parm, "{\\\"Vin_value\\\":%s\\,\\\"Iin_value\\\":%s\\,\\\"Vout_value\\\":%s\\,\\\"Iout_value\\\":%s}", datTostr[0], datTostr[1], datTostr[2], datTostr[3]);
		esp8266_pub_data(uart_tx_parm);
		
		memset(datTostr, 0, sizeof(datTostr));
		vTaskDelay(600);
		
		pv_Pin = SADC.IinAvg * SADC.VinAvg;
		buck_Pout = SADC.IoutAvg * SADC.VoutAvg;
		P_eff = f_min(buck_Pout/pv_Pin * 100, 99.99);
		/* oled show2 */
		if (flTstr_FirpTwo(pv_Pin, datTostr[0]) && flTstr_FirpTwo(buck_Pout, datTostr[1]) && flTstr_FirpTwo(P_eff, datTostr[2]))
		{
			tiny_sprintf(uart_tx_parm, "Pin:%s  Pout:%s",datTostr[0], datTostr[1]);
			GUI_ShowString(0, 56, uart_tx_parm,1);
			tiny_sprintf(uart_tx_parm, "Peff:%s", datTostr[2]);
			GUI_ShowString(66, 0, uart_tx_parm, 1);
		}
		
		/* LOT transmit2 */
		tiny_sprintf(uart_tx_parm, "{\\\"Pin_value\\\":%s\\,\\\"Pout_value\\\":%s\\,\\\"P_eff\\\":%s}", datTostr[0], datTostr[1], datTostr[2]);
		esp8266_pub_data(uart_tx_parm);

		memset(datTostr, 0, sizeof(datTostr));
		vTaskDelay(600);
  }
}

/* ---------------------- MAIN ----------------------------- */
void SetupHardware()
{
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  system_clock_config();
	
	dwt_delay_init();

	// BuckBoard_init();				/* init BuckBoard --- first! */
  at32_led_init();  					/* init led2 */
	
	at32_led_on(LED1);
	
	at32_key_init();						/* init key */
	
	ADC_INIT();									/* init adc */
	TMR1_Init();								/* init timer */
	
	while (esp8266_Init())			/* init esp8266 */
	{
		dwt_delay_ms(100);
	}
	
	OLED_Init();	  		 				/* init oled */
	
	at32_led_on(LED2);	
}

void start_task_function(void *pvParameters)
{
  taskENTER_CRITICAL(); /* enter critical */
	
  xTaskCreate(sStateInit_task_function,"Init_task", (uint16_t)100, NULL, 3, &sStateInit_handler);
  xTaskCreate(sStateFault_task_function,"Fault_task", (uint16_t)100, NULL, 2, &sStateFault_handler);
  xTaskCreate(sStateRun_task_function,"Run_task", (uint16_t)100, NULL, 1, &sStateRun_handler);
  xTaskCreate(show_task_function,"show_task", (uint16_t)500, NULL, 1, NULL);
	
	vTaskDelete(NULL);
	
  taskEXIT_CRITICAL(); /* exit critical */

}

int main(void)
{
	SetupHardware(); /* setup Hardware */
	
  xTaskCreate((TaskFunction_t )start_task_function,(const char*)"start_task",(uint16_t)512,(void*)NULL,(UBaseType_t)1,(TaskHandle_t*  )NULL);/* create start task */
	
  vTaskStartScheduler(); /* start scheduler */
}


