#include "timer.h"
#include "CtlLoop.h"
#include "at32f403a_407_board.h"

/* ---------------------- global ----------------------- */


/* ----------------------  function ------------------ */
void TMR1_Init(void)
{
	uint16_t timer1_period = 0;
	uint16_t channel1_pulse = 0;
	gpio_init_type gpio_init_struct;
	crm_clocks_freq_type crm_clocks_freq_struct = {0};
	
	 /* get system clock */
	crm_clocks_freq_get(&crm_clocks_freq_struct);
	
	/* enable tmr1/gpioa clk */
	crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);		//开启TIM1时钟
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);	//开启GPIOA时钟
	
	/* config TIMR1 io */
	// CH1 
	gpio_init_struct.gpio_pins = GPIO_PINS_8;			
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;		//复用功能
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init(GPIOA, &gpio_init_struct);
	
	// CH1N
	gpio_init_struct.gpio_pins = GPIO_PINS_13;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOB, &gpio_init_struct);		

	/* compute the value to be set in arr regiter to generate signal frequency at 100 khz */
	timer1_period = (crm_clocks_freq_struct.apb2_freq * 2 / 100000 ) - 1;
	/* compute ccr1 value to generate a duty cycle at 10% for channel 1 and 1n */
	channel1_pulse = (uint16_t) (((uint32_t) 1 * (timer1_period - 1)) / 10);

	
	/* TIMR为向上计数模式 */
	tmr_base_init(TMR1, timer1_period, 0);
	tmr_cnt_dir_set(TMR1, TMR_COUNT_UP);
	
	/* config TIMR1 channel */
	tmr_output_config_type tmr_output_struct;
	tmr_output_default_para_init(&tmr_output_struct);
	tmr_output_struct.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
	tmr_output_struct.oc_output_state = FALSE;
	tmr_output_struct.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
  tmr_output_struct.oc_idle_state = TRUE;
	
  tmr_output_struct.occ_output_state = FALSE; 							
  tmr_output_struct.occ_polarity = TMR_OUTPUT_ACTIVE_LOW; 
  tmr_output_struct.occ_idle_state = FALSE; 

	/* config TIMR刹车模式和死区模式 */
/*	tmr_brkdt_config_type tmr_brkdt_config_struct = {0};
	tmr_brkdt_default_para_init(&tmr_brkdt_config_struct);
  tmr_brkdt_config_struct.brk_enable = FALSE; 							
  tmr_brkdt_config_struct.auto_output_enable = TRUE;
  tmr_brkdt_config_struct.deadtime = 12; //开头0xx: DT[7:0]=t*240*e6 //72--300ns //12-50ns
  tmr_brkdt_config_struct.fcsodis_state = TRUE;
  tmr_brkdt_config_struct.fcsoen_state = TRUE;
  tmr_brkdt_config_struct.brk_polarity = TMR_BRK_INPUT_ACTIVE_HIGH;
  tmr_brkdt_config_struct.wp_level = TMR_WP_LEVEL_3;
  tmr_brkdt_config(TMR1, &tmr_brkdt_config_struct);
*/

	// CH1 配置
	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_1, channel1_pulse);
	
	// CH3 配置
	tmr_output_struct.oc_output_state = TRUE;
	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_3, &tmr_output_struct);
  tmr_channel_value_set(TMR1, TMR_SELECT_CHANNEL_3, (channel1_pulse/2) );
	
	TMR1->rpr = 0x04;		//5次响应一次中断
	
	/* 开启定时器溢出中断 */
	tmr_interrupt_enable(TMR1, TMR_OVF_INT, FALSE);
	nvic_irq_enable(TMR1_OVF_TMR10_IRQn, 2, 0);
//	
	/* TIM1 输出总开关打开 */
  tmr_output_enable(TMR1, TRUE);

	/* enable tmr1 */
  tmr_counter_enable(TMR1, TRUE);
	
}

void TMR1_OVF_TMR10_IRQHandler(void)
{
 /* 判断溢出标志位是否置起 */
 if(tmr_flag_get(TMR1, TMR_OVF_FLAG) == SET)
 {
	 Buck_LoopControl();
	
	 tmr_flag_clear(TMR1, TMR_OVF_FLAG);
 }
}
