/**
  **************************************************************************
  * @file     at32f403a_407_board.c
  * @brief    set of firmware functions to manage leds and push-button.
  *           initialize delay function.
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

#include "at32f403a_407_board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

/** @addtogroup AT32F403A_407_board
  * @{
  */

/** @defgroup BOARD
  * @brief onboard periph driver
  * @{
  */
	
/* delay variable */
static __IO uint32_t dwt_fac_us;
static __IO uint32_t dwt_fac_ms;

/* support printf function, usemicrolib is unnecessary */
#if (__ARMCC_VERSION > 6000000)
  __asm (".global __use_no_semihosting\n\t");
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
  FILE __stdout;
#else
 #ifdef __CC_ARM
  #pragma import(__use_no_semihosting)
  struct __FILE
  {
    int handle;
  };
  FILE __stdout;
  void _sys_exit(int x)
  {
    x = x;
  }
  /* __use_no_semihosting was requested, but _ttywrch was */
  void _ttywrch(int ch)
  {
    ch = ch;
  }
 #endif
#endif

#if defined (__GNUC__) && !defined (__clang__)
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
  * @brief  retargets the c library printf function to the usart.
  * @param  none
  * @retval none
  */
PUTCHAR_PROTOTYPE
{
  while(usart_flag_get(UART5, USART_TDBE_FLAG) == RESET);
  usart_data_transmit(UART5, (uint16_t)ch);
  while(usart_flag_get(UART5, USART_TDC_FLAG) == RESET);
	
  return ch;
}

#if (defined (__GNUC__) && !defined (__clang__)) || (defined (__ICCARM__))
#if defined (__GNUC__) && !defined (__clang__)
int _write(int fd, char *pbuffer, int size)
#elif defined ( __ICCARM__ )
#pragma module_name = "?__write"
int __write(int fd, char *pbuffer, int size)
#endif
{
  for(int i = 0; i < size; i ++)
  {
    while(usart_flag_get(PRINT_UART, USART_TDBE_FLAG) == RESET);
    usart_data_transmit(PRINT_UART, (uint16_t)(*pbuffer++));
    while(usart_flag_get(PRINT_UART, USART_TDC_FLAG) == RESET);
  }

  return size;
}
#endif


/****************************  usart  *****************************/
void uart5_init(void)
{
  gpio_init_type gpio_init_struct;

#if defined (__GNUC__) && !defined (__clang__)
  setvbuf(stdout, NULL, _IONBF, 0);
#endif
	
	/* enable iomux periph clock */
	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);//UART5是端口复用，记得开启端口复用时钟！
  /* enable the uart and gpio clock */
  crm_periph_clock_enable(CRM_UART5_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

	
  gpio_default_para_init(&gpio_init_struct);

  /* configure the uart tx pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_9;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

	/* configure the uart rx pin */
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_8;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  gpio_pin_remap_config(UART5_GMUX_0001, TRUE);
	
//	/* config usart nvic interrupt */
//	usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);//开启接收中断
//	nvic_irq_enable(USART1_IRQn, 8 ,0);

  /* configure uart param */
  usart_init(UART5, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
	usart_parity_selection_config(UART5, USART_PARITY_NONE);
  usart_hardware_flow_control_set(UART5, USART_HARDWARE_FLOW_NONE);
	
  usart_transmitter_enable(UART5, TRUE);
//	usart_receiver_enable(USART1, TRUE);

  usart_enable(UART5, TRUE);
}

/****************************  led  *****************************/
void at32_led_init(void)
{
  gpio_init_type gpio_init_struct;

  /* enable the led clock */
	crm_periph_clock_enable(LED_GPIO_CRM_CLK, TRUE);

  /* set default parameter */
  gpio_default_para_init(&gpio_init_struct);

  /* configure the led gpio */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
  gpio_init_struct.gpio_pins = LED1_PIN | LED2_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(LED_GPIO, &gpio_init_struct);
	
	LED_GPIO->scr = LED1_PIN;
	LED_GPIO->scr = LED2_PIN;
}

void at32_led_on(uint8_t lednum)
{
  switch (lednum)
	{
		case LED1:	LED_GPIO->clr = LED1_PIN;	break;
		case LED2:	LED_GPIO->clr = LED2_PIN;	break;
	}
}

void at32_led_off(uint8_t lednum)
{
	 switch (lednum)
	{
		case LED1:	LED_GPIO->scr = LED1_PIN;	break;
		case LED2:	LED_GPIO->scr = LED2_PIN;	break;
	}
}
	
void at32_led_toggle(uint8_t lednum)
{
	switch (lednum)
	{
		case LED1:	LED_GPIO->odt ^= LED1_PIN;	break;
		case LED2:	LED_GPIO->odt ^= LED2_PIN;	break;
	}
}

/****************************  dwt delay  *****************************/
/**
  * @brief  initialize dwt delay function   
  * @param  none
  * @retval none
  */
void dwt_delay_init()
{
	crm_clocks_freq_type crm_clocks_freq_struct = {0};
	/* get system clock */
  crm_clocks_freq_get(&crm_clocks_freq_struct);

	/* configure dwt */	
  dwt_fac_us = crm_clocks_freq_struct.sclk_freq / (1000000U);
  dwt_fac_ms = dwt_fac_us * (1000U);	
}

/**
  * @brief  inserts a dwt delay time.
  * @param  nus: specifies the dwt delay time length, in microsecond.
  * @retval none
  */
void dwt_delay_us(uint32_t nus)
{
	/* Enable CoreDebug DEMCR bit24: TRCENA */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
 	/* Clear CYCCNT */
	DWT->CYCCNT = 0x00;
 	/* Enable DWT CTRL bit0: CYCCNTENA */	
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	uint32_t temp = DWT->CYCCNT;
  while((DWT->CYCCNT - temp) < nus * dwt_fac_us);

 	/* Disable DWT CTRL bit0: CYCCNTENA */	
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
 	/* Clear CYCCNT */
	DWT->CYCCNT = 0x00;
	/* Disable CoreDebug DEMCR bit24: TRCENA */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
}

/**
  * @brief  inserts a dwt delay time.
  * @param  nms: 0x0000~0x45EF, specifies the dwt delay time length, in milliseconds.
  * @retval none
  */
void dwt_delay_ms(uint16_t nms)
{
	/* Enable CoreDebug DEMCR bit24: TRCENA */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
 	/* Clear CYCCNT */
	DWT->CYCCNT = 0x00;
 	/* Enable DWT CTRL bit0: CYCCNTENA */	
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	
	uint32_t temp = DWT->CYCCNT;
  while((DWT->CYCCNT - temp) < nms * dwt_fac_ms);

 	/* Disable DWT CTRL bit0: CYCCNTENA */	
	DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
 	/* Clear CYCCNT */
	DWT->CYCCNT = 0x00;
	/* Disable CoreDebug DEMCR bit24: TRCENA */
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
}

/**
  * @brief  inserts a dwt delay time.
  * @param  sec: specifies the dwt delay time, in seconds.
  * @retval none
  */
void dwt_delay_sec(uint16_t sec)
{
  uint16_t index;
  for(index = 0; index < sec; index++)
  {
    dwt_delay_ms(1000);
  }
}

/****************************  key  *****************************/
void at32_key_init(void)
{
	gpio_init_type gpio_init_struct;
	
	/* enable the key clock */
	crm_periph_clock_enable(KEY_GPIO_CRM_CLK, TRUE);
	
	/* set default parameter */
  gpio_default_para_init(&gpio_init_struct);
	
	/* configure the key gpio */
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = KEY1_PIN | KEY2_PIN;
  gpio_init_struct.gpio_pull = GPIO_PULL_UP;
  gpio_init(KEY_GPIO, &gpio_init_struct);
}


uint8_t Key_Scan(void)
{	
	/*检测是否有按键按下 */
	if (!gpio_input_data_bit_read(KEY_GPIO,KEY1_PIN) || !gpio_input_data_bit_read(KEY_GPIO,KEY2_PIN))  
	{	 
		dwt_delay_ms(100);
		if (!gpio_input_data_bit_read(KEY_GPIO,KEY1_PIN))	return KEY1;
		else if (!gpio_input_data_bit_read(KEY_GPIO,KEY2_PIN))	return KEY2;
	}
	return 0;
}

//u8 KEY_Scan(u8 mode)
//{
//static u8 key_up=1;//按键按松开标志
//if(mode)key_up=1; //支持连按 
//if(key_up&&(KEY0==0||KEY1==0||WK_UP==1))
//{
//delay_ms(10);//去抖动
//key_up=0;
//if(KEY0==0)return KEY0_PRES;
//else if(KEY1==0)return KEY1_PRES;
//else if(WK_UP==1)return WKUP_PRES;
//}else if(KEY0==1&&KEY1==1&&WK_UP==0)key_up=1; 
//return 0;// 无按键按下
//}
