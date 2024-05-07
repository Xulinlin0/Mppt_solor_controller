#include "adc.h"

/* ---------------------- global ----------------------- */
/* ADC采样数据 */
/*
	[0]	-	Vin
	[1]	- Iin
	[2] - Vout
	[3] - Iout
*/
__IO uint16_t adc1_ordinary_valuetab[NOFCHANEL] = {0};

/* ---------------------- function ------------------ */
void ADC_INIT(void)
{
	/************ gpio config ***************/
	gpio_init_type gpio_initstructure;
  crm_periph_clock_enable(ADC_GPIO_bOUT_CLK, TRUE);
  crm_periph_clock_enable(ADC_GPIO_bIN_CLK, TRUE);
	
  gpio_default_para_init(&gpio_initstructure);
  gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
  gpio_initstructure.gpio_pins = ADC_bVout | ADC_bIout;
	gpio_init(ADC_PORT_bOUT, &gpio_initstructure);
	
	gpio_initstructure.gpio_pins = ADC_bVin | ADC_bIin;
  gpio_init(ADC_PORT_bIN, &gpio_initstructure);
  
	
	/************ dma config ***************/
	dma_init_type dma_init_struct;
  crm_periph_clock_enable(ADC_DMA_CLK, TRUE);
	
  dma_reset(ADC_DMA_CHANNEL);																								 // 复位 DMA 控制器
	
  dma_default_para_init(&dma_init_struct);	
	dma_init_struct.peripheral_base_addr = (uint32_t)&(ADCx->odt); 						 // 外设基址为：ADC 数据寄存器地址
	dma_init_struct.memory_base_addr = (uint32_t)adc1_ordinary_valuetab;  		 // 存储器地址
	dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY; 								 // 数据源来自外设
	dma_init_struct.buffer_size = NOFCHANEL;														  		 // 缓冲区大小，应该等于数据目的地的大小
	dma_init_struct.peripheral_inc_enable = FALSE;											 			 // 外设寄存器只有一个，地址不用递增
  dma_init_struct.memory_inc_enable = TRUE;																	 // 存储器地址递增
	dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;// 外设数据大小为半字，即两个字节
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;				 // 内存数据大小也为半字，跟外设数据大小相同 
  dma_init_struct.loop_mode_enable = TRUE;																	 // 循环传输模式
  dma_init_struct.priority = DMA_PRIORITY_LOW;															 // DMA 传输通道优先级为高，当使用一个 DMA 通道时，优先级设置不影响
  dma_init(ADC_DMA_CHANNEL, &dma_init_struct);															 // 初始化 DMA
	
  dma_channel_enable(ADC_DMA_CHANNEL, TRUE);																 // 使能 DMA 通道
	
	/************ adc mode config ***************/
	adc_base_config_type adc_base_struct;
  crm_periph_clock_enable(ADC_CLK, TRUE);
	
  crm_adc_clock_div_set(CRM_ADC_DIV_6);												//  120/6=20M < 28M
	
	adc_combine_mode_select(ADC_INDEPENDENT_MODE);							// 独立模式
  adc_base_default_para_init(&adc_base_struct); 
	
  adc_base_struct.sequence_mode = TRUE;												// 扫描模式，转换设定的多个通道
  adc_base_struct.repeat_mode = FALSE;												// 单次转换模式（非连续）
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;						// 数据右对齐
  adc_base_struct.ordinary_channel_length = NOFCHANEL;				// 转换通道个数
  adc_base_config(ADCx, &adc_base_struct);										// 初始化ADC
	
	//触发源
	adc_ordinary_conversion_trigger_set(ADCx, ADC12_ORDINARY_TRIG_TMR1CH3, TRUE); /*!< timer1 ch3 event as trigger source of adc1/adc2 ordinary sequence */
	 
	//adc通道的转换顺序和采样时间
  adc_ordinary_channel_set(ADCx, ADC_CHAVin, 1, USR_ADC_SAMPLETIME);
  adc_ordinary_channel_set(ADCx, ADC_CHAIin, 2, USR_ADC_SAMPLETIME);
  adc_ordinary_channel_set(ADCx, ADC_CHAVout, 3, USR_ADC_SAMPLETIME);
	adc_ordinary_channel_set(ADCx, ADC_CHAIout, 4, USR_ADC_SAMPLETIME);
	
	adc_dma_mode_enable(ADCx, TRUE);														// 使能ADC DMA请求
	
  adc_enable(ADCx, TRUE);																			// 开启ADC，并开始转换
	adc_calibration_init(ADCx);																	// 初始化ADC校准寄存器
	while(adc_calibration_init_status_get(ADCx)){};							// 等待校准寄存器初始化完成
  adc_calibration_start(ADCx);																// ADC开始校准
	while(adc_calibration_status_get(ADCx));										// 等待校准完成

}
