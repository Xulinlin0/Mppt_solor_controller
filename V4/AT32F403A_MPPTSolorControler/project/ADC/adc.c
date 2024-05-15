#include "adc.h"

/* ---------------------- global ----------------------- */
/* ADC�������� */
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
	
  dma_reset(ADC_DMA_CHANNEL);																								 // ��λ DMA ������
	
  dma_default_para_init(&dma_init_struct);	
	dma_init_struct.peripheral_base_addr = (uint32_t)&(ADCx->odt); 						 // �����ַΪ��ADC ���ݼĴ�����ַ
	dma_init_struct.memory_base_addr = (uint32_t)adc1_ordinary_valuetab;  		 // �洢����ַ
	dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY; 								 // ����Դ��������
	dma_init_struct.buffer_size = NOFCHANEL;														  		 // ��������С��Ӧ�õ�������Ŀ�ĵصĴ�С
	dma_init_struct.peripheral_inc_enable = FALSE;											 			 // ����Ĵ���ֻ��һ������ַ���õ���
  dma_init_struct.memory_inc_enable = TRUE;																	 // �洢����ַ����
	dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;// �������ݴ�СΪ���֣��������ֽ�
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;				 // �ڴ����ݴ�СҲΪ���֣����������ݴ�С��ͬ 
  dma_init_struct.loop_mode_enable = TRUE;																	 // ѭ������ģʽ
  dma_init_struct.priority = DMA_PRIORITY_LOW;															 // DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ�� DMA ͨ��ʱ�����ȼ����ò�Ӱ��
  dma_init(ADC_DMA_CHANNEL, &dma_init_struct);															 // ��ʼ�� DMA
	
  dma_channel_enable(ADC_DMA_CHANNEL, TRUE);																 // ʹ�� DMA ͨ��
	
	/************ adc mode config ***************/
	adc_base_config_type adc_base_struct;
  crm_periph_clock_enable(ADC_CLK, TRUE);
	
  crm_adc_clock_div_set(CRM_ADC_DIV_6);												//  120/6=20M < 28M
	
	adc_combine_mode_select(ADC_INDEPENDENT_MODE);							// ����ģʽ
  adc_base_default_para_init(&adc_base_struct); 
	
  adc_base_struct.sequence_mode = TRUE;												// ɨ��ģʽ��ת���趨�Ķ��ͨ��
  adc_base_struct.repeat_mode = FALSE;												// ����ת��ģʽ����������
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;						// �����Ҷ���
  adc_base_struct.ordinary_channel_length = NOFCHANEL;				// ת��ͨ������
  adc_base_config(ADCx, &adc_base_struct);										// ��ʼ��ADC
	
	//����Դ
	adc_ordinary_conversion_trigger_set(ADCx, ADC12_ORDINARY_TRIG_TMR1CH3, TRUE); /*!< timer1 ch3 event as trigger source of adc1/adc2 ordinary sequence */
	 
	//adcͨ����ת��˳��Ͳ���ʱ��
  adc_ordinary_channel_set(ADCx, ADC_CHAVin, 1, USR_ADC_SAMPLETIME);
  adc_ordinary_channel_set(ADCx, ADC_CHAIin, 2, USR_ADC_SAMPLETIME);
  adc_ordinary_channel_set(ADCx, ADC_CHAVout, 3, USR_ADC_SAMPLETIME);
	adc_ordinary_channel_set(ADCx, ADC_CHAIout, 4, USR_ADC_SAMPLETIME);
	
	adc_dma_mode_enable(ADCx, TRUE);														// ʹ��ADC DMA����
	
  adc_enable(ADCx, TRUE);																			// ����ADC������ʼת��
	adc_calibration_init(ADCx);																	// ��ʼ��ADCУ׼�Ĵ���
	while(adc_calibration_init_status_get(ADCx)){};							// �ȴ�У׼�Ĵ�����ʼ�����
  adc_calibration_start(ADCx);																// ADC��ʼУ׼
	while(adc_calibration_status_get(ADCx));										// �ȴ�У׼���

}
