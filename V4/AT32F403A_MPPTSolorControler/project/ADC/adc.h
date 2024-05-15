 #ifndef __ADC_H__
#define __ADC_H__

#include "at32f403a_407_board.h"

/************ 	define 	***************/
// ADC ����궨��
#define ADCx 								ADC1
#define ADC_CLK 						CRM_ADC1_PERIPH_CLOCK
// ADC  IO��	
#define ADC_GPIO_bOUT_CLK 	CRM_GPIOA_PERIPH_CLOCK
#define ADC_PORT_bOUT 			GPIOA

#define ADC_GPIO_bIN_CLK		CRM_GPIOB_PERIPH_CLOCK
#define ADC_PORT_bIN 				GPIOB
	
 // ת��ͨ������	
#define NOFCHANEL 					4

#define ADC_bVin 						GPIO_PINS_1
#define ADC_CHAVin 					ADC_CHANNEL_9

#define ADC_bIin 						GPIO_PINS_0
#define ADC_CHAIin 					ADC_CHANNEL_8
	
#define ADC_bVout 					GPIO_PINS_6
#define ADC_CHAVout 				ADC_CHANNEL_6

#define ADC_bIout 					GPIO_PINS_7
#define ADC_CHAIout 				ADC_CHANNEL_7

//����ʱ���趨
#define USR_ADC_SAMPLETIME	ADC_SAMPLETIME_13_5

// ADC1 ��Ӧ DMA1 ͨ�� 1��ADC3 ��Ӧ DMA2 ͨ�� 5��ADC2 û�� DMA ����
#define ADC_DMA_CHANNEL 		DMA1_CHANNEL1
#define ADC_DMA_CLK					CRM_DMA1_PERIPH_CLOCK


/************ Function declaration ***************/ 
void ADC_INIT(void);

#endif
