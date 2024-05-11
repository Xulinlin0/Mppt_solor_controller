#include "esp8266.h"
#include "string.h"
#include "CtlLoop.h"
#include "charge.h"
#include "printf-stdarg.h"

/* ================ extern ================ */
extern pid_struct pid_ctrol[4];	
extern uint8_t	chargerMode;
extern uint8_t	mppt_enable;
extern BattryPara_TypeDef  gMy_Battry;

/* ================ global ================ */
/* esp8266 config aliyun param */
/*
const char* WIFI = "Xiaomi_8A6F";
const char* WIFIASSWORD = "13929385084?ljl";
*/

const char* WIFI = "9B305";
const char* WIFIASSWORD = "9b305305";

const char* ClintID = "k17tivpPKoi.mpptSolorDevice|securemode=2\\,signmethod=hmacsha256\\,timestamp=1715418061203|";
const char* username = "mpptSolorDevice&k17tivpPKoi";
const char* passwd = "6251d922d6e23d2fa050f446b6e061b2c09493b757736990077b56201f08bbaf";

const char* Url = "iot-06z00hz4jtn6ul5.mqtt.iothub.aliyuncs.com";

//const char* pubtopic = "/sys/k115qjMhgNj/mpptSolorDevice/thing/event/property/post";
const char* subtopic = "/sys/k17tivpPKoi/mpptSolorDevice/thing/event/property/post_reply";

// rec then find parms
const char* mqtt_rx_parms_pos = "params\":{\"";	//这里最开始写错了, 白白找了好久的问题！

char* g_identifier = 0;
char* g_iden_parm = 0;

// pub format
const char* mqtt_tx_pub_pre = "AT+MQTTPUB=0,\"/sys/k17tivpPKoi/mpptSolorDevice/thing/event/property/post\",\"{\\\"params\\\":";

const char* mqtt_tx_pub_end = "\\,\\\"version\\\":\\\"1.0.0\\\"}\",0,0\r\n";

/* usart1 datebuf param */
static u8 uart_tx_buf[256] = {0};
static char uart_tx_temp[256] = {0};
char uart_tx_parm[150] = {0};

static uint8_t usart_rx_counter = 0;
char uart_rx_buf[256]	= {0};

//物联网接收开关信号
int8_t Netbutton = -1;	//物联网接收信号-接收到开时刻为1，关时刻为0，其他时刻为-1

/* ================ function ================ */
void usart1_init(void)
{
	/*-------------- clock config -------------------*/
	/* enable dma1 periph clock */
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	
	/* enable gpioa periph clock */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	
	/* enable usart1 periph clock */
  crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
	
	/*-------------- gpio config -------------------*/
	gpio_init_type gpio_init_struct;
  gpio_default_para_init(&gpio_init_struct);
	
  /* configure the TX pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_9;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

  /* configure the RX pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_10;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOA, &gpio_init_struct);

	/*-------------- usart config -------------------*/
  /* configure usart param */
  usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  usart_transmitter_enable(USART1, TRUE);
  usart_receiver_enable(USART1, TRUE);
  usart_parity_selection_config(USART1, USART_PARITY_NONE);

  usart_dma_transmitter_enable(USART1, TRUE);
	
  usart_hardware_flow_control_set(USART1, USART_HARDWARE_FLOW_NONE);

  usart_enable(USART1, TRUE);
	
	/*-------------- nvic config -------------------*/
	nvic_irq_enable(USART1_IRQn, 1, 0);
	usart_interrupt_enable(USART1, USART_RDBF_INT, TRUE);//记得使能接收中断!

	/*-------------- dma config -------------------*/
  /* USART1 TX DMA1 Channel (triggered by USART1 Tx event) Config */
	dma_init_type dma_init_struct;
	
  dma_reset(DMA1_CHANNEL4);  
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.peripheral_base_addr = (u32)&USART1->dt;
  dma_init_struct.memory_base_addr = (uint32_t)uart_tx_buf;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.buffer_size = 0;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
  dma_init(DMA1_CHANNEL4, &dma_init_struct);
}

void USART1_IRQHandler(void)
{
	if (usart_flag_get(USART1,USART_RDBF_FLAG) != RESET)
	{
		uart_rx_buf[usart_rx_counter++] = usart_data_receive(USART1);
		if ((uart_rx_buf[usart_rx_counter - 2] == '\r') && (uart_rx_buf[ usart_rx_counter - 1] == '\n'))  
		{
			uart_rx_buf[usart_rx_counter - 2 ] = '\0';
			usart_rx_counter = 0;
			esp8266_sub_getparms();
		}
	}
}

uint8_t flTstr_FirpTwo(float data, char* datTostr)
{
	if (data >= 100 || data <= 0)	return 0;
	
	uint32_t dateToInt = data * 100;
	
	for (int8_t i = 4; i >= 0; i--)
	{
		if (i == 2)
		{
			datTostr[i] ='.';
		}
		else
		{
			datTostr[i] = dateToInt % 10 + '0';
			dateToInt /= 10;
		}
	}
	datTostr[5] = 0;
	return 1;
}

float strTof_FirpTwo(char* strTodat)
{
	return ((strTodat[0] - '0')*10 + (strTodat[1] - '0') + (strTodat[3] - '0')*0.1 + (strTodat[4] - '0')*0.01);
}
	
uint8_t esp8266_Init(void)
{
	usart1_init();

	Esp8266_send_data((u8*)"AT+RST\r\n", 9);
	dwt_delay_sec(2);
	
	memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
	Esp8266_send_data((u8*)"ATE0\r\n", 7);    //关闭回显
	dwt_delay_ms(10);
	if(strcmp(uart_rx_buf, "OK"))
	{
		return 1;
	}
	
	memset(uart_rx_buf,0,sizeof(uart_rx_buf));
	Esp8266_send_data((u8*)"AT+CWMODE=1\r\n", 14);
	dwt_delay_ms(100);
	if(strcmp(uart_rx_buf,"OK"))
	{
		return 2;
	}
	
	memset(uart_rx_buf,0,sizeof(uart_rx_buf));
	tiny_sprintf(uart_tx_temp, "AT+CWJAP=\"%s\",\"%s\"\r\n", WIFI, WIFIASSWORD);//连接热点
	Esp8266_send_data((u8*)uart_tx_temp, strlen(uart_tx_temp));
	dwt_delay_sec(2);
	if(strcmp(uart_rx_buf,"OK"))
	{
		return 3;
	}
	
	memset(uart_rx_buf,0,sizeof(uart_rx_buf));
	tiny_sprintf(uart_tx_temp, "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n", ClintID, username, passwd);//用户信息配置
	Esp8266_send_data((u8*)uart_tx_temp, strlen(uart_tx_temp));
	dwt_delay_ms(100);
	if(strcmp(uart_rx_buf,"OK"))
	{
	return 4;
	}
	
	memset(uart_rx_buf,0,sizeof(uart_rx_buf));
	tiny_sprintf(uart_tx_temp, "AT+MQTTCONN=0,\"%s\",1883,1\r\n",Url); //连接服务器
	Esp8266_send_data((u8*)uart_tx_temp, strlen(uart_tx_temp));
	dwt_delay_ms(1000);
	if(strcmp(uart_rx_buf,"OK"))
	{
		return 5;
	}
	
	memset(uart_rx_buf,0,sizeof(uart_rx_buf));
	tiny_sprintf(uart_tx_temp, "AT+MQTTSUB=0,\"%s\",1\r\n",subtopic); //订阅消息
	Esp8266_send_data((u8*)uart_tx_temp, strlen(uart_tx_temp));
	dwt_delay_ms(500);
	if(strcmp(uart_rx_buf,"OK"))
	{
		return 6;
	}
	
	memset(uart_rx_buf,0,sizeof(uart_rx_buf));
	return 0;
}
//why不可以直接while(0==strcmp(uart_rx_buf,"OK");

void esp8266_sub_getparms(void)
{
	if (strncmp(uart_rx_buf,"+MQTTSUBRECV:", 13) == 0)
	{	
		//find identifier's begin
		char *ptr = strstr(uart_rx_buf, mqtt_rx_parms_pos);
		if (ptr == NULL) return;
		char *ptr_ref = ptr + 10;
		
		//get identifier&param value
		g_identifier = strtok(ptr_ref, ":");
		g_identifier[strlen(g_identifier) - 1] = '\0';
		g_iden_parm = strtok(NULL, "}");
		
		//set effective value to right varible
		if (0 == strcmp(g_identifier, "DeviceSwitch"))
		{
			Netbutton = g_iden_parm[0] - '0';
		}
		else if (0 == strcmp(g_identifier, "mppt_enable"))
		{
			mppt_enable = g_iden_parm[0] - '0';
		}
		else if (0 == strcmp(g_identifier, "chargerMode"))
		{
			chargerMode = g_iden_parm[0] - '0';
		}
		else if (0 == strcmp(g_identifier, "Vout_value"))
		{
			pid_ctrol[2].target = strTof_FirpTwo(g_iden_parm);
		}
		else if (0 == strcmp(g_identifier, "Iout_value"))
		{
			pid_ctrol[3].target = strTof_FirpTwo(g_iden_parm);
		}
		else if (0 == strcmp(g_identifier, "floating_Io"))
		{
			gMy_Battry.floating_Io = strTof_FirpTwo(g_iden_parm);
		}
		else if (0 == strcmp(g_identifier, "charging_Io"))
		{
			gMy_Battry.charging_Io = strTof_FirpTwo(g_iden_parm);
		}
		else if (0 == strcmp(g_identifier, "precharg_Io"))
		{
			gMy_Battry.precharg_Io = strTof_FirpTwo(g_iden_parm);
		}
		else if (0 == strcmp(g_identifier, "cutoff_Vo"))
		{
			gMy_Battry.cutoff_Vo = strTof_FirpTwo(g_iden_parm);
		}
		else if (0 == strcmp(g_identifier, "floating_Vo"))
		{
			gMy_Battry.floating_Vo = strTof_FirpTwo(g_iden_parm);
		}
		else if (0 == strcmp(g_identifier, "charging_Vo"))
		{
			gMy_Battry.charging_Vo = strTof_FirpTwo(g_iden_parm);
		}
		tiny_sprintf(uart_tx_parm, "{\\\"%s\\\":%s}", g_identifier, g_iden_parm);
		esp8266_pub_data(uart_tx_parm);
	}
}

void Esp8266_send_data(u8 *buf, u16 len)
{
  memcpy(uart_tx_buf, buf, len);
  while(usart_flag_get(USART1, USART_TDBE_FLAG) == RESET);
  dma_channel_enable(DMA1_CHANNEL4, FALSE);
  dma_data_number_set(DMA1_CHANNEL4, len);
  dma_channel_enable(DMA1_CHANNEL4, TRUE);
}

//pubparm get by example —— sprintf(uart_tx_parm, "{\\\"Vout_value\\\":%6.3f\\,\\\"Vin_value\\\":%6.3f}", 12.8, 14.2);
void esp8266_pub_data(char* pubparm)
{ 
	tiny_sprintf(uart_tx_temp, "%s%s%s", mqtt_tx_pub_pre, pubparm, mqtt_tx_pub_end);
	Esp8266_send_data((u8*)uart_tx_temp, strlen(uart_tx_temp));
}

