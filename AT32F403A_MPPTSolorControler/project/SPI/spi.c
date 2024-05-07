#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

void SPI1_Init(void)
{
  gpio_init_type gpio_init_struct;
  spi_init_type spi_init_struct;

  gpio_default_para_init(&gpio_init_struct);
  spi_default_para_init(&spi_init_struct);

	/* enable iomux periph clock */
	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
	
	/* enable gpiob periph clock */
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

  /* enable spi1 periph clock */
  crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
	
	gpio_pin_remap_config(SWJTAG_GMUX_010, TRUE);
	gpio_pin_remap_config(SPI1_GMUX_0001, TRUE);
	
  /* configure the SCK pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_3;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the MISO pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init_struct.gpio_pins = GPIO_PINS_4;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure the MOSI pin */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init_struct.gpio_pins = GPIO_PINS_5;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init(GPIOB, &gpio_init_struct);

  /* configure param */
  spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
  spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
  spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
  spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
  spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
  spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
  spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
  spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
  spi_init(SPI1, &spi_init_struct);
	spi_crc_polynomial_set(SPI1, 0x07);
	
  spi_enable(SPI1, TRUE);
}

u8 SPI1_WriteByte(u8 Byte)
{
	while((SPI1->sts&SPI_I2S_TDBE_FLAG)==RESET);		//等待发送区空	  
	SPI1->dt=Byte;	 	//发送一个byte   
	while((SPI1->sts&SPI_I2S_RDBF_FLAG)==RESET);//等待接收完一个byte  
	return SPI1->dt;          	     //返回收到的数据		
} 
