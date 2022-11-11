#include "main.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "ADC_CUSTOM.h"
#include "spi.h"
#include "gpio.h"
#include "stdio.h"
#include "cmsis_os.h"


ADC_CONFIG_T ADC_CONFIG={
   .GAIN  = PGA_THIRTY_TWO,
   .SAMPLING_RATE = datarate_FIFTEEN,
   .INPUT_MODE = ADC_CUSTOM_INPUT_MODE,
   .REPORT_INTERVAL_MS = 5,

   .S_INPUT_CHANNEL.ADC_CUSTOM_SINGLE_CH0 = 1,
   .S_INPUT_CHANNEL.ADC_CUSTOM_SINGLE_CH1 = 0,
   .S_INPUT_CHANNEL.ADC_CUSTOM_SINGLE_CH2 = 0,
   .S_INPUT_CHANNEL.ADC_CUSTOM_SINGLE_CH3 = 0,
   .S_INPUT_CHANNEL.ADC_CUSTOM_SINGLE_CH4 = 0,
   .S_INPUT_CHANNEL.ADC_CUSTOM_SINGLE_CH5 = 0,
   .S_INPUT_CHANNEL.ADC_CUSTOM_SINGLE_CH6 = 0,
   .S_INPUT_CHANNEL.ADC_CUSTOM_SINGLE_CH7 = 0,

    .D_INPUT_CHANNEL.ADC_CUSTOM_DIFF_CH0 = 1,
    .D_INPUT_CHANNEL.ADC_CUSTOM_DIFF_CH0 = 0,
    .D_INPUT_CHANNEL.ADC_CUSTOM_DIFF_CH0 = 0,
    .D_INPUT_CHANNEL.ADC_CUSTOM_DIFF_CH0 = 0,
};

ADC_CHANNEL_INFO_T ADC_CHANNEL_INFO;

extern TIM_HandleTypeDef htim12;


void ADC_DELAY_US(uint16_t usec)
{
    htim12.Instance->CNT = 0;
    while(htim12.Instance->CNT < usec);
}

static int32_t ADC_CONV2UV(int32_t ADC_RESULT)
{
    int gain=0;
    float VOLTAGE_UV =  ((float)ADC_RESULT)*2*adc_vref_voltage/8388607.0;
    if((ADC_CONFIG.GAIN) == 0)
    {
    	gain=1;
    }
    else if((ADC_CONFIG.GAIN) == 1)
    {
    	gain=2;
    }
	else if((ADC_CONFIG.GAIN) == 2)
	{
		gain=4;
	}
	else if((ADC_CONFIG.GAIN) == 3)
	{
		gain=8;
	}
	else if((ADC_CONFIG.GAIN) == 4)
	{
		gain=16;
	}
	else if((ADC_CONFIG.GAIN) == 5)
	{
		gain=32;
	}
	else if((ADC_CONFIG.GAIN) == 6)
	{
		gain=64;
	}
	else
	{
		gain=1;
	}
    VOLTAGE_UV /=gain;
    VOLTAGE_UV *= 1000000;
    return (int32_t)VOLTAGE_UV;
}


static void ADC_WRITE_REG(uint8_t REG_ADDR, uint8_t WDATA)
{

    uint8_t WRBUF[3];

    WRBUF[0] = CMD_WREGISTER | REG_ADDR;
    WRBUF[1] = 0;
    WRBUF[2] = WDATA;

    while(  HAL_SPI_Transmit(&hspi2,WRBUF,sizeof(WRBUF),spi4_handle_timeout) != HAL_OK );

}

static void ADC_WRITE_REGS(uint8_t REG_ADDR, uint8_t *MSG, uint8_t LEN)
{

	uint8_t WRBUF[2];

    WRBUF[0] = CMD_WREGISTER | REG_ADDR;
    WRBUF[1] = LEN-1;

    while( HAL_SPI_Transmit(&hspi2,WRBUF,sizeof(WRBUF),spi4_handle_timeout) != HAL_OK );
    while( HAL_SPI_Transmit(&hspi2,MSG,LEN,spi4_handle_timeout) != HAL_OK );
}

static void ADC_READ_REGS(uint8_t REG_ADDR, uint8_t *MSG, uint8_t LEN)
{
    uint8_t WRBUF[2];
    WRBUF[0] = CMD_RREGISTER | REG_ADDR;
    WRBUF[1] = LEN-1;

    while(HAL_SPI_Transmit(&hspi2,WRBUF,sizeof(WRBUF),spi4_handle_timeout) != HAL_OK);
    ADC_DELAY_US(10);

    while( HAL_SPI_Receive(&hspi2, MSG, LEN, spi4_handle_timeout) != HAL_OK );

}

static void ADC_WRITE_CMD(uint8_t _CMD)
{

while( HAL_SPI_Transmit(&hspi2,&_CMD,sizeof(_CMD),spi4_handle_timeout) != HAL_OK ) ;
}

static void ADC_SET_SINGLE_CHANNEL(uint8_t _CH)
{
  ADC_WRITE_REG(reg_mux, (_CH << 4) | (1 << 3));
}

static void ADC_SET_DIFF_CHANNEL(uint8_t _CH)
{
    uint8_t CHANNEL = _CH * 2;
    ADC_WRITE_REG(reg_mux,(CHANNEL << 4) | (CHANNEL + 1) );
}

static int32_t ADC_READ_RESULT(void)
{
    int32_t READ = 0;
    uint8_t BUF[3];

    ADC_WRITE_CMD(cmd_rdata);
    ADC_DELAY_US(10);
    while( HAL_SPI_Receive(&hspi2, BUF, sizeof(BUF), spi4_handle_timeout) != HAL_OK );
    READ = ((uint32_t)BUF[0] << 16) & 0x00FF0000;
    READ |= ((uint32_t)BUF[1] << 8) & 0x0000FF00;; /* Pay attention to It is wrong   read |= (buf[1] << 8) */
    READ |= BUF[2];

	if((READ >> 23) == 1)
	{
		READ = READ - 16777216;
	}
    printf("raw_data:%ld\t",READ);

    return (int32_t)READ;

}

static uint8_t ADC_SELECT_NEXT_CHANNEL(uint8_t CHANNEL)
{
	uint8_t selected=0,i;
	i = (ADC_CHANNEL_INFO.CHANNEL_NUM >= ADC_CHANNEL_NUM - 1)?0:ADC_CHANNEL_INFO.CHANNEL_NUM+1;
        for( ;i<ADC_CHANNEL_NUM;i++)
	{
          if( CHANNEL & (1<<i) )
          {
            ADC_CHANNEL_INFO.CHANNEL_NUM = i;
            selected = 1;
            break;
          }
        }

        if( selected == 0x00 )
        {
            for(i=0;i<ADC_CHANNEL_INFO.CHANNEL_NUM;i++)
            {
                if( CHANNEL & (1<<i) )
                {
                ADC_CHANNEL_INFO.CHANNEL_NUM = i;
                selected = 1;
                break;
                }
            }
        }
    return selected;
}

void ADC_DRDY_ISR(void)
{
    uint8_t SELECTED;
    uint8_t ADC_RESULT_IDX = ADC_CHANNEL_INFO.CHANNEL_NUM;

    if ( ADC_CONFIG.INPUT_MODE == adc_single_input)
    {
        SELECTED = ADC_SELECT_NEXT_CHANNEL( *(uint8_t*)&ADC_CONFIG.S_INPUT_CHANNEL );

	if(SELECTED)
       {
            ADC_SET_SINGLE_CHANNEL(ADC_CHANNEL_INFO.CHANNEL_NUM);
            ADC_WRITE_CMD(cmd_sync);
            ADC_WRITE_CMD(cmd_wakeup);
        }

       ADC_CHANNEL_INFO.ADC_RESULT[ADC_RESULT_IDX] = ADC_READ_RESULT();
       ADC_CHANNEL_INFO.VOLTAGE_UV[ADC_RESULT_IDX] = ADC_CONV2UV( ADC_CHANNEL_INFO.ADC_RESULT[ADC_RESULT_IDX]);
       int DATA1 = (ADC_CHANNEL_INFO.VOLTAGE_UV[0]);

       float VALUE1 = (float)DATA1 /1000000000;
       VALUE1 = VALUE1*1000000;
       printf("on_board_adc :%lf mvolts\t",VALUE1 );
       float PSI, RES = 0.00005;
       int PSI_INT = (0.0629*VALUE1 - 0.1306)/RES;
       PSI = PSI_INT*RES;
       printf("psi:%lf\r\n",PSI);
     }
   else
     {
	 SELECTED = ADC_SELECT_NEXT_CHANNEL( *(uint8_t*)&ADC_CONFIG.D_INPUT_CHANNEL);

	if(SELECTED)
        {
             ADC_SET_DIFF_CHANNEL(ADC_CHANNEL_INFO.CHANNEL_NUM);
             ADC_WRITE_CMD(cmd_sync);
             ADC_WRITE_CMD(cmd_wakeup);
        }
	ADC_CHANNEL_INFO.ADC_RESULT[ADC_RESULT_IDX] = ADC_READ_RESULT();
        ADC_CHANNEL_INFO.VOLTAGE_UV[ADC_RESULT_IDX] = ADC_CONV2UV( ADC_CHANNEL_INFO.ADC_RESULT[ADC_RESULT_IDX]);
        int DATA1 = (ADC_CHANNEL_INFO.VOLTAGE_UV[0]);
        float VALUE1 = (float)DATA1/1000;
        printf("_d_on_adc :%.4lf mvolts\r\n",VALUE1);
     }
}

void ADC_CHANNEL_INIT(void)
{
	uint8_t CHANNEL,i;
	ADC_CHANNEL_INFO.CHANNEL_NUM = 0;
    if ( ADC_CONFIG.INPUT_MODE == adc_single_input )
    {
       CHANNEL = *(uint8_t*)&ADC_CONFIG.S_INPUT_CHANNEL;
        for(i=0;i<ADC_CHANNEL_NUM;i++)
         {
            if( CHANNEL &(1<<i) )
             {
                ADC_CHANNEL_INFO.CHANNEL_NUM = i;
                break;
            }
         }
    ADC_SET_SINGLE_CHANNEL(ADC_CHANNEL_INFO.CHANNEL_NUM);
    }
    else
    {
        CHANNEL = *(uint8_t*)&ADC_CONFIG.D_INPUT_CHANNEL;
        for(i=0;i<ADC_CHANNEL_NUM;i++){
            if( CHANNEL &(1<<i) ){
             ADC_CHANNEL_INFO.CHANNEL_NUM = i;
                break;
            }
        }
        ADC_SET_DIFF_CHANNEL(ADC_CHANNEL_INFO.CHANNEL_NUM);
    }
}

uint8_t BUF[4];

uint8_t ADC_INIT(void)
{
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    do{
        HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
        HAL_Delay(1);

        while(HAL_GPIO_ReadPin(DRDY_GPIO_Port,DRDY_Pin));

        printf(" mr nayak\r\n");

        ADC_READ_REGS(0,BUF,sizeof(BUF));

        if(BUF[1]==0x01 && BUF[2]==0x20 && BUF[3]==0xF0 )
            break;
        HAL_Delay(100);
    }
    while(HAL_GPIO_ReadPin(DRDY_GPIO_Port,DRDY_Pin));
    BUF[reg_status]=0xf4;
    BUF[reg_adcon]=clkout_off+detect_off+ADC_CONFIG.GAIN;
    BUF[reg_drate]=ADC_CONFIG.SAMPLING_RATE;
    ADC_WRITE_REGS(reg_status,BUF,sizeof(BUF));
    while(!HAL_GPIO_ReadPin(DRDY_GPIO_Port,DRDY_Pin));
    while(HAL_GPIO_ReadPin(DRDY_GPIO_Port,DRDY_Pin));
    ADC_READ_REGS(0,BUF,sizeof(BUF));
    while(HAL_GPIO_ReadPin(DRDY_GPIO_Port,DRDY_Pin));
    ADC_CHANNEL_INIT();
    ADC_WRITE_CMD(cmd_sync);
    ADC_WRITE_CMD(cmd_wakeup);
    while(HAL_GPIO_ReadPin(DRDY_GPIO_Port,DRDY_Pin));
    ADC_WRITE_CMD(cmd_selfcal);
    while(HAL_GPIO_ReadPin(DRDY_GPIO_Port,DRDY_Pin));
    while(HAL_GPIO_ReadPin(DRDY_GPIO_Port,DRDY_Pin));
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    return 1;
}

