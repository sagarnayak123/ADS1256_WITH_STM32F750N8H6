/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_tim.h"
#include "spi.h"
#include "gpio.h"
#include "ADS1256.h"
#include "stdio.h"
//#include "core_cm3.h"
float sum=0;

#define SPI2_HANDLE_TIMEOUT  10

/* Private variables ---------------------------------------------------------*/

//#define ADS125X_VREF_VOLTAGE      1.486//uint:V
#define ADS125X_VREF_VOLTAGE      	2.5//uint:V 0.05231=>104.62/2*1000

ads125x_conf_t ads125x_conf={
    .gain = PGA_1,
    .sampling_rate = DATARATE_10,
    .input_mode = ADC1256_INPUT_MODE,
    .report_interval_ms = 5, //uint:ms

    /*single-ended input channel:1=enable, 0=disable*/
    .single_input_channel.ADS1256_SINGLE_CH0 = 1,
    .single_input_channel.ADS1256_SINGLE_CH1 = 0,
    .single_input_channel.ADS1256_SINGLE_CH2 = 0,
    .single_input_channel.ADS1256_SINGLE_CH3 = 0,
    .single_input_channel.ADS1256_SINGLE_CH4 = 0,
    .single_input_channel.ADS1256_SINGLE_CH5 = 0,
    .single_input_channel.ADS1256_SINGLE_CH6 = 0,
    .single_input_channel.ADS1256_SINGLE_CH7 = 0,

    /*differential input channel:1=enable, 0=disable*/
    .diff_input_channel.ADS1256_DIFF_CH0 = 1, /*AINp=AIN0, AINn=AIN1*/
    .diff_input_channel.ADS1256_DIFF_CH1 = 0, /*AINp=AIN2, AINn=AIN3*/
    .diff_input_channel.ADS1256_DIFF_CH2 = 0, /*AINp=AIN4, AINn=AIN5*/
    .diff_input_channel.ADS1256_DIFF_CH3 = 0, /*AINp=AIN6, AINn=AIN7*/
};

ads125x_channel_info_t ads125x_channel_info;

extern TIM_HandleTypeDef htim9;



/* Private function prototypes -----------------------------------------------*/

/*
*********************************************************************************************************
*	name:
*	function:
*	parameter:
*	The return value: NULL
*********************************************************************************************************
*/
//#pragma optimize=none
void ads1256_delay_us(uint32_t usec)
{
    //#define OSC     (72)                                 //����Ϊ72M
    //#define OSC_D   (OSC/7)
    //    uint32_t i;
    //    for(i=0; i<OSC_D*usec; i++){
    //        ;
    //        }

//	htim4.Instance->CNT = 0;
//	while (htim4.Instance->CNT < usec);

    uint16_t cccnt,pcnt,dcnt;
    pcnt=htim9.Instance->CNT;

    do{
        cccnt = htim9.Instance->CNT;
        dcnt = (cccnt >= pcnt)?(cccnt - pcnt):(0xFFFF - pcnt + cccnt);

    }while(dcnt<usec);


}

/*
*********************************************************************************************************
*	name: Voltage_Convert
*	function:  Voltage value conversion function
*	parameter: Vref : The reference voltage 3.3V or 5V
*			   voltage : output DAC value
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ads1256_conv2uv(int32_t adc_result)
{
    /* Vin = ( (2*Vr) / G ) * ( x / (2^23 -1)) */
	int gain=0;
    float voltage_uv =  ((float)adc_result)*2*ADS125X_VREF_VOLTAGE/8388607.0;
//    printf("gain:%d\t",(ads125x_conf.gain));
    if((ads125x_conf.gain) == 0)
    {
    	gain=1;
    }
    else if((ads125x_conf.gain) == 1)
    {
    	gain=2;
    }
	else if((ads125x_conf.gain) == 2)
	{
		gain=4;
	}
	else if((ads125x_conf.gain) == 3)
	{
		gain=8;
	}
	else if((ads125x_conf.gain) == 4)
	{
		gain=16;
	}
	else if((ads125x_conf.gain) == 5)
	{
		gain=32;
	}
	else if((ads125x_conf.gain) == 6)
	{
		gain=64;
	}
	else
	{
		gain=1;
	}
    voltage_uv /=gain;
    voltage_uv *= 1000000;
    return (int32_t)voltage_uv;



}
/*
*********************************************************************************************************
*	name: ads1256_write_reg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ads1256_write_reg(uint8_t reg_addr, uint8_t wdata)
{
    uint8_t wrbuf[3];

    wrbuf[0] = CMD_WREG | reg_addr;	/*Write command register */
    wrbuf[1] = 0; /*Write the register number */
    wrbuf[2] = wdata;

    while(  HAL_SPI_Transmit(&hspi4,wrbuf,sizeof(wrbuf),SPI2_HANDLE_TIMEOUT) != HAL_OK ); 	/*send register value */
}

/*
*********************************************************************************************************
*	name: ads1256_write_reg
*	function: Write the corresponding register
*	parameter: _RegID: register  ID
*			 _RegValue: register Value
*	The return value: NULL
*********************************************************************************************************
*/
static void ads1256_write_regs(uint8_t reg_addr, uint8_t *msg, uint8_t len)
{
    uint8_t wrbuf[2];

    wrbuf[0] = CMD_WREG | reg_addr;	/*Write command register */
    wrbuf[1] = len-1; /*Write the register number */

    while( HAL_SPI_Transmit(&hspi4,wrbuf,sizeof(wrbuf),SPI2_HANDLE_TIMEOUT) != HAL_OK );  /*send register value */
    while( HAL_SPI_Transmit(&hspi4,msg,len,SPI2_HANDLE_TIMEOUT) != HAL_OK ); 	/*send register value */
}

/*
*********************************************************************************************************
*	name: ads1256_read_reg
*	function: Read  the corresponding register
*	parameter: _RegID: register  ID
*	The return value: read register value
*********************************************************************************************************
*/
static void ads1256_read_regs(uint8_t reg_addr, uint8_t *msg, uint8_t len)
{

    uint8_t wrbuf[2];
    wrbuf[0] = CMD_RREG | reg_addr;	/*Write command register */
    wrbuf[1] = len-1; /*Write the register number */

    while(HAL_SPI_Transmit(&hspi4,wrbuf,sizeof(wrbuf),SPI2_HANDLE_TIMEOUT) != HAL_OK); /*send register value */

    ads1256_delay_us(10);	/*delay time */


    while( HAL_SPI_Receive(&hspi4, msg, len, SPI2_HANDLE_TIMEOUT) != HAL_OK );  /* Read the register values */

}

/*
*********************************************************************************************************
*	name: ads1256_write_cmd
*	function: Sending a single byte order
*	parameter: _cmd : command
*	The return value: NULL
*********************************************************************************************************
*/
static void ads1256_write_cmd(uint8_t _cmd)
{
    while( HAL_SPI_Transmit(&hspi4,&_cmd,sizeof(_cmd),SPI2_HANDLE_TIMEOUT) != HAL_OK ) ;  /*send comand value */
}

/*
*********************************************************************************************************
*	name: ads1256_set_single_channel
*	function: Configuration channel number
*	parameter:  _ch:  channel number  0--7
*	The return value: NULL
*********************************************************************************************************
*/
static void ads1256_set_single_channel(uint8_t _ch)
{
    ads1256_write_reg(REG_MUX, (_ch << 4) | (1 << 3));	/* Bit3 = 1, AINN connection AINCOM */
}

/*
*********************************************************************************************************
*	name: ads1256_set_diff_channel
*	function: The configuration difference channel
*	parameter:  _ch:  channel number  0--3
*	The return value:  four high status register
*********************************************************************************************************
*/
static void ads1256_set_diff_channel(uint8_t _ch)
{
    uint8_t channel = _ch * 2;
    ads1256_write_reg(REG_MUX,(channel << 4) | (channel + 1) );
}

/*
*********************************************************************************************************
*	name: ads1256_read_result
*	function: read ADC value
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static int32_t ads1256_read_result(void)
{
    int32_t read = 0;
    uint8_t buf[3];

    ads1256_write_cmd(CMD_RDATA);	/* read ADC command  */

    ads1256_delay_us(10);	/*delay time  */

    /*Read the sample results 24bit*/
    while( HAL_SPI_Receive(&hspi4, buf, sizeof(buf), SPI2_HANDLE_TIMEOUT) != HAL_OK );
    read = ((uint32_t)buf[0] << 16) & 0x00FF0000;
    read |= ((uint32_t)buf[1] << 8) & 0x0000FF00;; /* Pay attention to It is wrong   read |= (buf[1] << 8) */
    read |= buf[2];

    /* Extend a signed number*/

//    read|=0x800000;
//      if(read > 0x800000)
//      {
//      	read-=16777215-read;
//      }

	if((read >> 23) == 1)
	{
		read = read - 16777216; //Handling negative value
	}

//    if(read & 0x800000)
//    {
//    	read |= 0xFF000000;
//    }
	printf("raw_data:%ld\t",read );

    return (int32_t)read;

}


/*
*********************************************************************************************************
*	name:
*	function:
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
static uint8_t ads1256_select_next_channel(uint8_t channel)
{
    uint8_t selected=0,i;

    i = (ads125x_channel_info.channel_num >= ADS1256_CHANNEL_NUM - 1)?0:ads125x_channel_info.channel_num+1;
    for( ;i<ADS1256_CHANNEL_NUM;i++){
        if( channel & (1<<i) ){
            ads125x_channel_info.channel_num = i;
            selected = 1;
            break;
        }
    }
    if( selected == 0x00 ){
        for(i=0;i<ads125x_channel_info.channel_num;i++){
            if( channel & (1<<i) ){
                ads125x_channel_info.channel_num = i;
                selected = 1;
                break;
            }
        }
    }
    return selected;
}
float pradeep_sensor_PSI(float mV)
{
	float psi, res = 0.00005;
	int psi_int = (0.0629*mV - 0.1306)/res;  //converting milli volts into psi
	psi = psi_int*res;
	return psi;
}

/*
*********************************************************************************************************
*	name: ads1256_drdy_isr
*	function: Collection procedures
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
void ads1256_drdy_isr(void)
{
    uint8_t selected;
    uint8_t adc_result_idx = ads125x_channel_info.channel_num;

    if ( ads125x_conf.input_mode == ADS1256_SIGNGLE_INPUT){/*  0  Single-ended input  8 channel?? 1 Differential input  4 channel */
        selected = ads1256_select_next_channel( *(uint8_t*)&ads125x_conf.single_input_channel );

        if(selected){
            ads1256_set_single_channel(ads125x_channel_info.channel_num);	/*Switch channel mode */
            ads1256_write_cmd(CMD_SYNC);
            ads1256_write_cmd(CMD_WAKEUP);
        }
//        for(int i=0;i<1000;i++)
//        {
        ads125x_channel_info.adc_result[adc_result_idx] = ads1256_read_result();
        ads125x_channel_info.voltage_uv[adc_result_idx] = ads1256_conv2uv( ads125x_channel_info.adc_result[adc_result_idx]);
        int data1 = (ads125x_channel_info.voltage_uv[0]);
       // int data2 = (ads125x_channel_info.voltage_uv[1]);
        float value1 = (float)data1 /1000000000;
        value1 = value1*1000000;
//        sum+=value1;
//        }
		printf("channel_0 :%lf mvolts\t",value1 );

		float psi, res = 0.00005;
		int psi_int = (0.0629*value1 - 0.1306)/res;  //converting milli volts into psi
		psi = psi_int*res;
		printf("psi:%lf\r\n",psi);
//		sum=0;

//		float bar = value1*0.001971-1.97379;
//		sum+=bar;


    }
    else{
        /*DiffChannal*/

         selected = ads1256_select_next_channel( *(uint8_t*)&ads125x_conf.diff_input_channel );

         if(selected){
             ads1256_set_diff_channel(ads125x_channel_info.channel_num);	/* change DiffChannal */
             ads1256_write_cmd(CMD_SYNC);
             ads1256_write_cmd(CMD_WAKEUP);
        }
        ads125x_channel_info.adc_result[adc_result_idx] = ads1256_read_result();
        ads125x_channel_info.voltage_uv[adc_result_idx] = ads1256_conv2uv( ads125x_channel_info.adc_result[adc_result_idx]);
        int data1 = (ads125x_channel_info.voltage_uv[0]);
        float value1 = (float)data1/1000;
//        float pascal=40/5*value1;
//        float pascal = value1 *0.025367-0.01682;
//        float pascal = value1 *0.12686-0.07797;
//        pascal = value1 *0.12686-0.07797;
        float pascal = value1 *0.12788418686122-0.083301076088708;
        printf("_diff_channel_0 :%.4lf mvolts\t",value1);
        printf("_diff_channel_0_pascal :%.4lf psi\t",pascal);
        printf("bar:%lf\r\n",((0.0689476*pascal)*1000)-38.651736);

//        float psi = pradeep_sensor_PSI(value1);
//        printf("pressure_psi:%lf\r\n",psi);


     //   value2=value1*15.076+0.00854;
		//float max_val=75362000/34473.01;
		//printf("channel_0 :%.6lf mvolts\r\n",value2);


    }
}

/*
*********************************************************************************************************
*	name:
*	function:
*	parameter: NULL
*	The return value:  NULL
*********************************************************************************************************
*/
void ads1256_channel_init(void)
{
    uint8_t channel,i;

    ads125x_channel_info.channel_num = 0;
    if ( ads125x_conf.input_mode == ADS1256_SIGNGLE_INPUT ){/*  0  Single-ended input  8 channel?? 1 Differential input  4 channel */
        channel = *(uint8_t*)&ads125x_conf.single_input_channel;
        for(i=0;i<ADS1256_CHANNEL_NUM;i++){
            if( channel &(1<<i) ){
                ads125x_channel_info.channel_num = i;
                break;
            }
        }
        ads1256_set_single_channel(ads125x_channel_info.channel_num);
    }
    else{
        channel = *(uint8_t*)&ads125x_conf.diff_input_channel;
        for(i=0;i<ADS1256_CHANNEL_NUM;i++){
            if( channel &(1<<i) ){
                ads125x_channel_info.channel_num = i;
                break;
            }
        }
        ads1256_set_diff_channel(ads125x_channel_info.channel_num);
    }
}
/*
*********************************************************************************************************
*	name: ads1256_init(
*	function: The configuration parameters of ADC, gain and data rate
*	parameter: _gain:gain 1-64
*                      _drate:  data  rate
*	The return value: NULL
*********************************************************************************************************
*/
uint8_t buf[4];
uint8_t ads1256_init(void)
{

    HAL_GPIO_WritePin(ADC2_CSN_GPIO_Port, ADC2_CSN_Pin, GPIO_PIN_RESET);
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
    do{
        HAL_GPIO_WritePin(ADC2_RST_GPIO_Port, ADC2_RST_Pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(ADC2_RST_GPIO_Port, ADC2_RST_Pin, GPIO_PIN_SET);
        HAL_Delay(1);

        while(HAL_GPIO_ReadPin(ADC2_DRDY_GPIO_Port,ADC2_DRDY_Pin));


        ads1256_read_regs(0,buf,sizeof(buf));


        if( buf[1]==0x01 && buf[2]==0x20 && buf[3]==0xF0 )
            break;
        HAL_Delay(100);
    }
    while(HAL_GPIO_ReadPin(ADC2_DRDY_GPIO_Port,ADC2_DRDY_Pin));
    buf[REG_STATUS]=0xf4;//STATUS REGISTER:Auto-Calibration Enabled,Analog Input buffer Disabled
    buf[REG_ADCON]=CLKOUT_OFF+DETECT_OFF+ads125x_conf.gain;   //ADCON=00h
    buf[REG_DRATE]=ads125x_conf.sampling_rate;
    ads1256_write_regs(REG_STATUS,buf,sizeof(buf));
    while(!HAL_GPIO_ReadPin(ADC2_DRDY_GPIO_Port,ADC2_DRDY_Pin));
    while(HAL_GPIO_ReadPin(ADC2_DRDY_GPIO_Port,ADC2_DRDY_Pin));
    ads1256_read_regs(0,buf,sizeof(buf));
    while(HAL_GPIO_ReadPin(ADC2_DRDY_GPIO_Port,ADC2_DRDY_Pin));
    ads1256_channel_init();
    ads1256_write_cmd(CMD_SYNC);
    ads1256_write_cmd(CMD_WAKEUP);
    while(HAL_GPIO_ReadPin(ADC2_DRDY_GPIO_Port,ADC2_DRDY_Pin));
    ads1256_write_cmd(CMD_SELFCAL); //self-calibration
    while(!HAL_GPIO_ReadPin(ADC2_DRDY_GPIO_Port,ADC2_DRDY_Pin));
    while(HAL_GPIO_ReadPin(ADC2_DRDY_GPIO_Port,ADC2_DRDY_Pin));
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    printf("done\r\n");
   return 1;
}


