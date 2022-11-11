/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H


/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"
#include "stdlib.h"
#include "stdbool.h"



#define adc_single_input         0
#define adc_differential_input    1


//#define ADC_CUSTOM_INPUT_MODE   adc_single_input        //Single-ended input
#define ADC_CUSTOM_INPUT_MODE   adc_differential_input  //ADifferential input

#if (ADC_CUSTOM_INPUT_MODE == adc_single_input)
#define ADC_CHANNEL_NUM    8
#else
#define ADC_CHANNEL_NUM    4
#endif

#define spi4_handle_timeout  10
#define adc_vref_voltage      	2.5//uint:V 0.05231=>104.62/2*1000

//*************************** o��?����?***********************************************/
/*registers' Address*/
#define reg_status   0
#define reg_mux      1
#define reg_adcon    2
#define reg_drate    3 
#define reg_io       4 
#define reg_ofc0     5 
#define reg_ofc1     6
#define reg_opc2     7 
#define reg_fsc0     8 
#define reg_fsc1     9
#define reg_fsc2     10

/*Operation Command*/
//#define cmd_WAKEUP     0x00
#define cmd_rdata       0x01
#define cmd_rdatac      0x03 
#define cmd_sdatac      0x0F
#define CMD_RREGISTER   0x10
#define CMD_WREGISTER   0x50
#define cmd_selfcal     0xf0
#define cmd_selfocal    0xf1
#define cmd_selfgcal    0xf2
#define cmd_sysocal     0xf3
#define cmd_sysgcal     0xf4
#define cmd_sync        0xfc
#define cmd_standby     0xfd
#define cmd_reset       0xfe
#define cmd_wakeup      0xFF

#define PGA_ONE             0x00  //��5V
#define PGA_TWO             0x01  //��2.5V
#define PGA_FOUR            0x02  //��1.25V
#define PGA_EIGHT           0x03  //��0.625V
#define PGA_SIXTEEN         0x04  //��312.5mV
#define PGA_THIRTY_TWO      0x05  //��156.25mV
#define PGA_SIXTY_FOUR      0x06  //��78.125mV

#define positive_ain0             (0X00<<4)
#define positive_ain1             (0X01<<4)
#define positive_ain2             (0X02<<4)
#define positive_ain3             (0X03<<4)
#define positive_ain4             (0X04<<4)
#define positive_ain5             (0X05<<4)
#define positive_ain6             (0X06<<4)
#define positive_ain7             (0X07<<4)
#define positive_aincom           (0X08<<4)

#define negative_ain0               0X00
#define negative_ain1               0X01
#define negative_ain2               0X02
#define negative_ain3               0X03
#define negative_ain4               0X04
#define negative_ain5               0X05
#define negative_ain6               0X06
#define negative_ain7               0X07
#define negative_aincom             0X08


#define datarate_THIRTYK               0xf0
#define datarate_FIFTEENK              0xe0
#define datarate_SEVEN_FIVEK           0xd0
#define datarate_THREE_SEVEN_FIVEK     0xc0
#define datarate_TWO_K                 0xb0
#define datarate_ONE_K                 0xa1
#define datarate_FIVE_HUNDRED          0x92
#define datarate_HUNDRED               0x82
#define datarate_SIXTY                 0x72
#define datarate_FIFTY                 0x63
#define datarate_THIRTY                0x53
#define datarate_TWENTY_FIVE           0x43
#define datarate_FIFTEEN               0x33
#define datarate_TEN                   0x23
#define datarate_FIVE                  0x13
#define datarate_TWO                   0x03

/*STATUS regISTER*/
#define msb_frist                 (0x00<<3)
#define lsb_frist                 (0x01<<3)
#define acal_off                  (0x00<<2)
#define acal_on                   (0x01<<2)
#define bufen_off                 (0x00<<1)
#define bufen_on                  (0x01<<1)

/*adcon regISTER*/
#define clkout_off                (0x00<<5)
#define clkout_clkin              (0x01<<5)
#define detect_off                (0x00<<3)
#define detect_on_2ua             (0x02<<3)

//#pragma pack(1)
//
typedef struct
{
    uint8_t GAIN;		/* GAIN  */
    uint8_t SAMPLING_RATE;	/* DATA output  speed*/
    uint8_t INPUT_MODE;          /*single-ended input channel:1=enable, 0=disable*/
    uint16_t REPORT_INTERVAL_MS;

    struct{
        uint8_t ADC_CUSTOM_SINGLE_CH0:1;
        uint8_t ADC_CUSTOM_SINGLE_CH1:1;
        uint8_t ADC_CUSTOM_SINGLE_CH2:1;
        uint8_t ADC_CUSTOM_SINGLE_CH3:1;
        uint8_t ADC_CUSTOM_SINGLE_CH4:1;
        uint8_t ADC_CUSTOM_SINGLE_CH5:1;
        uint8_t ADC_CUSTOM_SINGLE_CH6:1;
        uint8_t ADC_CUSTOM_SINGLE_CH7:1;
    }S_INPUT_CHANNEL;

    struct{
        uint8_t ADC_CUSTOM_DIFF_CH0:1;
        uint8_t ADC_CUSTOM_DIFF_CH1:1;
        uint8_t ADC_CUSTOM_DIFF_CH2:1;
        uint8_t ADC_CUSTOM_DIFF_CH3:1;
    }D_INPUT_CHANNEL;

}ADC_CONFIG_T;

typedef struct{

    uint8_t CHANNEL_NUM;
    int32_t ADC_SUM[ADC_CHANNEL_NUM];
    uint32_t ADC_COUT[ADC_CHANNEL_NUM];
    int32_t ADC_RESULT[ADC_CHANNEL_NUM];	 /* adc  Conversion value */
    int32_t VOLTAGE_UV[ADC_CHANNEL_NUM];	 /* channel voltage*/
}ADC_CHANNEL_INFO_T;



uint8_t  ADC_INIT(void);
void ADC_DRDY_ISR(void);

#endif

