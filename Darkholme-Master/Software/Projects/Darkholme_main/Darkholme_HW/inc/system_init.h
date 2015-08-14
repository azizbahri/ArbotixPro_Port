/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : system_init.h
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : This file contains the defines used for Sys init fuctions
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYSTEM_INIT_H
#define __SYSTEM_INIT_H





/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define USART_BUFFER_SIZE	0x3FF

///////////////////ADC Channels//////////////////////////////////////////////////////////
#define ADC1_Channel_X  ADC_Channel_10
#define ADC2_Channel_X  ADC_Channel_4
//when changing peripherals, make sure to change the corresponding AFs
///////////////////UART\//////////////////////////////////////////////////////////
#define DXL_USART       USART1
#define PC_USART        USART3
#define ZIGBEE_USART    UART5
#define CPU_USART       UART4  //unused

///////////////////SPI\//////////////////////////////////////////////////////////
#define IMU_SPI         SPI2

///////////////////AF\//////////////////////////////////////////////////////////
#define DXL_USART_AF       GPIO_AF_USART1
#define PC_USART_AF        GPIO_AF_USART3
#define ZIGBEE_USART_AF    GPIO_AF_UART5
#define CPU_USART_AF       GPIO_AF_UART4  //unused
#define SPI_AF             GPIO_AF_SPI2     

///////////////////IMU I2C//////////////////////////////////////////////////////////
//the i2c pins are defined in    tm_stm32f4_mpu6050.h      #
//SCL			PA8			Clock line for I2C
//SDA			PC9			Data line for I2C                      
///////////////////UART GPIO//////////////////////////////////////////////////////////
//CPU Uart
#define PORT_CPU_TXD			        GPIOC
#define PORT_CPU_RXD			        GPIOC
#define PIN_CPU_TXD				GPIO_Pin_10
#define PIN_CPU_RXD				GPIO_Pin_11
//Zigbee Uart
#define PIN_ZIGBEE_RXD			        GPIO_Pin_2
#define PIN_ZIGBEE_TXD			        GPIO_Pin_12
#define PORT_ZIGBEE_TXD			        GPIOC
#define PORT_ZIGBEE_RXD			        GPIOD
//DXL Uart
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PORT_DXL_TXD			        GPIOB
#define PORT_DXL_RXD			        GPIOB
//PC uart
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD				GPIO_Pin_11
#define PORT_PC_TXD				GPIOB
#define PORT_PC_RXD				GPIOB

///////////////////ADC Pins//////////////////////////////////////////////////////////
#define PIN_ADC0				GPIO_Pin_0
#define PIN_ADC1				GPIO_Pin_1
#define PIN_ADC2				GPIO_Pin_2
#define PIN_ADC3				GPIO_Pin_3
#define PIN_ADC4				GPIO_Pin_0
#define PIN_ADC5				GPIO_Pin_1
#define PIN_ADC6				GPIO_Pin_2
#define PIN_ADC7				GPIO_Pin_3
#define PIN_ADC8				GPIO_Pin_4
#define PIN_ADC9				GPIO_Pin_5
#define PIN_ADC10				GPIO_Pin_6
#define PIN_ADC11				GPIO_Pin_7
#define PIN_ADC12				GPIO_Pin_4
#define PIN_ADC13				GPIO_Pin_5
#define PIN_ADC14				GPIO_Pin_0
#define PIN_ADC15				GPIO_Pin_1

#define PORT_ADC0				GPIOC
#define PORT_ADC1				GPIOC
#define PORT_ADC2				GPIOC
#define PORT_ADC3				GPIOC
#define PORT_ADC4				GPIOA
#define PORT_ADC5				GPIOA
#define PORT_ADC6				GPIOA
#define PORT_ADC7				GPIOA
#define PORT_ADC8				GPIOA
#define PORT_ADC9				GPIOA
#define PORT_ADC10				GPIOA
#define PORT_ADC11				GPIOA
#define PORT_ADC12				GPIOC
#define PORT_ADC13				GPIOC
#define PORT_ADC14				GPIOB
#define PORT_ADC15				GPIOB

///////////////////GPIO Pins//////////////////////////////////////////////////////////
//#define PIN_LED6_R				GPIO_Pin_8
#define PIN_PA8					GPIO_Pin_8
//#define PIN_LED6_G				GPIO_Pin_11
#define PIN_SW_MODE				GPIO_Pin_11
//#define PIN_LED6_B				GPIO_Pin_12
#define PIN_SW_START			        GPIO_Pin_12
#define PIN_PA13				GPIO_Pin_13

//#define PIN_SW_START			        GPIO_Pin_14
#define PIN_PA15				GPIO_Pin_14

//#define PIN_SW_MODE				GPIO_Pin_15
#define PIN_PA14				GPIO_Pin_15



//#define PORT_LED6_R				GPIOA
#define PORT_PA8				GPIOA



//#define PORT_LED6_G				GPIOA
//#define PORT_LED6_B				GPIOA
#define PORT_SW_MODE			        GPIOA
#define PORT_SW_START			        GPIOA

#define PORT_PA13				GPIOA

#define PORT_PA14				GPIOA
#define PORT_PA15				GPIOA


///////////////////PORTB//////////////////////////////////////////////////////////

#define PIN_BOOT1				GPIO_Pin_2


//#define PIN_ENABLE_ZIGBEE		        GPIO_Pin_3
#define PIN_PB3					GPIO_Pin_3

//#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_PB4					GPIO_Pin_4

#define PIN_ENABLE_RXD			        GPIO_Pin_5

#define PIN_ENABLE_DXLPWR		        GPIO_Pin_8
#define PIN_BUZZER				GPIO_Pin_9

#define PIN_LED3				GPIO_Pin_12
#define PIN_SIG_SCK				GPIO_Pin_13
#define PIN_SIG_MISO			        GPIO_Pin_14
#define PIN_SIG_MOSI			        GPIO_Pin_15

#define PORT_BOOT1				GPIOB

//#define PORT_ENABLE_ZIGBEE	GPIOB
#define PORT_PB3				GPIOB

//#define PORT_ENABLE_TXD			GPIOB
#define PORT_PB4				GPIOB

#define PORT_ENABLE_RXD			        GPIOB

#define PORT_ENABLE_DXLPWR		        GPIOB
#define PORT_BUZZER				GPIOB

#define PORT_LED3				GPIOB
#define PORT_SIG_SCK			        GPIOB
#define PORT_SIG_MISO			        GPIOB
#define PORT_SIG_MOSI			        GPIOB



///////////////////PORTC//////////////////////////////////////////////////////////

#define PIN_LED4				GPIO_Pin_6


//#define PIN_LED5_R				GPIO_Pin_7
#define PIN_PC7					GPIO_Pin_7

//#define PIN_LED5_G				GPIO_Pin_8
#define PIN_ENABLE_ZIGBEE		        GPIO_Pin_8

//#define PIN_LED5_B				GPIO_Pin_9
#define PIN_ENABLE_TXD			        GPIO_Pin_9

#define PIN_SIG_ACC_CS			        GPIO_Pin_10
#define PIN_SIG_GYRO_CS			        GPIO_Pin_11

#define PIN_LED_TX				GPIO_Pin_13
#define PIN_LED_RX				GPIO_Pin_14
#define PIN_LED2				GPIO_Pin_15


#define PORT_LED4				GPIOC

//#define PORT_LED5_R				GPIOC
#define PORT_PC7				GPIOC

//#define PORT_LED5_G				GPIOC
#define PORT_ENABLE_ZIGBEE		        GPIOC

//#define PORT_LED5_B				GPIOC
#define PORT_ENABLE_TXD			        GPIOC

#define PORT_SIG_ACC_CS			        GPIOC
#define PORT_SIG_GYRO_CS		        GPIOC

#define PORT_LED_TX				GPIOC
#define PORT_LED_RX				GPIOC
#define PORT_LED2				GPIOC



///////////////////PORTD///////////////////////////////////////////////////////



/////////////////////////// LED REDEFINE /////////////////////////////////////////
#define PIN_LED_MANAGE		                PIN_LED2
#define PIN_LED_EDIT		                PIN_LED3
#define PIN_LED_PLAY		                PIN_LED4

#define PORT_LED_MANAGE		                PORT_LED2
#define PORT_LED_EDIT		                PORT_LED3
#define PORT_LED_PLAY		                PORT_LED4

//#define PIN_LED_POWER		                PIN_LED6_R

//#define PIN_LED_AUX			        PIN_LED5_R
//#define PIN_LED_AUX			        PIN_PC7

/*


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART_Configuration(u8 PORT, u32 baudrate);
u32 USART_GetBaudrate(u8 PORT);
void ADC_Configuration(void);
void Timer_Configuration(void);
void SysTick_Configuration(void);
void EXTI_Configuration(void);
void System_Configuration(void);
void AHB1Periph_GPIOall(void);
void SPI_Configuration(void);
void Buzzer_Configuration(void);
void vGPIO_Configure(GPIO_TypeDef* GPIOx,
                     uint32_t GPIO_Pin,
                     GPIOMode_TypeDef GPIO_Mode,
                     GPIOSpeed_TypeDef GPIO_Speed,
                     GPIOOType_TypeDef GPIO_OType,
                     GPIOPuPd_TypeDef GPIO_PuPd);



#endif /* __SYSTEM_INIT_H */

/************************ (C) COPYRIGHT 2010 ROBOTIS ********END OF FILE*******/
