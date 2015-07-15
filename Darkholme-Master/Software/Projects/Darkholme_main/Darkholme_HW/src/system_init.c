/************************* (C) COPYRIGHT 2010 ROBOTIS *************************
* File Name          : system_init.c
* Author             : zerom
* Version            : V0.1
* Date               : 2010/08/25
* Description        : functions about system init
*******************************************************************************/
/************************* PROJECT DARKHOLME **************************
* File Name          : main.c
* Author             : Aziz
* Version            : V0.0.1
* Date               : 07/07/2015
* Description        : Main program body ported for STM32F429I
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"
#include "common_type.h"
#include "system_init.h"
#include "system_func.h"
#include "usart.h"
#include "dynamixel.h"
#include "zigbee.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/




// CCR unit= 10uS
vu16 CCR1_Val = 100; 		// 1ms
vu16 CCR2_Val = 778; 		// 7.81ms
vu16 CCR3_Val = 12400;    	// 125ms
vu16 CCR4_Val = 12;    		// 12us

u32 Baudrate_DXL = 9600;//	1000000;
u32 Baudrate_ZIGBEE = 9600;//57600;
//u32 Baudrate_PC = 57600;
u32 Baudrate_PC = 9600;//1000000;

//u8 SPI_Data_Transmit_Complete=FALSE;

/* Private function prototypes -----------------------------------------------*/


/* Private functions ---------------------------------------------------------*/

/**
* @brief  sets the specified pin on the specified port
*/
void vGPIO_Configure(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed,GPIOOType_TypeDef GPIO_OType,GPIOPuPd_TypeDef GPIO_PuPd){
  GPIO_InitTypeDef GPIO_InitStructure;
  //initialize DXL uart pins
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
  GPIO_InitStructure.GPIO_OType = GPIO_OType;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;
  GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void System_Configuration(void)
{
  
  __disable_interrupt();
  /* System Clocks Configuration */
  RCC_Configuration();            //RCC configuration is made in system_stm32f4xx.c
  
  /* NVIC configuration */
  NVIC_Configuration();
  
  
  /* Configure the GPIO ports */
  GPIO_Configuration();
  
  
  
  /* Unlock the Flash Program Erase controller */
  FLASH_Unlock();
  
  /* USART Configuration */
  USART_Configuration(USART_DXL,Baudrate_DXL);
  //dxl_initialize(USART_DXL,Baudrate_DXL);
  //zgb_initialize(0);
  USART_Configuration(USART_ZIGBEE,Baudrate_ZIGBEE);
  
  //USART_Configuration(USART_PC,1000000);
  //USART_Configuration(USART_PC,3000000);
  USART_Configuration(USART_PC,Baudrate_PC);
  
  
  /* ADC Configuration */
  ADC_Configuration();
  
  
  
  SysTick_Configuration();
  
  Timer_Configuration();
  
  
  SPI_Configuration();
  
  Buzzer_Configuration();
  
  
  GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);	// TX Disable
  GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);	// RX Enable
  GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);
  GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);
  
  __enable_interrupt();
  
  
  
  Gyro_Configuration();
  ACC_Configuration();
  
  
  
  
  
  
}



void Buzzer_Configuration(void)
{
  
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  
  // Timer Base Init	- Buzzer
  TIM_DeInit(TIM4);
  TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = 2000;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  // PWM Init			- Buzzer
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure. TIM_Period / 2;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);
  TIM_Cmd(TIM4, ENABLE);
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
  
  
}



void Timer_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  TIM_DeInit(TIM2);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);
  
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  /*
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val ;
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);
  
  */
  /*
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val ;
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
  
  
  
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val ;
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Disable);
  */
  
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val ;
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
  
  
  /* TIM IT enable */
  TIM_ITConfig(TIM2, /*TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 |*/ TIM_IT_CC4 , ENABLE);
  
  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

//#define NO_RESET			0
//#define PIN_RESET			1
//#define POWER_RESET		2
//#define SOFT_RESET		3
//#define IWDG_RESET		4 // independent watchdog reset
//#define WWDG_RESET		5 // window watchdog reset
//#define LOW_POWER_RESET 	6	


void SysTick_Configuration(void)
{
  
  //system clock 180Mhz, 1ms pulse
  SysTick_Config(SystemCoreClock/1000);
  //  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
  //  SysTick_SetReload(9000);
  //  
  //  /* Enable SysTick interrupt */
  //  SysTick_ITConfig(ENABLE);
}




/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void RCC_Configuration(void)
{
  //	ErrorStatus HSEStartUpStatus;
  //	/* RCC system reset(for debug purpose) */
  //	RCC_DeInit();
  //
  //	/* Enable HSE */
  //	RCC_HSEConfig(RCC_HSE_ON);
  //
  //	/* Wait till HSE is ready */
  //	HSEStartUpStatus = RCC_WaitForHSEStartUp();
  //
  //	if(HSEStartUpStatus == SUCCESS)
  //	{
  //		/* Enable Prefetch Buffer */
  //		FLASH_PrefetchBufferCmd(ENABLE);   //the argument for this has changed
  //
  //		/* Flash 2 wait state */
  //		FLASH_SetLatency(FLASH_Latency_2);
  //
  //		/* HCLK = SYSCLK */
  //		RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  //
  //		/* PCLK2 = HCLK */
  //		RCC_PCLK2Config(RCC_HCLK_Div1); 
  //
  //		/* PCLK1 = HCLK/2 */
  //		RCC_PCLK1Config(RCC_HCLK_Div2);
  //
  //		/* PLLCLK = 8MHz * 9 = 72 MHz */
  //		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
  //
  //		/* Enable PLL */ 
  //		RCC_PLLCmd(ENABLE);
  //
  //		/* Wait till PLL is ready */
  //		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
  //		{
  //		}
  //
  //		/* Select PLL as system clock source */
  //		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  //
  //		/* Wait till PLL is used as system clock source */
  //		while(RCC_GetSYSCLKSource() != 0x08)
  //		{
  //		}
  //	} 
  // 
  //	/* Enable peripheral clocks --------------------------------------------------*/
  //  //enable all GPIO peripheral clocks
  AHB1Periph_GPIOall();
  /* Enable USART5, GPIOA,and AFIO clocks */
  /* Enable USART5, GPIOA, GPIOB, and AFIO clocks */
  
  //Usart 1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
  //TIM1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
  //TM8
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
  //ADC1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
  //ADC2
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);
  //TIM2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
  //TIM3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
  //TIM4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
  //TIM5
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
  //USART3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  //UART5
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
  //SPI2
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  //PWR
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);
  
  
  PWR_BackupAccessCmd(ENABLE);
}



void USART_Configuration(u8 PORT, u32 baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  
  USART_StructInit(&USART_InitStructure);
  
  
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  
  if( PORT == USART_DXL )
  {
    Baudrate_DXL = baudrate;
    
    USART_DeInit(DXL_USART);
    /* Configure the USART1 */
    USART_Init(DXL_USART, &USART_InitStructure);
    
    /* Enable USART1 Receive and Transmit interrupts */
    USART_ITConfig(DXL_USART, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    
    /* Enable the USART1 */
    USART_Cmd(DXL_USART, ENABLE);
  }
  else if( PORT == USART_ZIGBEE )
  {
    Baudrate_ZIGBEE = baudrate;
    
    //USART_DeInit(ZIGBEE_USART);
    /* Configure the UART5 */
    USART_Init(ZIGBEE_USART, &USART_InitStructure);
    
    
    /* Enable UART5 Receive and Transmit interrupts */
    USART_ITConfig(ZIGBEE_USART, USART_IT_RXNE, ENABLE);
    
    /* Enable the UART5 */
    USART_Cmd(ZIGBEE_USART, ENABLE);
  }
  else if( PORT == USART_PC )
  {
    Baudrate_PC = baudrate;
    
    USART_DeInit(PC_USART );
    
    /* Configure the USART3 */
    USART_Init(PC_USART , &USART_InitStructure);
    
    /* Enable USART3 Receive and Transmit interrupts */
    USART_ITConfig(PC_USART , USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART3, USART_IT_TC, ENABLE);
    
    /* Enable the USART3 */
    USART_Cmd(PC_USART , ENABLE);
  }
  
}
u32 USART_GetBaudrate(u8 PORT)
{
  
  if( PORT == USART_DXL )
  {
    return Baudrate_DXL;
  }
  else if( PORT == USART_ZIGBEE )
  {
    return Baudrate_ZIGBEE;
  }
  else if( PORT == USART_PC )
  {
    return Baudrate_PC;
  }
  
  return 0;
}

void ADC_Configuration(void)
{
  
  ADC_InitTypeDef ADC_InitStructure;
  
  ADC_StructInit(&ADC_InitStructure);
  
  //Clock configuration
  // RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);//The ADC1 is connected the APB2 peripheral bus thus we will use its clock source
  //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2,ENABLE);
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_DeInit();
  ADC_DeInit();
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//data converted will be shifted to right
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//Input voltage is converted into a 12bit number giving a maximum value of 4096
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //the conversion is continuous, the input data is converted more than once
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;// conversion is synchronous with TIM1 and CC1 (actually I'm not sure about this one :/)
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//no trigger for conversion
  ADC_InitStructure.ADC_NbrOfConversion = 1;//I think this one is clear :p
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//The scan is configured in one channel
  
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_Init(ADC2, &ADC_InitStructure);
  
  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC1_Channel_X, 1 , ADC_SampleTime_480Cycles);
  ADC_ITConfig(ADC1, ADC_IT_EOC, DISABLE);
  
  /* ADC2 regular channels configuration */
  ADC_RegularChannelConfig(ADC2, ADC2_Channel_X, 1, ADC_SampleTime_480Cycles);
  ADC_ITConfig(ADC2, ADC_IT_EOC, DISABLE);
  
  /* Enable ADC1 DMA */
  //ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1,2 */
  ADC_Cmd(ADC1, ENABLE);
  ADC_Cmd(ADC2, ENABLE);
  
  
  
  
  //F4 chips no longer require calibration ADC
  
  
  /* Start ADC2 Software Conversion */
  ADC_SoftwareStartConv(ADC1);
  ADC_SoftwareStartConv(ADC2);
}


void SPI_Configuration(void)
{
  
  //SPI_StructInit(&SPI_InitStructure);
  
  SPI_InitTypeDef   SPI_InitStructure;
  
  GPIO_SetBits(PORT_SIG_GYRO_CS,PIN_SIG_GYRO_CS);
  GPIO_SetBits(PORT_SIG_ACC_CS,PIN_SIG_ACC_CS);
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  
  //SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Init(IMU_SPI, &SPI_InitStructure);
  
  
  /* Enable SPI2 RXNE interrupt */
  //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, DISABLE);
  //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE);
  
  /* Enable SPI1 */
  //SPI_Cmd(SPI1, ENABLE);
  
  /* Enable SPI2 */
  SPI_Cmd(SPI2, ENABLE);
  
  
}
/*******************************************************************************
* Function Name  : AHB1Periph_GPIOall
* Description    : Peripheral clock enable
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AHB1Periph_GPIOall(void){
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOK, ENABLE);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{
  
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_StructInit(&GPIO_InitStruct);

  
  
  
  /*-------- Configuring User GPIO pins --------*/
  vGPIO_Configure(PORT_PA8,
                  PIN_PA8,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  
  /*-------- Configuring PIN_CPU_RXD and PIN_CPU_TXD --------*/
  
  //PIN_CPU_RXD
  
  GPIO_InitStruct.GPIO_Pin = PIN_CPU_RXD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(PORT_CPU_RXD, &GPIO_InitStruct);
  GPIO_PinAFConfig(PORT_CPU_RXD, GPIO_PinSource11, CPU_USART_AF); 
  
  //PIN_CPU_TXD
  
  GPIO_InitStruct.GPIO_Pin = PIN_CPU_TXD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(PORT_CPU_TXD, &GPIO_InitStruct);
  
  GPIO_PinAFConfig(PORT_CPU_TXD, GPIO_PinSource10, CPU_USART_AF);
  
  /*-------- Configuring DXL uart pins --------*/
  GPIO_InitStruct.GPIO_Pin = PIN_DXL_RXD | PIN_DXL_TXD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(PORT_DXL_RXD, &GPIO_InitStruct);
  GPIO_PinAFConfig(PORT_DXL_RXD, GPIO_PinSource6, DXL_USART_AF); //
  GPIO_PinAFConfig(PORT_DXL_RXD, GPIO_PinSource7, DXL_USART_AF);
  
  //PIN_ENABLE_TXD
  vGPIO_Configure(PORT_ENABLE_TXD ,
                  PIN_ENABLE_TXD ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  
    
  /*-------- Configuring PC uart pins--------*/
  
  //PIN_PC_RXD
  GPIO_InitStruct.GPIO_Pin = PIN_PC_RXD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(PORT_PC_RXD, &GPIO_InitStruct);
 
  GPIO_PinAFConfig(PORT_PC_RXD, GPIO_PinSource11, PC_USART_AF);
  
  //PIN_PC_TXD
  GPIO_InitStruct.GPIO_Pin = PIN_PC_TXD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(PORT_PC_TXD, &GPIO_InitStruct);
  
  GPIO_PinAFConfig(PORT_PC_TXD, GPIO_PinSource10, PC_USART_AF);

  
    /*-------- Configuring Zigbee pins--------*/
  //PIN_ENABLE_ZIGBEE
  vGPIO_Configure(PORT_ENABLE_ZIGBEE ,
                  PIN_ENABLE_ZIGBEE ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  //PIN_ZIGBEE_TXD
  
  GPIO_InitStruct.GPIO_Pin = PIN_ZIGBEE_TXD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(PORT_ZIGBEE_TXD, &GPIO_InitStruct);
  
  GPIO_PinAFConfig(PORT_ZIGBEE_TXD, GPIO_PinSource12, ZIGBEE_USART_AF);
  
  //PIN_ZIGBEE_RXD
  GPIO_InitStruct.GPIO_Pin = PIN_ZIGBEE_RXD;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(PORT_ZIGBEE_RXD, &GPIO_InitStruct);
  
  GPIO_PinAFConfig(PORT_ZIGBEE_RXD, GPIO_PinSource12, ZIGBEE_USART_AF);
  
  /*-------- Configuring Analogue pins --------*/
  //PIN_ADC0
  vGPIO_Configure(PORT_ADC0 ,
                  PIN_ADC0 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  
  //PIN_ADC1
  vGPIO_Configure(PORT_ADC1 ,
                  PIN_ADC1 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC2
  vGPIO_Configure(PORT_ADC2 ,
                  PIN_ADC2 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC3
  vGPIO_Configure(PORT_ADC3 ,
                  PIN_ADC3 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC4
  vGPIO_Configure(PORT_ADC4 ,
                  PIN_ADC4 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC5
  vGPIO_Configure(PORT_ADC5 ,
                  PIN_ADC5 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC6
  vGPIO_Configure(PORT_ADC6 ,
                  PIN_ADC6 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC7
  vGPIO_Configure(PORT_ADC7 ,
                  PIN_ADC7 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC8
  vGPIO_Configure(PORT_ADC8 ,
                  PIN_ADC8 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC9
  vGPIO_Configure(PORT_ADC9 ,
                  PIN_ADC9 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC10
  vGPIO_Configure(PORT_ADC10 ,
                  PIN_ADC10 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC11
  vGPIO_Configure(PORT_ADC11 ,
                  PIN_ADC11 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC12
  vGPIO_Configure(PORT_ADC12 ,
                  PIN_ADC12 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  //PIN_ADC13
  vGPIO_Configure(PORT_ADC13 ,
                  PIN_ADC13 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  
  //PIN_ADC14
  vGPIO_Configure(PORT_ADC14 ,
                  PIN_ADC14 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  
  //PIN_ADC15
  vGPIO_Configure(PORT_ADC15 ,
                  PIN_ADC15 ,
                  GPIO_Mode_AIN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  
  /*-------- Configuring Switches --------*/
  //PIN_SW_MODE
  vGPIO_Configure(PORT_SW_MODE ,
                  PIN_SW_MODE ,
                  GPIO_Mode_IN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);        //not sure up or down
  //PIN_SW_START
  vGPIO_Configure(PORT_SW_START ,
                  PIN_SW_START ,
                  GPIO_Mode_IN,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  
  /*-------- Configuring Enable pins --------*/
  //PIN_ENABLE_RXD
  vGPIO_Configure(PORT_ENABLE_RXD ,
                  PIN_ENABLE_RXD ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  //PIN_ENABLE_DXLPWR
  vGPIO_Configure(PORT_ENABLE_DXLPWR ,
                  PIN_ENABLE_DXLPWR ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  
  /*-------- Configuring LED Pins --------*/
  
  //PIN_LED3
  vGPIO_Configure(PORT_LED3 ,
                  PIN_LED3 ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);    
  //PIN_LED4
  vGPIO_Configure(PORT_LED4 ,
                  PIN_LED4 ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  //PIN_LED_TX
  vGPIO_Configure(PORT_LED_TX ,
                  PIN_LED_TX ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  //PIN_LED_RX
  vGPIO_Configure(PORT_LED_RX ,
                  PIN_LED_RX ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  //PIN_LED2
  vGPIO_Configure(PORT_LED2 ,
                  PIN_LED2 ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  


  
  
  /*-------- Configuring SCK, MISO, MOSI --------*/
  
  //PIN_SIG_SCK
  vGPIO_Configure(PORT_SIG_SCK ,
                  PIN_SIG_SCK ,
                  GPIO_Mode_AF,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  GPIO_PinAFConfig(PORT_SIG_SCK, PIN_SIG_SCK, SPI_AF);
  
  //PIN_SIG_MOSI
  vGPIO_Configure(PORT_SIG_MOSI ,
                  PIN_SIG_MOSI ,
                  GPIO_Mode_AF,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  GPIO_PinAFConfig(PORT_SIG_MOSI, PIN_SIG_MOSI, SPI_AF);
  
  //PIN_SIG_MISO
  vGPIO_Configure(PORT_SIG_MISO ,
                  PIN_SIG_MISO ,
                  GPIO_Mode_AF,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_NOPULL);
  GPIO_PinAFConfig(PORT_SIG_MISO, PIN_SIG_MISO, SPI_AF);
  
  
  /*-------- Configuring Buzzer Pin --------*/
  //PIN_BUZZER
  vGPIO_Configure(PORT_BUZZER ,
                  PIN_BUZZER ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  
  
  /*-------- Configuring Chip select pins --------*/
  
  //PIN_SIG_ACC_CS
  vGPIO_Configure(PORT_SIG_ACC_CS ,
                  PIN_SIG_ACC_CS ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  
  //PIN_SIG_GYRO_CS
  vGPIO_Configure(PORT_SIG_GYRO_CS ,
                  PIN_SIG_GYRO_CS ,
                  GPIO_Mode_OUT,
                  GPIO_Speed_50MHz,
                  GPIO_OType_PP,
                  GPIO_PuPd_UP);
  
  

  
  
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  //	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);   
  //	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);     
  
  
#ifdef  VECT_TAB_RAM  
  // Set the Vector Table base location at 0x20000000  
  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  // VECT_TAB_FLASH  
  // Set the Vector Table base location at 0x08003000
  //	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);   
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);     
#endif
  
  
  // Configure the NVIC Preemption Priority Bits   
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  // Enable the USART1 Interrupt 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  
  
  /* Enable the TIM2 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
  
  
  
  /* Configure and enable SPI2 interrupt -------------------------------------*/
  /*
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_Init(&NVIC_InitStructure);
  */
  
  /* Configure and enable ADC interrupt */
  
  //NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQChannel;
  //NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  //NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  //NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //NVIC_Init(&NVIC_InitStructure);
  
}




/******************* (C) COPYRIGHT 2010 ROBOTIS *****END OF FILE****/
