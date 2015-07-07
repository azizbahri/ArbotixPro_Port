/**
  ******************************************************************************
  * @file    Template/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    20-September-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "stm32f429i_discovery_lcd.h"

/** @addtogroup Template
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t SysTicDelay;
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t time);
void AHB1Periph_GPIOall(void);
void GPIO_Set_output(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_X,GPIOMode_TypeDef GPIO_Mode_X);
void ConfigureUsart(int baudrate);
void SendData(USART_TypeDef* USARTx, volatile char *s);
/* Private functions ---------------------------------------------------------*/

 
/**
  * @brief   Main program
  * @param  None
  * @retval None
  */

int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f429_439xx.s) before to branch to application main. 
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */  
  /* Add your application code here */
  
  //GPIO Clock Enable
 // AHB1Periph_GPIOall();
  
  /*set up LED*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
   GPIO_Set_output(GPIOG,GPIO_Pin_13,GPIO_Mode_OUT);
    /*Uart Init*/
  //PA9-UART1_TXa
  //PA10-UART1_RX aa
  
ConfigureUsart(9600);
  
  SysTick_Config(SystemCoreClock/1000);
  /* Infinite loop */

  
  
  while (1)
  {
    Delay(1500);
    //set PG13
    //GPIO_ToggleBits(GPIOB,GPIO_Pin_2);
    //GPIOG->ODR ^= GPIO_Pin_14;
    //LCD_DrawCircle((uint16_t)202,(uint16_t)64,(uint16_t)20);
    SendData(USART1, "Aziz");
  }
}


/**
  * @}
  */

void Delay(__IO uint32_t time){
  SysTicDelay = time;
  while(SysTicDelay !=0);
}

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

void GPIO_Set_output(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_X,GPIOMode_TypeDef GPIO_Mode_X){
    //GPIO init PG13(LED)
  GPIO_InitTypeDef GPIO_initStruct;
  GPIO_initStruct.GPIO_Pin = GPIO_Pin_X; //Pin number
  GPIO_initStruct.GPIO_Mode = GPIO_Mode_X;
  GPIO_initStruct.GPIO_OType =GPIO_OType_PP;
  GPIO_initStruct.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_initStruct.GPIO_PuPd = GPIO_PuPd_UP;
  
  
  GPIO_Init(GPIOx,&GPIO_initStruct);
}


void ConfigureUsart(int baudrate){

	//structures used configure the hardware
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	//enable the clocks for the GPIOB and the USART
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	//Initialise pins GPIOA 9 and GPIOA 10
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Connect the TX and RX pins to their alternate function pins
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	//configure USART
	USART_InitStruct.USART_BaudRate = baudrate;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //enable send and receive (Tx and Rx)
	USART_Init(USART1, &USART_InitStruct);

	//Enable the interupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

//writes out a string to the passed in usart. The string is passed as a pointer
void SendData(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
