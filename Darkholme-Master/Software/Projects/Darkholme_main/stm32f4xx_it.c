/**
******************************************************************************
* @file    Template/stm32f4xx_it.c 
* @author  MCD Application Team
* @version V1.0.0
* @date    20-September-2013
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and 
*          peripherals interrupt service routine.
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
#include "stm32f4xx_it.h"
#include "main.h"
#include "usart.h"
#include "isr.h"
#include "system_func.h"
#include "system_init.h"
#include "led.h"
#include "CM_DXL_COM.h"

/** @addtogroup Template
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void USART1_IRQHandler(void);
/* Private functions ---------------------------------------------------------*/

/* Current mode */
extern u16 gwCurrentMode;   // from mode.c
extern vu16 CCR1_Val;       // from system_init.c
extern vu16 CCR2_Val;       // from system_init.c
extern vu16 CCR3_Val;       // from system_init.c
extern vu16 CCR4_Val;       // from system_init.c

//vu32 capture = 0;
vu8 Counter = 0;
vu16 gwCounter1 = 0;

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
* @brief   This function handles NMI exception.
* @param  None
* @retval None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval None
*/
void SVC_Handler(void)
{
}

/**
* @brief  This function handles Debug Monitor exception.
* @param  None
* @retval None
*/
void DebugMon_Handler(void)
{
}

/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval None
*/
void PendSV_Handler(void)
{
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/

void SysTick_Handler(void)
{
  __ISR_DELAY();
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f429_439xx.s).                                               */
/******************************************************************************/

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval None
*/
/*void PPP_IRQHandler(void)
{
}*/
void USART1_IRQHandler(void){
  ISR_USART_DXL();
}


/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
  ISR_USART_PC();
}



/*******************************************************************************
* Function Name  : UART5_IRQHandler
* Description    : This function handles UART5 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART5_IRQHandler(void)
{
  ISR_USART_ZIGBEE();
}

/**
* @}
*/ 
/*******************************************************************************
* Function Name  : ADC1_2_IRQHandler
* Description    : This function handles ADC1 and ADC2 global interrupts requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC_IRQHandler(void)
{
  ISR_ADC();
}

/*******************************************************************************
* Function Name  : TIM2_IRQHandler
* Description    : This function handles TIM2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TIM2_IRQHandler(void)
{
  static byte b1Sec=0;
  
  //checks for capture compare flag,
  //clears it
  //read adc
  //resets counter back to 0
  //resets cc value
  if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET) // 120us, 8000Hz
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    ISR_ADC();
    TIM_SetCounter(TIM2, 0);
    TIM_SetCompare4(TIM2, CCR4_Val);
    
    
    if( !( gwCounter1 & 7 ) ) // 840us
    {
      ISR_1ms_TIMER();
    }
    
    if( !( gwCounter1 & 3 ) ) // 480us, 2000Hz
    {
      ISR_LED_RGB_TIMER();
      
    }
    if( !( gwCounter1 & 31 ) ) // 3840us, 250Hz
    {
        //call IMU data
        
      ISR_IMU_READ();    //MPU6050 is on I2C port
      //ISR_SPI_READ(); //ArbotixPro IMU is on the SPI port
      //__ISR_Buzzer_Manage();  Removed sound
      GB_BUTTON = ReadButton();
    }
    
    if( !( gwCounter1 & 0x3FF ) ) // 125ms
    {
      LED_SetState(LED_RX,OFF);
      LED_SetState(LED_TX,OFF);
      
      if( !(b1Sec&0x07) )
      {
        ISR_BATTERY_CHECK();
      }
      
      b1Sec++;
      
      
    }
    
    
    
    /*
    if( !( Counter1 & 32 ) ) // 3960us, 250Hz
    {
    
  }
    */
    gwCounter1++;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
