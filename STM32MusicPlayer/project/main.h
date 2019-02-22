/******************** (C) COPYRIGHT 2007 STMicroelectronics ********************
* File Name          : main.h
* Author             : MCD Application Team
* Version            : V1.0
* Date               : 10/08/2007
* Description        : Header for main.c module
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stdio.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define USE_USART1
//#define USE_USART2
//#define USE_USART3

#ifdef USE_USART1
  #define  USARTx                   USART1
  #define  GPIOx                    GPIOA
  #define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOA
  #define  GPIO_RxPin               GPIO_Pin_10
  #define  GPIO_TxPin               GPIO_Pin_9
#elif defined USE_USART2
  #define  USARTx                   USART2
  #define  GPIOx                    GPIOD
  #define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOD
  #define  RCC_APB1Periph_USARTx    RCC_APB1Periph_USART2
  #define  GPIO_RxPin               GPIO_Pin_6
  #define  GPIO_TxPin               GPIO_Pin_5
#elif defined USE_USART3
  #define  USARTx                   USART3
  #define  GPIOx                    GPIOB
  #define  RCC_APB2Periph_GPIOx     RCC_APB2Periph_GPIOB
  #define  RCC_APB1Periph_USARTx    RCC_APB1Periph_USART3
  #define  GPIO_RxPin               GPIO_Pin_11
  #define  GPIO_TxPin               GPIO_Pin_10
#endif 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */


/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
