/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define DRV3_A_Pin GPIO_PIN_0
#define DRV3_A_GPIO_Port GPIOA
#define DRV3_B_Pin GPIO_PIN_1
#define DRV3_B_GPIO_Port GPIOA
#define DRV3_EN_Pin GPIO_PIN_2
#define DRV3_EN_GPIO_Port GPIOA
#define CSN_Pin GPIO_PIN_4
#define CSN_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define USB_CTL_Pin GPIO_PIN_0
#define USB_CTL_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_1
#define INT_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_10
#define SCL_GPIO_Port GPIOB
#define SDA_Pin GPIO_PIN_11
#define SDA_GPIO_Port GPIOB
#define COUNTER0_Pin GPIO_PIN_12
#define COUNTER0_GPIO_Port GPIOB
#define COUNTER0_EXTI_IRQn EXTI15_10_IRQn
#define COUNTER1_Pin GPIO_PIN_13
#define COUNTER1_GPIO_Port GPIOB
#define COUNTER1_EXTI_IRQn EXTI15_10_IRQn
#define COUNTER2_Pin GPIO_PIN_14
#define COUNTER2_GPIO_Port GPIOB
#define COUNTER2_EXTI_IRQn EXTI15_10_IRQn
#define COUNTER3_Pin GPIO_PIN_15
#define COUNTER3_GPIO_Port GPIOB
#define COUNTER3_EXTI_IRQn EXTI15_10_IRQn
#define DRV2_EN_Pin GPIO_PIN_8
#define DRV2_EN_GPIO_Port GPIOA
#define DRV2_B_Pin GPIO_PIN_9
#define DRV2_B_GPIO_Port GPIOA
#define DRV3_AA10_Pin GPIO_PIN_10
#define DRV3_AA10_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define DRV0_EN_Pin GPIO_PIN_15
#define DRV0_EN_GPIO_Port GPIOA
#define DRV0_A_Pin GPIO_PIN_3
#define DRV0_A_GPIO_Port GPIOB
#define DRV0_B_Pin GPIO_PIN_4
#define DRV0_B_GPIO_Port GPIOB
#define DRV1_EN_Pin GPIO_PIN_5
#define DRV1_EN_GPIO_Port GPIOB
#define DRV1_A_Pin GPIO_PIN_6
#define DRV1_A_GPIO_Port GPIOB
#define DRV1_B_Pin GPIO_PIN_7
#define DRV1_B_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
