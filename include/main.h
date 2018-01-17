/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
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
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CHG_EN_Pin GPIO_PIN_4
#define CHG_EN_GPIO_Port GPIOE
#define CHRG_Pin GPIO_PIN_5
#define CHRG_GPIO_Port GPIOE
#define ACPR_Pin GPIO_PIN_6
#define ACPR_GPIO_Port GPIOE
#define LOOP_IT_Pin GPIO_PIN_0
#define LOOP_IT_GPIO_Port GPIOA
#define ADC_3V3_Pin GPIO_PIN_1
#define ADC_3V3_GPIO_Port GPIOA
#define ADC_5V_Pin GPIO_PIN_2
#define ADC_5V_GPIO_Port GPIOA
#define ADC_12V_Pin GPIO_PIN_3
#define ADC_12V_GPIO_Port GPIOA
#define ADC_VBAT_Pin GPIO_PIN_4
#define ADC_VBAT_GPIO_Port GPIOA
#define ADC_VIN_Pin GPIO_PIN_5
#define ADC_VIN_GPIO_Port GPIOA
#define RUN_5V_Pin GPIO_PIN_4
#define RUN_5V_GPIO_Port GPIOC
#define RUN_12V_Pin GPIO_PIN_5
#define RUN_12V_GPIO_Port GPIOC
#define LOOP_ADC3_Pin GPIO_PIN_0
#define LOOP_ADC3_GPIO_Port GPIOB
#define LOOP_ADC2_Pin GPIO_PIN_1
#define LOOP_ADC2_GPIO_Port GPIOB
#define OUT_HS_ON_Pin GPIO_PIN_8
#define OUT_HS_ON_GPIO_Port GPIOE
#define OUT_LS_ON_Pin GPIO_PIN_9
#define OUT_LS_ON_GPIO_Port GPIOE
#define OUT_RELAY_Pin GPIO_PIN_10
#define OUT_RELAY_GPIO_Port GPIOE
#define OUT_BUZZER_Pin GPIO_PIN_11
#define OUT_BUZZER_GPIO_Port GPIOE
#define SOL_IN_Pin GPIO_PIN_8
#define SOL_IN_GPIO_Port GPIOD
#define SOL_ST_Pin GPIO_PIN_9
#define SOL_ST_GPIO_Port GPIOD
#define AO_SEL_Pin GPIO_PIN_11
#define AO_SEL_GPIO_Port GPIOD
#define CO_POL_Pin GPIO_PIN_12
#define CO_POL_GPIO_Port GPIOD
#define CO1_Pin GPIO_PIN_13
#define CO1_GPIO_Port GPIOD
#define CO2_Pin GPIO_PIN_14
#define CO2_GPIO_Port GPIOD
#define CO3_Pin GPIO_PIN_15
#define CO3_GPIO_Port GPIOD
#define BT_PWR_EN_Pin GPIO_PIN_0
#define BT_PWR_EN_GPIO_Port GPIOD
#define BT_RESET_Pin GPIO_PIN_1
#define BT_RESET_GPIO_Port GPIOD
#define RS485_TXE_Pin GPIO_PIN_3
#define RS485_TXE_GPIO_Port GPIOD
#define RS485_TE_Pin GPIO_PIN_4
#define RS485_TE_GPIO_Port GPIOD
#define RS485_RXE_Pin GPIO_PIN_7
#define RS485_RXE_GPIO_Port GPIOD
#define PGOOD_3V3_Pin GPIO_PIN_5
#define PGOOD_3V3_GPIO_Port GPIOB
#define PGOOD_5V_Pin GPIO_PIN_6
#define PGOOD_5V_GPIO_Port GPIOB
#define PGOOD_12V_Pin GPIO_PIN_7
#define PGOOD_12V_GPIO_Port GPIOB
#define SS_FAULT_Pin GPIO_PIN_0
#define SS_FAULT_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
