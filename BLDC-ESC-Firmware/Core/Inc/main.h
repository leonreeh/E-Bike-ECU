/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BLDC.h"
#include "FOC.h"
#include "USB.h"
#include <Mymath.h>
#include "Math.h"
#include "liquidcrystal_i2c.h"
#include <stdio.h>

//state machine
extern int STATE;
//rpm & speed calc
extern int hallCC;
extern float rpm;
extern int timcc;


//ADC data [Voltage,Current,Temp,Speed]
extern float ADC_VAL[4];
//Button data[Light,Blinker L, Blinker R, Aux]
extern uint16_t but[4];
extern lcd_ar lcd_val;
extern uint32_t swfault_time_counter;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
//interrupt Handlers
void handleBreakInterrupt();
void handleHardwareFaultInterrupt();
void handleHallSensorInterrupt(uint16_t GPIO_Pin);
//helper functions
void setFaultState(const char* errorMessage);
void writeState();
void ADC3_Select_CH(int ch);
void readADCs();
void doADCs();
void readDI();
void setDO();
void resetDO();
//state functions
void ready();
void drive();
void breaking();
void swfault();
void hwfault();
void debug();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC13_DI_AUX_Pin GPIO_PIN_13
#define PC13_DI_AUX_GPIO_Port GPIOC
#define ADC2_IN10_U_CUR_Pin GPIO_PIN_0
#define ADC2_IN10_U_CUR_GPIO_Port GPIOC
#define ADC2_IN11_V_CUR_Pin GPIO_PIN_1
#define ADC2_IN11_V_CUR_GPIO_Port GPIOC
#define ADC2_IN12_W_CUR_Pin GPIO_PIN_2
#define ADC2_IN12_W_CUR_GPIO_Port GPIOC
#define ADC3_IN13_TEMP_Pin GPIO_PIN_3
#define ADC3_IN13_TEMP_GPIO_Port GPIOC
#define ADC3_IN0_BUS_VOLT_Pin GPIO_PIN_0
#define ADC3_IN0_BUS_VOLT_GPIO_Port GPIOA
#define ADC3_IN1_BUS_CUR_Pin GPIO_PIN_1
#define ADC3_IN1_BUS_CUR_GPIO_Port GPIOA
#define ADC1_IN5_Throttle_Pin GPIO_PIN_5
#define ADC1_IN5_Throttle_GPIO_Port GPIOA
#define TIM3_CH1_Blinker_R_Pin GPIO_PIN_6
#define TIM3_CH1_Blinker_R_GPIO_Port GPIOA
#define TIM3_CH2_Blinker_L_Pin GPIO_PIN_7
#define TIM3_CH2_Blinker_L_GPIO_Port GPIOA
#define EXTI5_Break_Pin GPIO_PIN_5
#define EXTI5_Break_GPIO_Port GPIOC
#define EXTI5_Break_EXTI_IRQn EXTI9_5_IRQn
#define PB1_LED_RED_Pin GPIO_PIN_0
#define PB1_LED_RED_GPIO_Port GPIOB
#define PB0_LED_GREEN_Pin GPIO_PIN_1
#define PB0_LED_GREEN_GPIO_Port GPIOB
#define PB13_U_Pin GPIO_PIN_13
#define PB13_U_GPIO_Port GPIOB
#define PB14_V_Pin GPIO_PIN_14
#define PB14_V_GPIO_Port GPIOB
#define PB15_W_Pin GPIO_PIN_15
#define PB15_W_GPIO_Port GPIOB
#define EXTI6_HALL_U_Pin GPIO_PIN_6
#define EXTI6_HALL_U_GPIO_Port GPIOC
#define EXTI6_HALL_U_EXTI_IRQn EXTI9_5_IRQn
#define EXTI7_HALL_V_Pin GPIO_PIN_7
#define EXTI7_HALL_V_GPIO_Port GPIOC
#define EXTI7_HALL_V_EXTI_IRQn EXTI9_5_IRQn
#define EXTI8_HALL_W_Pin GPIO_PIN_8
#define EXTI8_HALL_W_GPIO_Port GPIOC
#define EXTI8_HALL_W_EXTI_IRQn EXTI9_5_IRQn
#define EXTI9_FAULT_Pin GPIO_PIN_9
#define EXTI9_FAULT_GPIO_Port GPIOC
#define EXTI9_FAULT_EXTI_IRQn EXTI9_5_IRQn
#define TIM1_CH1_U_Pin GPIO_PIN_8
#define TIM1_CH1_U_GPIO_Port GPIOA
#define TIM1_CH2_V_Pin GPIO_PIN_9
#define TIM1_CH2_V_GPIO_Port GPIOA
#define TIM1_CH3_W_Pin GPIO_PIN_10
#define TIM1_CH3_W_GPIO_Port GPIOA
#define PC10_DI_LIGHT_Pin GPIO_PIN_10
#define PC10_DI_LIGHT_GPIO_Port GPIOC
#define PC11_DI_BLINKER_L_Pin GPIO_PIN_11
#define PC11_DI_BLINKER_L_GPIO_Port GPIOC
#define PC12_DI_BLINKER_R_Pin GPIO_PIN_12
#define PC12_DI_BLINKER_R_GPIO_Port GPIOC
#define PB3_DO_LIGHT_Pin GPIO_PIN_3
#define PB3_DO_LIGHT_GPIO_Port GPIOB
#define PB5_DO_DC_ON_Pin GPIO_PIN_5
#define PB5_DO_DC_ON_GPIO_Port GPIOB
#define PB8_DO_FAN_Pin GPIO_PIN_8
#define PB8_DO_FAN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
//Machine States
#define READY 0
#define DRIVE 1
#define BREAK 2
#define SWFAULT 3
#define HWFAULT 4
#define DEBUGST 6
//BLDC
#define THROTTLE_THRESHOLD 30 //Threshold for initial start to prevent adc drift
#define MAXDUTY 100		//Motor PWM Duty Cycle
#define MINDUTY 0		//Motor PWM Duty Cycle
#define MAXADC 3170		//throttle MAX 2,55V
#define MINADC 1055		//throttle MIN 0.85V
#define MAXRPM 4500		//Max RPM of the given motor
#define MINRPM 25		//Min RPM of the given motor
//Software limits for ADC measurments
#define SW_OC 		 22		//Over Current 			20,5A	((20,5*0,004)/20) /12bitADC
#define SW_UV 		 44		//Under Voltage 		43,5V	((43,5/(39000+2200))*2200) /12bitADC
#define SW_OV 		 60		//Over Voltage 			43,5V
#define SW_OT 		 100	//Over Temperature 	95°C	((3,3/(10000+1140))*10000)) /12bitADC
#define Temp_FAN_ON  80	//Turn on fan here	75°C
#define Temp_FAN_OFF 60	//Turn off fan here 50°C
//Blinker PWM
#define BLINKER_START 250 // Run blinker at nom. PWM
#define BLINKER_STOP 500  // Turns off blinker PWM
//other
#define ADC_TIMEOUT 20
#define SWFAULT_TIMOUT 300
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
