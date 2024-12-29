/*
 * BLDC.c
 *
 *  Created on: Jun 29, 2024
 *      Author: Leon
 */
#include "main.h"
#include "BLDC.h"
#include "math.h"
#include "Mymath.h"

/**
  * @brief Takes hallsensor states and determines the commutator position
  * @param hall[] 		= arry containing the values read from the 3 Hall Sensors
  * @retval			= The Communator step between 0-5 used to set BLDC gates
  * Hall Sensor State	Rotor Position	Electrical Angle (Degrees)
  *				001		0° - 60°		0° - 1440°
  *				101		60° - 120°		1440° - 2880°
  *				100		120° - 180°		2880° - 4320°
  *				110		180° - 240°		4320° - 5760°
  *				010		240° - 300°		5760° - 7200°
  *				011		300° - 360°		7200° - 8640°
  *
  */
uint16_t hallState(uint16_t hall[]){

	uint16_t commutatorState = -1;
	  if ((hall[0] == 1) && (hall[1] == 0) && (hall[2] == 1)) {
		//Mechanical Angle 0°-60°
		  commutatorState = 0;
	  }
	  else if  ((hall[0] == 0) && (hall[1] == 0) && (hall[2] == 1)) {
		//Mechanical Angle 60°-120°
		  commutatorState = 1;
	  }
	  else if  ((hall[0] == 0) && (hall[1] == 1) && (hall[2] == 1)) {
		//Mechanical Angle 120°-180°
		  commutatorState = 2;
	  }
	  else if  ((hall[0] == 0) && (hall[1] == 1) && (hall[2] == 0)) {
		//Mechanical Angle 180°-240°
		  commutatorState = 3;
	  }
	  else if  ((hall[0] == 1) && (hall[1] == 1) && (hall[2] == 0)) {
		//Mechanical Angle 240°-300°
		  commutatorState = 4;
	  }
	  else if  ((hall[0] == 1) && (hall[1] == 0) && (hall[2] == 0)) {
		//Mechanical Angle 300°-360°
		  commutatorState = 5;
	  }
	  return commutatorState;
}



/**
  * @brief Sets Three Phase gate driver according to commutator position
  * @param commutatorStep 		= 0-5 determined by FOC hall sensor position
  * @param duty					= PWM duty cycle 0-100
  * @param dir					= motor direction: 1 = fwr; -1= rws
  * @retval			= void
  */
void commutator(int commutatorStep, int duty, int dir){

	if(dir ==1){
		switch(commutatorStep){

		//U PWM
		//Hall state: 		U = HIGH	V = LOW		W = HIGH
		//Phase Current:	U = I		V =	-I		W = 0
		//High Side			U = PWM		V = OFF		W = OFF
		//Low Side			U = OFF		V = ON		W = OFF
		case 0:
			//Low Side
			HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_RESET);
			//High Side
			TIM1->CCR1 = duty;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;

			break;

		//W PWM
		//Hall state: 		U = LOW		V = LOW		W = HIGH
		//Phase Current:	U = 0		V =	-I		W = I
		//High Side			U = OFF		V = OFF		W = PWM
		//Low Side			U = OFF		V = ON		W = OFF
		case 1:
			//Low Side
			HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_RESET);
			//High Side
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = duty;

			break;

		//W PWM
		//Hall state: 		U = LOW		V = HIGH	W = HIGH
		//Phase Current:	U = -I		V =	0		W = I
		//High Side			U = OFF		V = OFF		W = PWM
		//Low Side			U = ON		V = OFF		W = OFF
		case 2:
			//Low Side
			HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_RESET);
			//High Side
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = duty;

			break;

		//V PWM
		//Hall state: 		U = LOW		V = HIGH	W = LOW
		//Phase Current:	U = -I		V =	I		W = 0
		//High Side			U = OFF		V = PWM		W = OFF
		//Low Side			U = ON		V = OFF		W = OFF
		case 3:
			//Low Side
			HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_RESET);
			//High Side
			TIM1->CCR1 = 0;
			TIM1->CCR2 = duty;
			TIM1->CCR3 = 0;

			break;

		//V PWM
		//Hall state: 		U = HIGH; 	V = HIGH	W = LOW
		//Phase Current:	U = 0		V =	I		W = -I
		//High Side			U = OFF		V = PWM		W = OFF
		//Low Side			U = OFF		V = OFF		W = ON
		case 4:
			//Low Side
			HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_SET);
			//High Side
			TIM1->CCR1 = 0;
			TIM1->CCR2 = duty;
			TIM1->CCR3 = 0;

			break;

		//U PWM
		//Hall state: 		U = HIGH	V = LOW		W = LOW
		//Phase Current:	U = I		V =	0		W = -I
		//High Side			U = PWM		V = OFF		W = OFF
		//Low Side			U = OFF		V = OFF		W = ON
		case 5:
			//Low Side
			HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_SET);
			//High Side
			TIM1->CCR1 = duty;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;

			break;

		//i have no idea how you ended up here pls stop motor
		//High Side			U = OFF		V = OFF		W = OFF
		//Low Side			U = HIGH	V = HIGH	W = HIGH
		default:
			HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_SET);

			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			break;
		}
	}
	if(dir !=1){
		//not implimented yet just break
		HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_SET);

		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		TIM1->CCR3 = 0;
	}
}
/**
  * @brief Initializes the BLDC motor by reading hall sensor values and setting the commutator step
  * @param None
  * @retval void
  */
void initBLDC(){
	uint16_t hall[3];
	hall[0]= (GPIOC->IDR & GPIO_IDR_ID6)? 0x0001 : 0x0000; // Sensor A
	hall[1]= (GPIOC->IDR & GPIO_IDR_ID7)? 0x0001 : 0x0000; // Sensor B
	hall[2]= (GPIOC->IDR & GPIO_IDR_ID8)? 0x0001 : 0x0000; // Sensor C

	int step= hallState(hall);
	commutator(step, 15,1);
}

/**
  * @brief Engages the brake for the BLDC motor by setting all output phases high and stopping PWM signals
  * @param None
  * @retval void
  */
void BLDCbreak(){
	HAL_GPIO_WritePin(GPIOB,PB13_U_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,PB14_V_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,PB15_W_Pin,GPIO_PIN_SET);

	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
}

