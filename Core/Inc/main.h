/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//Estructura para definir el vector posicion

typedef struct Vector3D{
	double x;
	double y;
	double z;
}Vec3D;

typedef enum { false, true } bool;
typedef enum {INIT,READY,WORKING,HOME,DEMO,FAULT} statesMachine;

typedef struct Motor{

	double theta;     		//Posicion angular[degrees]
	double omega;     		//Velocidad angular[rad/s]
	double rpm;				//Velocidad angular[rev/min]

	bool hom;				//Booleano que nos permite saber si un motor a logrado hacer un homing con exito
	bool stepReached;		//Booleano utilizado para indicar cuando un motor a llegado a su posicion objetivo

	double currentAngle;	//Este valor almacena el angulo actual del motor
	double calcStep; 		//Numero de pasos a realizar con punto flotante
	uint32_t numStep;		//Numero exacto de pasos a realizar
	double remainder;		//Remanente para compensar la perdida de pasos por casteo a int

	uint32_t pMotor;		//Contador que se incrementa con las interrupciones de inputCapture para saber si un motor llego a la posicion deseada

}Motor;

extern bool faultDrivers,endStopAlarmInf,endStopAlarmSup,continuar;
extern bool timeFlag;
extern uint8_t rx_index,rx_buffer[30],rx_data;
extern statesMachine state;
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUTTON_Pin GPIO_PIN_13
#define BUTTON_GPIO_Port GPIOC
#define BUTTON_EXTI_IRQn EXTI15_10_IRQn
#define S_DirPaP3_Pin GPIO_PIN_0
#define S_DirPaP3_GPIO_Port GPIOC
#define S_DirPaP2_Pin GPIO_PIN_1
#define S_DirPaP2_GPIO_Port GPIOC
#define stepFeedback1_Pin GPIO_PIN_0
#define stepFeedback1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define S_PulsePaP2_Pin GPIO_PIN_6
#define S_PulsePaP2_GPIO_Port GPIOA
#define faultDriver3_Pin GPIO_PIN_7
#define faultDriver3_GPIO_Port GPIOA
#define faultDriver3_EXTI_IRQn EXTI9_5_IRQn
#define S_Enable_3_Pin GPIO_PIN_5
#define S_Enable_3_GPIO_Port GPIOC
#define S_DirPaP1_Pin GPIO_PIN_0
#define S_DirPaP1_GPIO_Port GPIOB
#define faultDriver2_Pin GPIO_PIN_14
#define faultDriver2_GPIO_Port GPIOB
#define faultDriver2_EXTI_IRQn EXTI15_10_IRQn
#define faultDriver1_Pin GPIO_PIN_15
#define faultDriver1_GPIO_Port GPIOB
#define faultDriver1_EXTI_IRQn EXTI15_10_IRQn
#define S_Enable_2_Pin GPIO_PIN_6
#define S_Enable_2_GPIO_Port GPIOC
#define S_Enable_1_Pin GPIO_PIN_8
#define S_Enable_1_GPIO_Port GPIOC
#define S_PulsePaP1_Pin GPIO_PIN_8
#define S_PulsePaP1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define E_EndStop3_Sup_Pin GPIO_PIN_10
#define E_EndStop3_Sup_GPIO_Port GPIOC
#define E_EndStop3_Sup_EXTI_IRQn EXTI15_10_IRQn
#define E_EndStop2_Sup_Pin GPIO_PIN_11
#define E_EndStop2_Sup_GPIO_Port GPIOC
#define E_EndStop2_Sup_EXTI_IRQn EXTI15_10_IRQn
#define E_EndStop3_Inf_Pin GPIO_PIN_12
#define E_EndStop3_Inf_GPIO_Port GPIOC
#define E_EndStop3_Inf_EXTI_IRQn EXTI15_10_IRQn
#define E_EndStop2_Inf_Pin GPIO_PIN_2
#define E_EndStop2_Inf_GPIO_Port GPIOD
#define E_EndStop2_Inf_EXTI_IRQn EXTI2_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define E_EndStop1_Sup_Pin GPIO_PIN_4
#define E_EndStop1_Sup_GPIO_Port GPIOB
#define E_EndStop1_Sup_EXTI_IRQn EXTI4_IRQn
#define E_EndStop1_Inf_Pin GPIO_PIN_5
#define E_EndStop1_Inf_GPIO_Port GPIOB
#define E_EndStop1_Inf_EXTI_IRQn EXTI9_5_IRQn
#define S_PulsePaP3_Pin GPIO_PIN_6
#define S_PulsePaP3_GPIO_Port GPIOB
#define stepFeedback2_Pin GPIO_PIN_8
#define stepFeedback2_GPIO_Port GPIOB
#define stepFeedback3_Pin GPIO_PIN_9
#define stepFeedback3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
//Macros de la lectura de los finales de carrera
#define ES1s_PRESSED !(HAL_GPIO_ReadPin(E_EndStop1_Sup_GPIO_Port, E_EndStop1_Sup_Pin))
#define ES2s_PRESSED !(HAL_GPIO_ReadPin(E_EndStop2_Sup_GPIO_Port, E_EndStop2_Sup_Pin))
#define ES3s_PRESSED !(HAL_GPIO_ReadPin(E_EndStop3_Sup_GPIO_Port, E_EndStop3_Sup_Pin))

#define ES1s_UNPRESSED (HAL_GPIO_ReadPin(E_EndStop1_Sup_GPIO_Port, E_EndStop1_Sup_Pin))
#define ES2s_UNPRESSED (HAL_GPIO_ReadPin(E_EndStop2_Sup_GPIO_Port, E_EndStop2_Sup_Pin))
#define ES3s_UNPRESSED (HAL_GPIO_ReadPin(E_EndStop3_Sup_GPIO_Port, E_EndStop3_Sup_Pin))


#define ES1i_PRESSED !(HAL_GPIO_ReadPin(E_EndStop1_Inf_GPIO_Port, E_EndStop1_Inf_Pin))
#define ES2i_PRESSED !(HAL_GPIO_ReadPin(E_EndStop2_Inf_GPIO_Port, E_EndStop2_Inf_Pin))
#define ES3i_PRESSED !(HAL_GPIO_ReadPin(E_EndStop3_Inf_GPIO_Port, E_EndStop3_Inf_Pin))

#define ES1i_UNPRESSED (HAL_GPIO_ReadPin(E_EndStop1_Inf_GPIO_Port, E_EndStop1_Inf_Pin))
#define ES2i_UNPRESSED (HAL_GPIO_ReadPin(E_EndStop2_Inf_GPIO_Port, E_EndStop2_Inf_Pin))
#define ES3i_UNPRESSED (HAL_GPIO_ReadPin(E_EndStop3_Inf_GPIO_Port, E_EndStop3_Inf_Pin))

//Macros para encender o apargar el pwm de los motores

#define Start_PWM_MOTOR_1 (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1))
#define Start_PWM_MOTOR_2 (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1))
#define Start_PWM_MOTOR_3 (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1))

#define Stop_PWM_MOTOR_1 (HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1))
#define Stop_PWM_MOTOR_2 (HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1))
#define Stop_PWM_MOTOR_3 (HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1))

//Macros para definir el sentido de giro de los motores.  POSITIVO SENTIDO HORARIO (VISTA FRONTAL DEL MOTOR)
// 														  NEGATIVO SENTIDO ANTIHORARIO (VISTA FRONTAL DEL MOTOR)

#define positive_Dir_MOTOR_1 (HAL_GPIO_WritePin(S_DirPaP1_GPIO_Port, S_DirPaP1_Pin, GPIO_PIN_RESET))
#define positive_Dir_MOTOR_2 (HAL_GPIO_WritePin(S_DirPaP2_GPIO_Port, S_DirPaP2_Pin, GPIO_PIN_RESET))
#define positive_Dir_MOTOR_3 (HAL_GPIO_WritePin(S_DirPaP3_GPIO_Port, S_DirPaP3_Pin, GPIO_PIN_RESET))

#define negative_Dir_MOTOR_1 (HAL_GPIO_WritePin(S_DirPaP1_GPIO_Port, S_DirPaP1_Pin, GPIO_PIN_SET))
#define negative_Dir_MOTOR_2 (HAL_GPIO_WritePin(S_DirPaP2_GPIO_Port, S_DirPaP2_Pin, GPIO_PIN_SET))
#define negative_Dir_MOTOR_3 (HAL_GPIO_WritePin(S_DirPaP3_GPIO_Port, S_DirPaP3_Pin, GPIO_PIN_SET))

#define relayAbierto (HAL_GPIO_WritePin(relayDrivers_GPIO_Port, relayDrivers_Pin, GPIO_PIN_SET))
#define relayCerrado (HAL_GPIO_WritePin(relayDrivers_GPIO_Port, relayDrivers_Pin, GPIO_PIN_RESET))

#define FCL 64000000.0
#define BUFFER_SIZE 30

#define MICROSTEPRESOLUTION 8000.0			   //Micropasos por revolucion. Corresponde a la configuracion de los Swithces del Driver
#define REDUCTOR 1.0				 		   //Relacion de reduccion (Se la esablece a 1 ya que hemos quitado los reductores del robot)
#define STEPREV MICROSTEPRESOLUTION*REDUCTOR   //Paso por revolucion considerando la reduccion.

#define COUNTERPERIOD(rpm) (uint32_t)((FCL/((double)(TIM1->PSC) + 1.0))*( 60.0 / ((rpm) * STEPREV)) - 1.0)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
