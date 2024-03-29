/*
 * statesMachine.c
 *
 *  Created on: Apr 3, 2023
 *      Author: Elias Correa y Eliseo Elorga
 */

#include "statesMachine.h"



statesMachine state = INIT;

bool homFin = false;

bool startMotors = false;
bool stopMotors = false;

bool upperESalarm = false;
bool lowerESalarm = false;
bool faultDrivers = false;

bool manualTrigger = false;



//--------------------------------------------
//Valores para crear el perfil de velocidad
//'q' es una variable fisica que puede interpretarse como posicon cartesiana o articular. qd,qdd,qddd son sus respectivas derivadas

//Todos los valores estan experesados en el "Sistema Internacional de Unidades" es decir, rad, s.

double q=0,qd=0,qdd=0,qddd=0;

double vmax = 0.75;		//Velocidad maxima
double vmin;			//Velocidad minima

double vi = 0.7;		//Velocidad inicial
double vf = 0.5;			//Velocidad final

double amax = 5;		//Aceleracion maxima
double amin;			//Aceleracion minima

double jmax = 300;        //Jerk maximo
double jmin;			//Jerk minimo
//--------------------------------------------

Vec3D Pini,Pfin;       //Punto inicial y final  (coordenadas cartesianas)
double temp1,temp2,temp3;
double arrayParams1[7];
double arrayParams2[7];
double arrayParams3[7];

double rpm_fault = 1;

bool timeFlag;
uint8_t rx_index = 0;
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t rx_data;

uint8_t message1[]="The robot is ready to be used.\n";
uint8_t message2[]="done\n";

bool receptionFlag=false;


void statesMachineLoop(void){



 	switch (state){

	case INIT:

		HAL_UART_Transmit(&huart2,(uint8_t*)"S1\n", 4, 100);
		HAL_UART_Receive_IT(&huart2, &rx_data, 1);

		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_Base_Start(&htim3);
		HAL_TIM_Base_Start(&htim4);

		//Enable drivers motores (0 es habilitado)
		ENABLE_PIN_RESET_1;
		ENABLE_PIN_RESET_2;
		ENABLE_PIN_RESET_3;

		HAL_Delay(DELAY_ENABLE);

		//Se estable la direccion horario por defecto
		positive_Dir_MOTOR_1;
		positive_Dir_MOTOR_2;
		positive_Dir_MOTOR_3;

		motor1.stepReached = false;
		motor2.stepReached = false;
		motor3.stepReached = false;


		HAL_UART_Transmit(&huart2, message1, sizeof(message1), 100); //Mensaje inidicando que el Robot esta listo para su uso
		HAL_Delay(100);
		HAL_UART_Transmit(&huart2,(uint8_t*)"S2\n", 4, 100);
		state = READY;

		break;

	case HOME:

		HAL_UART_Transmit(&huart2,(uint8_t*)"S3\n", 4, 100);
		receptionFlag = false; //Solo para asegurarse de no saltar al estado ready con esta bandera en true

		homing();

        if(homFin){

        	homFin = false;

        	HAL_NVIC_EnableIRQ(EXTI0_IRQn);		//Enciendo interrupcion EndStop 1 Superior
        	HAL_NVIC_EnableIRQ(EXTI1_IRQn);		//Enciendo interrupcion EndStop 1 Inferior
        	HAL_NVIC_EnableIRQ(EXTI2_IRQn);		//Enciendo interrupcion EndStop 2 Superior
        	HAL_NVIC_EnableIRQ(EXTI3_IRQn);		//Enciendo interrupcion EndStop 2 Inferior
        	HAL_NVIC_EnableIRQ(EXTI4_IRQn);		//Enciendo interrupcion EndStop 3 Superior
        	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);	//Enciendo interrupcion EndStop 3 Inferior
        	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //Enciendo interrupcion faultDriver

        	HAL_Delay(10);

			Pini.x=0;
			Pini.y=0;
			Pini.z =-0.5208; //antes era -0.33

			motor1.theta = 0.0;
			motor2.theta = 0.0;
			motor3.theta = 0.0;

			motor1.currentAngle = 0.0;
			motor2.currentAngle = 0.0;
			motor3.currentAngle = 0.0;

			upperESalarm = false;
			lowerESalarm = false;

			HAL_UART_Transmit(&huart2,(uint8_t*)"S2\n", 4, 100);
			state = READY;

        }

		break;

	case WORKING:

		receptionFlag = false;
		//HAL_UART_Transmit(&huart2,(uint8_t*)"S4\n", 4, 100);

		while (!(motor1.stepReached && motor2.stepReached  && motor3.stepReached)){

			if (state==FAULT)break;

			if (motor1.stepReached) {
				Stop_PWM_MOTOR_1;
				HAL_TIM_IC_Stop(&htim5, TIM_CHANNEL_1);
			}else if (motor2.stepReached) {
				Stop_PWM_MOTOR_2;
				HAL_TIM_IC_Stop(&htim10, TIM_CHANNEL_1);
			}else if (motor3.stepReached){
				Stop_PWM_MOTOR_3;
				HAL_TIM_IC_Stop(&htim11, TIM_CHANNEL_1);
			}

			motor1.omega = get_Straj(time,temp1*DEG_TO_RAD,motor1.theta*DEG_TO_RAD,arrayParams1);
			motor2.omega = get_Straj(time,temp2*DEG_TO_RAD,motor2.theta*DEG_TO_RAD,arrayParams2);
			motor3.omega = get_Straj(time,temp3*DEG_TO_RAD,motor3.theta*DEG_TO_RAD,arrayParams3);

			setProfilTimer();

			if(startMotors){
				startMotors = false;
				Start_PWM_MOTOR_1;	// Activar generacion de pwm
				Start_PWM_MOTOR_2;	// Activar generacion de pwm
				Start_PWM_MOTOR_3;	// Activar generacion de pwm
			}

			stopMotors = true;

		}// End while

		if (stopMotors){   //If steps goals for each motor were reached, we stop motors

			startMotors = false;

			HAL_TIM_IC_Stop(&htim5,  TIM_CHANNEL_1);
			HAL_TIM_IC_Stop(&htim10, TIM_CHANNEL_1);
			HAL_TIM_IC_Stop(&htim11, TIM_CHANNEL_1);

			if (motor1.stepReached) Stop_PWM_MOTOR_1;
			if (motor2.stepReached) Stop_PWM_MOTOR_2;
			if (motor3.stepReached)	Stop_PWM_MOTOR_3;
		}

		//Update coordinantes
		Pini.x = Pfin.x;
		Pini.y = Pfin.y;
		Pini.z = Pfin.z;

		HAL_UART_Transmit(&huart2, message2, sizeof(message2), 100);

		HAL_TIM_Base_Stop_IT(&htim9);
		HAL_TIM_Base_Stop(&htim2);

		state = READY;

		break;

	case READY:

		if (receptionFlag){

			receptionFlag = false;

			startMotors = true;

			HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
			HAL_TIM_IC_Start_IT(&htim10, TIM_CHANNEL_1);
			HAL_TIM_IC_Start_IT(&htim11, TIM_CHANNEL_1);

			inverseKinematic(Pfin);

			update_ScurveTraj(motor1.currentAngle*DEG_TO_RAD, motor1.theta*DEG_TO_RAD, vi, vf, vmax, amax, jmax, arrayParams1);
			update_ScurveTraj(motor2.currentAngle*DEG_TO_RAD, motor2.theta*DEG_TO_RAD, vi, vf, vmax, amax, jmax, arrayParams2);
			update_ScurveTraj(motor3.currentAngle*DEG_TO_RAD, motor3.theta*DEG_TO_RAD, vi, vf, vmax, amax, jmax, arrayParams3);

			temp1=motor1.currentAngle;
			temp2=motor2.currentAngle;
			temp3=motor3.currentAngle;

			configMotor(&motor1,1);
			configMotor(&motor2,2);
			configMotor(&motor3,3);

			timeFlag = false;

			motor1.pMotor = 0;
			motor2.pMotor = 0;
			motor3.pMotor = 0;

			motor1.stepReached = false;
			motor2.stepReached = false;
			motor3.stepReached = false;

			HAL_TIM_Base_Start_IT(&htim9);
			HAL_TIM_Base_Start(&htim2);

			HAL_UART_Transmit(&huart2,(uint8_t*)"S4\n", 4, 100);
			state = WORKING;
		}
		break;

	case FAULT:

		__HAL_TIM_SET_AUTORELOAD(&htim1,COUNTERPERIOD(rpm_fault)); //Escritura del registro ARR
		__HAL_TIM_SET_AUTORELOAD(&htim3,COUNTERPERIOD(rpm_fault));
		__HAL_TIM_SET_AUTORELOAD(&htim4,COUNTERPERIOD(rpm_fault));

		TIM1->CCR1 = (uint32_t)((double)(TIM1->ARR) / 2.0);
		TIM3->CCR1 = (uint32_t)((double)(TIM3->ARR) / 2.0);
		TIM4->CCR1 = (uint32_t)((double)(TIM4->ARR) / 2.0);

		while((upperESalarm || lowerESalarm) && manualTrigger){


			 if (ES1i_PRESSED){
				 HAL_Delay(DELAY_FC_SENSOR);
				 if (ES1i_PRESSED){
					 positive_Dir_MOTOR_1;
					 HAL_Delay(DELAY_DIR); 							//delay cambio de dir
					 Start_PWM_MOTOR_1;
					 HAL_Delay(DELAY_FAULT);
					 Stop_PWM_MOTOR_1;
				 }
			 }
			 if (ES1s_PRESSED){
				 HAL_Delay(DELAY_FC_SENSOR);
				 if (ES1s_PRESSED){
					 negative_Dir_MOTOR_1;
					 HAL_Delay(DELAY_DIR); 							//delay cambio de dir
					 Start_PWM_MOTOR_1;
					 HAL_Delay(DELAY_FAULT);
					 Stop_PWM_MOTOR_1;
				 }
			 }
			 if (ES2i_PRESSED){
				 HAL_Delay(DELAY_FC_SENSOR);
				 if (ES2i_PRESSED){
					 positive_Dir_MOTOR_2;
					 HAL_Delay(DELAY_DIR);
					 Start_PWM_MOTOR_2;
					 HAL_Delay(DELAY_FAULT);
					 Stop_PWM_MOTOR_2;
				 }
			 }
			 if (ES2s_PRESSED){
				 HAL_Delay(DELAY_FC_SENSOR);
				 if (ES2s_PRESSED){
					 negative_Dir_MOTOR_2;
					 HAL_Delay(DELAY_DIR);
					 Start_PWM_MOTOR_2;
					 HAL_Delay(DELAY_FAULT);
					 Stop_PWM_MOTOR_2;
				 }
			 }
			 if (ES3i_PRESSED){
				 HAL_Delay(DELAY_FC_SENSOR);
				 if (ES3i_PRESSED){
					 positive_Dir_MOTOR_3;
					 HAL_Delay(DELAY_DIR);
					 Start_PWM_MOTOR_3;
					 HAL_Delay(DELAY_FAULT);
					 Stop_PWM_MOTOR_3;
				 }
			 }
			 if (ES3s_PRESSED){
				 HAL_Delay(DELAY_FC_SENSOR);
				 if (ES3s_PRESSED){
					 negative_Dir_MOTOR_3;
					 HAL_Delay(DELAY_DIR);
					 Start_PWM_MOTOR_3;
					 HAL_Delay(DELAY_FAULT);
					 Stop_PWM_MOTOR_3;
				 }
			 }


			 if(ES1s_UNPRESSED && ES2s_UNPRESSED && ES3s_UNPRESSED && ES1i_UNPRESSED && ES2i_UNPRESSED && ES3i_UNPRESSED){
				 HAL_Delay(DELAY_FC_SENSOR);
				 if(ES1s_UNPRESSED && ES2s_UNPRESSED && ES3s_UNPRESSED && ES1i_UNPRESSED && ES2i_UNPRESSED && ES3i_UNPRESSED){

					 upperESalarm = false;
					 lowerESalarm = false;
					 manualTrigger = false;
					 HAL_UART_Transmit(&huart2,(uint8_t*)"S2\n", 13, 100);
					 state = READY;
				 }
			 }

		}//End while

		while(faultDrivers && manualTrigger){


			//relayAbierto;
			//HAL_Delay(100);
			//relayCerrado;

			faultDrivers = false;
			manualTrigger = false;

			HAL_UART_Transmit(&huart2,(uint8_t*)"S2\n", 13, 100);
			state = READY;

		}//End while


		break;


	default:break;
	}
}
