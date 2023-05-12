/*
 * statesMachine.h
 *
 *  Created on: Apr 3, 2023
 *      Author: Elias Correa y Eliseo Elorga
 */

#ifndef INC_STATESMACHINE_H_
#define INC_STATESMACHINE_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "main.h"
#include "trajectory.h"
#include "homing.h"
#include "motor.h"


extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1,htim2,htim3,htim5,htim10,htim11,htim4,htim9;
extern Motor motor1,motor2,motor3;
extern double time;

void statesMachineLoop(void);


#endif /* INC_STATESMACHINE_H_ */
