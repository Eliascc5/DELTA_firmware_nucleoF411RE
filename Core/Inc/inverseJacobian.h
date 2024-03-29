/*
 * inverseJacobian.c
 *
 *  Created on: 15 sep. 2022
 *      Author: Elias Correa y Eliseo Elorga
 */

#ifndef INC_INVERSEJACOBIAN_H_
#define INC_INVERSEJACOBIAN_H_

#include <GlobalFunc.h>
#include "main.h"

extern Motor motor1,motor2,motor3;
extern double XJ1_1, YJ1_1, ZJ1_1;
extern double XJ2_1, YJ2_1, ZJ2_1;
extern double XJ3_1, YJ3_1, ZJ3_1;
extern double omega[3];
extern double omega1, omega2, omega3;

void inverseJacobian(double Vxaux, double Vyaux, double Vzaux, double Pxaux, double Pyaux, double Pzaux);

#endif /* INC_INVERSEJACOBIAN_H_ */
