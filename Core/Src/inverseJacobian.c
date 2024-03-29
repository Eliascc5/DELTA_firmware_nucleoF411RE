/*
 * inverseJacbian.c
 *
 *  Created on: 15 sep. 2022
 *      Author: Elias Correa y Eliseo Elorga
 */


#include "inverseJacobian.h"


double F1[3] = { 0, -0.1680, 0 };
double F2[3] = { 0.145492267835786, 0.084, 0 };
double F3[3] = { -0.145492267835786, 0.084, 0 };

double SA1[3] = { 1, 0, 0 };
double SA2[3] = { -0.5, 0.866025403784439, 0 };
double SA3[3] = { -0.5, -0.866025403784439, 0 };

double E1[3];
double E2[3];
double E3[3];

double l1[3][3];
double l2[3][3];


double Jcd[3][3], Jci[3][3], Jci_inv[3][3], Jinv[3][3];

double cross_P1[3];
double cross_P2[3];
double cross_P3[3];
double dot[3];


void inverseJacobian(double Vxaux, double Vyaux, double Vzaux, double Pxaux, double Pyaux, double Pzaux) {
	/*   -----------------------------------------------------------------------
	 * 	 Funcion que devuelve la velocidad angular de cada motor
	 *	 Entrada: Velocidad en (x,y,z) y Posicion en (x,y,z)
	 *	 Salida:  Velocidad angular en rad/s
	 	 -----------------------------------------------------------------------*/

	double Vaux[3] = { Vxaux, Vyaux, Vzaux };

	E1[0] = Pxaux + 0;
	E1[1] = Pyaux - 0.07;
	E1[2] = Pzaux;

	E2[0] = Pxaux + 0.060621778264911;
	E2[1] = Pyaux + 0.035;
	E2[2] = Pzaux;

	E3[0] = Pxaux - 0.060621778264911;
	E3[1] = Pyaux + 0.035;
	E3[2] = Pzaux;

	l1[0][0] = XJ1_1 - F1[0];//l11=[(XJ1(1)-F1(1,1));(YJ1(1)-F1(1,2));(ZJ1(1)-F1(1,3))];
	l1[0][1] = YJ1_1 - F1[1];
	l1[0][2] = ZJ1_1 - F1[2];
	l1[1][0] = XJ2_1 - F2[0];//l12=[(XJ2(1)-F2(1,1));(YJ2(1)-F2(1,2));(ZJ2(1)-F2(1,3))];
	l1[1][1] = YJ2_1 - F2[1];
	l1[1][2] = ZJ2_1 - F2[2];
	l1[2][0] = XJ3_1 - F3[0];//l13=[(XJ3(1)-F3(1,1));(YJ3(1)-F3(1,2));(ZJ3(1)-F3(1,3))];
	l1[2][1] = YJ3_1 - F3[1];
	l1[2][2] = ZJ3_1 - F3[2];

	l2[0][0] = E1[0] - XJ1_1;//l21=[(E1(1,1)-XJ1(1));(E1(1,2)-YJ1(1));(E1(1,3)-ZJ1(1))];
	l2[1][0] = E1[1] - YJ1_1;
	l2[2][0] = E1[2] - ZJ1_1;
	l2[0][1] = E2[0] - XJ2_1;//l22=[(E2(1,1)-XJ2(1));(E2(1,2)-YJ2(1));(E2(1,3)-ZJ2(1))];
	l2[1][1] = E2[1] - YJ2_1;
	l2[2][1] = E2[2] - ZJ2_1;
	l2[0][2] = E3[0] - XJ3_1;//l23=[(E3(1,1)-XJ3(1));(E3(1,2)-YJ3(1));(E3(1,3)-ZJ3(1))];
	l2[1][2] = E3[1] - YJ3_1;
	l2[2][2] = E3[2] - ZJ3_1;

	matrixTranspose(l2, Jcd);//Jacobiano de la cinematica directa


	cross_P1[0] = SA1[1] * l1[0][2] - SA1[2] * l1[0][1];//Producto Cruz entre vectores
	cross_P1[1] = SA1[2] * l1[0][0] - SA1[0] * l1[0][2];
	cross_P1[2] = SA1[0] * l1[0][1] - SA1[1] * l1[0][0];

	cross_P2[0] = SA2[1] * l1[1][2] - SA2[2] * l1[1][1];//vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1]
	cross_P2[1] = SA2[2] * l1[1][0] - SA2[0] * l1[1][2];//vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2]
	cross_P2[2] = SA2[0] * l1[1][1] - SA2[1] * l1[1][0];//vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]

	cross_P3[0] = SA3[1] * l1[2][2] - SA3[2] * l1[2][1];
	cross_P3[1] = SA3[2] * l1[2][0] - SA3[0] * l1[2][2];
	cross_P3[2] = SA3[0] * l1[2][1] - SA3[1] * l1[2][0];

	dot[0] = dotProduct(cross_P1, Jcd[0],3);			//Producto Punto
	dot[1] = dotProduct(cross_P2, Jcd[1],3);
	dot[2] = dotProduct(cross_P3, Jcd[2],3);

	for (int i = 0; i < 3; ++i) {			//Jacobiano de la cinematica inversa
		for (int j = 0; j < 3; ++j) {
			if (i == j) {
				Jci[i][j] = dot[i];
			} else {
				Jci[i][j] = 0;
			}
		}
	}

	inv(Jci,Jci_inv);
	matrixProduct(Jcd,Jci_inv,Jinv); //Jacobiano Inverso


	for (int i = 0; i < 3; ++i) {
		double sum = 0;
		for (int j = 0; j < 3; ++j) {
			sum += Jinv[i][j] * Vaux[j];
		}
		omega[i] = sum;
	}
	motor1.omega = omega[0];
	motor2.omega = omega[1];
	motor3.omega = omega[2];

}
