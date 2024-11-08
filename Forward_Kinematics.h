/*
 * Forward_Kinematics.h
 *
 *  Created on: Nov 6, 2024
 *      Author: Knnn
 */

#ifndef FORWARD_KINEMATICS_H_
#define FORWARD_KINEMATICS_H_

#include "stdint.h"
#include "stdlib.h"

#include "math.h"
#include "stm32G0xx_hal.h"

//https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
//forward kinematic equations mecanum wheel

extern float _Lx;
extern float _Ly;
extern float _Radius_wheel;

void Setup_Forward_Kinematic(float Lx ,float Ly ,float Radius_wheel);

void Odometry_Forward_Kinematic(float w_LF, float w_LB, float w_RF, float w_RB);

float get_Vx();
float get_Vy();
float get_Vz();

void Position_Robot();

float get_Sx();
float get_Sy();
float get_Sz();
#endif /* FORWARD_KINEMATICS_H_ */
