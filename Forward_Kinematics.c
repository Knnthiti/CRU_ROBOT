/*
 * Forward_Kinematics.c
 *
 *  Created on: Nov 6, 2024
 *      Author: Knnn
 */
#include "Forward_Kinematics.h"

float _Lx;
float _Ly;
float _Radius_wheel;

void Setup_Forward_Kinematic(float Lx ,float Ly ,float Radius_wheel){
	_Lx = Lx;
	_Ly = Ly;
	_Radius_wheel = Radius_wheel;
}

float _Vx = 0;
float _Vy = 0;
float _Vz = 0;

void Odometry_Forward_Kinematic(float w_LF, float w_LB, float w_RF, float w_RB){
	//w_LF w_LB w_RF and w_RB represent the angular velocities.

	_Vy = (w_LF + w_RF + w_LB + w_RB) * (_Radius_wheel / 4.0f);
	_Vx = ((-w_LF + w_RF + w_LB - w_RB) * (_Radius_wheel / 4.0f)) * (-1.0f);
	_Vz = (-w_LF + w_RF - w_LB + w_RB) * (_Radius_wheel / (4.0f * (_Lx + _Ly)));
}

float get_Vx(){
	return _Vx;
}

float get_Vy(){
	return _Vy;
}

float get_Vz(){
	return _Vz;
}

long _Time_Present = 0;
long _Time_Past = 0;
float _Time;

float _Pos_X = 0.0f;
float _Pos_Y = 0.0f;
float _Pos_Z = 0.0f;

void Position_Robot(){
	//Heading_Robot is rad/s
	_Time_Present = HAL_GetTick();
	_Time = (_Time_Present - _Time_Past) / 1000.0f;

	_Pos_Z += _Vz * (float)_Time;
	_Pos_X += ((_Vx * cos(_Pos_Z)) - (_Vy * sin(_Pos_Z))) * _Time;
	_Pos_Y += ((_Vx * sin(_Pos_Z)) + (_Vy * cos(_Pos_Z))) * _Time;

	_Time_Past = _Time_Present;
}

float get_Sx(){
	return _Pos_X;
}

float get_Sy(){
	return _Pos_Y;
}

float get_Sz(){
	return _Pos_Z;
}
