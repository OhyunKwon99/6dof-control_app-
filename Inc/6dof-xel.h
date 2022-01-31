#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "round_function.h"

float deg2rad(float degree);  //angle tf

float rad2deg(float radius);  //angle tf

float cosd(float degree);   //deg func

float sind(float degree);   //deg func

float atan2d(float s, float c);   //deg func

void endeff_set(double angle1,double angle2,double angle3);   //endeffector initialize

double half(double num);    // round off func

void seperate(double hex_1,double hex_2, double hex_3,double hex_4);
//seperate by priority bits 

void dec2hex(double dec_1, double dec_2, double dec_3, double dec_4);  //change data form

double angle_calibrate(double angle);  // angle limit func

double angle_calibrate_5(double angle);  // angle limit func

void cal_target_val(float px, float py, float pz);  //inverse kinematics

void value_assign();  // calculated data assign

void Packet_value_set();  //assign data to transmit buffer
