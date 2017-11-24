#include "Init_Navigation.h"
#pragma once 

void sinscal_zundamp(double quart_del );
void attUpdate();
void navigation(double o_vel_e, double o_vel_n, double o_ang, char mode);
void sinscal_TRANSVERSE(struct SYS_ELEMENTtransverse &inforS, double quart_del, double gyro[2][3]);