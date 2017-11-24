#include "MatCal.h"
#pragma once 

void rotx(double c[3][3], double x);          /* 绕x轴旋转n->b */
void roty(double c[3][3], double x);          /* 绕y轴旋转n->b */
void rotz(double c[3][3], double x);          /* 绕z轴旋转n->b */
void ang2cnb(double c[3][3], double angle[3]);/* 姿态角计算旋转矩阵n->b */
void qcal(double deltasita[3], double quart[4]); /* 比卡算法更新四元数 */
void cnb2q(double c[3][3], double q[4]);      /* 姿态矩阵到四元数 */
void q2cnb(double c[3][3], double q[4]);      /* 四元数算姿态矩阵 */
void optq(double q[4]);                       /* 四元数归一化 */
void cnb2ang(double cnb[3][3], double angle[3]); /* 姿态矩阵算姿态角 */
void ang2q(double q[4], double angle[3]);
double latitog(double lati);                  /* 根据输入纬度值计算重力加速度值 */
void X2cnn(double c[3][3], double X_vector[3]);
void set_R(FILTER &kal, double v1, double v2, double ang);
void DeltaAtt2Phi(SYS_ELEMENT temp_infor,double phi[3],double delta_att[3]);//姿态误差角到失准角的处理




