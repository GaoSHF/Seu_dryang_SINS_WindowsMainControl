#include "MatCal.h"
#pragma once 

void rotx(double c[3][3], double x);          /* ��x����תn->b */
void roty(double c[3][3], double x);          /* ��y����תn->b */
void rotz(double c[3][3], double x);          /* ��z����תn->b */
void ang2cnb(double c[3][3], double angle[3]);/* ��̬�Ǽ�����ת����n->b */
void qcal(double deltasita[3], double quart[4]); /* �ȿ��㷨������Ԫ�� */
void cnb2q(double c[3][3], double q[4]);      /* ��̬������Ԫ�� */
void q2cnb(double c[3][3], double q[4]);      /* ��Ԫ������̬���� */
void optq(double q[4]);                       /* ��Ԫ����һ�� */
void cnb2ang(double cnb[3][3], double angle[3]); /* ��̬��������̬�� */
void ang2q(double q[4], double angle[3]);
double latitog(double lati);                  /* ��������γ��ֵ�����������ٶ�ֵ */
void X2cnn(double c[3][3], double X_vector[3]);
void set_R(FILTER &kal, double v1, double v2, double ang);
void DeltaAtt2Phi(SYS_ELEMENT temp_infor,double phi[3],double delta_att[3]);//��̬���ǵ�ʧ׼�ǵĴ���




