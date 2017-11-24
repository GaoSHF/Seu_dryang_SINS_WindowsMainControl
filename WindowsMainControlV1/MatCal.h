#include "math.h"
#include "Struct.h"
#pragma once

void vecmul(int m, int n, double *a, double *w, double *b);// ������������� a(m,1) = w(m,n) * b(n,1)
void vecadd(int m, double *a, double *b, double *c); //vecadd ������� a(m, 1) = b(m, 1) + c(m, 1)
void vecsub(int m, double *a, double *b, double *c); // vecsub ʸ����� a(m, 1) = b(m, 1) - c(m, 1)
void crossm(double *a, double *b);// crossm ʸ�����˾���[0 -  b(2)  b(1)]
									//			      a = [b(2)   0 - b(0)]
									//				      [-b(1)  b(0)  0 ]
void cvecmul(double *a, double *b, double *c);// cvecmul ʸ��������� a(3,1) = b(3,1) x c(3,1)
void mamul(int n1, int n2, int n3, double *a, double *b, double *c); //����������� a(m, n) = b(m, p) * c(p, n)
void maadd(int n1, int n2, double *a, double *b, double *c);//maadd ����������� a(m,n) = b(m,n) + c(m,n)
void masub(int n1, int n2, double *a, double *b, double *c);//masub ����������� a(m,n) = b(m,n) - c(m,n)
void avecmul(int n, double *a, double *b, double c);//ʸ���������� a(m,1) = b(m,1) .x c(1,1)
void amamul(int n1, int n2, double *a, double *b, double c);// amamul ������������ a(m,n) = b(m,n) .x c(1,1)
void maturn(int n1, int n2, double *c, double *d);//����ת������ c(n,m) = d(m,n)'
void swap(double *a, double *b);//����������
void mainv(int n, double *a);//mainv ������������ a = inv(a)
void qmulv(double vout[3], double q[4], double vin[3]);//
void qmul(double q[4], double q1[4], double q2[4]);//
void alphaMat(double aMat[4][4], double alpha[3]);
void betaMat(double bMat[4][4], double beta[3]);

int sign(double x);//���ź���
double fal(double x, float a, float delta);
void fhan(void);
double vectormo(double *a, int num);//������ģ
void bubblesort(double *a, int num);//����by dr.yang��Ӧ�����ⲻ��
double white();//����������




