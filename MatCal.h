#include "math.h"
#include "Struct.h"
#pragma once

void vecmul(int m, int n, double *a, double *w, double *b);// 矩阵乘向量运算 a(m,1) = w(m,n) * b(n,1)
void vecadd(int m, double *a, double *b, double *c); //vecadd 向量相加 a(m, 1) = b(m, 1) + c(m, 1)
void vecsub(int m, double *a, double *b, double *c); // vecsub 矢量相减 a(m, 1) = b(m, 1) - c(m, 1)
void crossm(double *a, double *b);// crossm 矢量变叉乘矩阵[0 -  b(2)  b(1)]
									//			      a = [b(2)   0 - b(0)]
									//				      [-b(1)  b(0)  0 ]
void cvecmul(double *a, double *b, double *c);// cvecmul 矢量叉乘运算 a(3,1) = b(3,1) x c(3,1)
void mamul(int n1, int n2, int n3, double *a, double *b, double *c); //矩阵相乘运算 a(m, n) = b(m, p) * c(p, n)
void maadd(int n1, int n2, double *a, double *b, double *c);//maadd 矩阵相加运算 a(m,n) = b(m,n) + c(m,n)
void masub(int n1, int n2, double *a, double *b, double *c);//masub 矩阵相减运算 a(m,n) = b(m,n) - c(m,n)
void avecmul(int n, double *a, double *b, double c);//矢量数乘运算 a(m,1) = b(m,1) .x c(1,1)
void amamul(int n1, int n2, double *a, double *b, double c);// amamul 矩阵数乘运算 a(m,n) = b(m,n) .x c(1,1)
void maturn(int n1, int n2, double *c, double *d);//矩阵转置运算 c(n,m) = d(m,n)'
void swap(double *a, double *b);//交换两个数
void mainv(int n, double *a);//mainv 矩阵求逆运算 a = inv(a)
void qmulv(double vout[3], double q[4], double vin[3]);//
void qmul(double q[4], double q1[4], double q2[4]);//
void alphaMat(double aMat[4][4], double alpha[3]);
void betaMat(double bMat[4][4], double beta[3]);

int sign(double x);//符号函数
double fal(double x, float a, float delta);
void fhan(void);
double vectormo(double *a, int num);//向量求模
void bubblesort(double *a, int num);//排序by dr.yang，应该问题不大
double white();//产生白噪声




