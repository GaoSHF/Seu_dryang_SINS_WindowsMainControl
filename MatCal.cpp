/* Includes ------------------------------------------------------------------*/
#include "stdafx.h"
#include "MatCal.h"

/**
* @brief  vecmul 矩阵乘向量运算 a(m,1) = w(m,n) * b(n,1)
* @param  m 整型 表示矩阵行数m
*         n 整型 表示矩阵列数n
*         a 浮点指针 表示返回向量(m,1)
*         w 浮点指针 表示输入矩阵(m,n)
*         b 浮点指针 表示输入向量(n,1)
* @retval None
*/
void vecmul(int m, int n, double *a, double *w, double *b)
{
	double aa[30];
	int i, j;

	for (i = 0; i<m; i++)
	{
		aa[i] = 0.0;
		for (j = 0; j<n; j++)
			aa[i] += (*(w + i*n + j))*(*(b + j));
	}

	for (i = 0; i<m; i++)
		*(a + i) = aa[i];
}

/**
* @brief  vecadd 向量相加 a(m,1) = b(m,1) + c(m,1)
* @param  m 整型 表示向量维数
*         a 浮点指针 表示返回向量(m,1)
*         b 浮点指针 表示输入向量(m,1)
*         c 浮点指针 表示输入向量(m,1)
* @retval None
*/
void vecadd(int m, double *a, double *b, double *c)
{
	int i;

	for (i = 0; i<m; i++)
		*(a + i) = *(b + i) + *(c + i);
}

/**
* @brief  vecsub 矢量相减 a(m,1) = b(m,1) - c(m,1)
* @param  m 整型 表示向量维数
*         a 浮点指针 表示返回向量(m,1)
*         b 浮点指针 表示输入向量(m,1)
*         c 浮点指针 表示输入向量(m,1)
* @retval None
*/
void vecsub(int m, double *a, double *b, double *c)
{
	int i;

	for (i = 0; i<m; i++)
		*(a + i) = *(b + i) - *(c + i);
}

/**
* @brief  crossm 矢量变叉乘矩阵
[0     -b(2)  b(1)]
a = [b(2)   0    -b(0)]
[-b(1)  b(0)  0   ]
* @param  a 浮点指针 表示返回矩阵(3,3)
*         b 浮点指针 表示输入向量(3,1)
* @retval None
*/
void crossm(double *a, double *b)
{
	*(a) = 0.0l;                                        /* 为何不直接给0 */
	*(a + 1) = -*(b + 2);
	*(a + 2) = *(b + 1);
	*(a + 3) = *(b + 2);
	*(a + 4) = 0.0l;
	*(a + 5) = -*b;
	*(a + 6) = -*(b + 1);
	*(a + 7) = *b;
	*(a + 8) = 0.0l;
}

/**
* @brief  cvecmul 矢量叉乘运算 a(3,1) = b(3,1) x c(3,1)
* @param  a 浮点指针 表示返回向量(3,1)
*         b 浮点指针 表示输入矩阵(3,1)
*         c 浮点指针 表示输入向量(3,1)
* @retval None
*/
void cvecmul(double *a, double *b, double *c)
{
	double bb[3][3];
	crossm((double *)bb, b);
	vecmul(3, 3, a, (double *)bb, c);
}

/**
* @brief  mamul 矩阵相乘运算 a(m,n) = b(m,p) * c(p,n)
* @param  n1 整型 表示矩阵维数m
*         n2 整型 表示矩阵维数p
*         n3 整型 表示矩阵维数n
*         a 浮点指针 表示返回向量(m,n)
*         b 浮点指针 表示输入矩阵(m,p)
*         c 浮点指针 表示输入向量(p,n)
* @retval None
*/
void mamul(int n1, int n2, int n3, double *a, double *b, double *c)
{
	double d[30][30];
	int i, j, k;

	for (i = 0; i<n1; i++)
		for (j = 0; j<n3; j++)
		{
			d[i][j] = 0.0;
			for (k = 0; k<n2; k++)
				d[i][j] += *(b + i*n2 + k)*(*(c + k*n3 + j));
		}

	for (i = 0; i<n1; i++)
		for (j = 0; j<n3; j++)
			*(a + i*n3 + j) = d[i][j];
}

/**
* @brief  maadd 矩阵相加运算 a(m,n) = b(m,n) + c(m,n)
* @param  n1 整型 表示矩阵行数m
*         n2 整型 表示矩阵列数n
*         a 浮点指针 表示返回向量(m,n)
*         b 浮点指针 表示输入矩阵(m,n)
*         c 浮点指针 表示输入向量(m,n)
* @retval None
*/
void maadd(int n1, int n2, double *a, double *b, double *c)
{
	int i, j;

	for (i = 0; i<n1; i++)
		for (j = 0; j<n2; j++)
			*(a + i*n2 + j) = *(b + i*n2 + j) + *(c + i*n2 + j);
}

/**
* @brief  masub 矩阵相减运算 a(m,n) = b(m,n) - c(m,n)
* @param  n1 整型 表示矩阵行数m
*         n2 整型 表示矩阵列数n
*         a 浮点指针 表示返回向量(m,n)
*         b 浮点指针 表示输入矩阵(m,n)
*         c 浮点指针 表示输入向量(m,n)
* @retval None
*/
void masub(int n1, int n2, double *a, double *b, double *c)
{
	int i, j;

	for (i = 0; i<n1; i++)
		for (j = 0; j<n2; j++)
			*(a + i*n2 + j) = *(b + i*n2 + j) - *(c + i*n2 + j);
}

/**
* @brief  avecmul 矢量数乘运算 a(m,1) = b(m,1) .x c(1,1)
* @param  n 整型 表示矩阵行数m
*         a 浮点指针 表示返回向量(m,1)
*         b 浮点指针 表示输入矩阵(m,1)
*         c 浮点指针 表示输入向量(1,1)
* @retval None
*/
void avecmul(int n, double *a, double *b, double c)
{
	int i;

	for (i = 0; i<n; i++)
		*(a + i) = *(b + i)*c;
}

/**
* @brief  amamul 矩阵数乘运算 a(m,n) = b(m,n) .x c(1,1)
* @param  n1 整型 表示矩阵行数m
*         n2 整型 表示矩阵列数n
*         a 浮点指针 表示返回向量(m,n)
*         b 浮点指针 表示输入矩阵(m,n)
*         c 浮点指针 表示输入向量(1,1)
* @retval None
*/
void amamul(int n1, int n2, double *a, double *b, double c)
{
	int i, j;

	for (i = 0; i<n1; i++)
		for (j = 0; j<n2; j++)
			*(a + i*n2 + j) = *(b + i*n2 + j)*c;
}

/**
* @brief  矩阵转置运算 c(n,m) = d(m,n)'
* @param  n1 整型 表示矩阵行数m
*         n2 整型 表示矩阵列数n
*         c 浮点指针 表示返回向量(n,m)
*         d 浮点指针 表示输入矩阵(m*n)
* @retval None
*/
void maturn(int n1, int n2, double *c, double *d)
{
	double a[30][30];
	int i, j;

	for (i = 0; i<n1; i++)
		for (j = 0; j<n2; j++)
			a[j][i] = *(d + j + i*n2);

	for (i = 0; i<n2; i++)
		for (j = 0; j<n1; j++)
			*(c + i*n1 + j) = a[i][j];
}

/**
* @brief  swap 两个数交换运算 a <-> b
* @param  a 浮点指针 表示返回向量(1,1)
*         b 浮点指针 表示输入向量(1,1)
* @retval None
*/
void swap(double *a, double *b)
{
	double c;

	c = *a;
	*a = *b;
	*b = c;
}

/**
* @brief  mainv 矩阵求逆运算 a = inv(a)
* @param  n 整型 表示矩阵维数m
*         a 浮点指针 表示返回向量(m*m)
* @retval None
*/
void mainv(int n, double *a)
{
	int		i, j;
	int		is[30], js[30];
	int		f = 1, k;
	double	fDet = 1.0;

	for (k = 0; k < n; k++)
	{
		/* 第一步，全选主元 */
		double fMax = 0.0;
		for (i = k; i < n; i++)
		{
			for (j = k; j < n; j++)
			{
				const double fm = fabs(*(a + i*n + j));
				if (fm > fMax)
				{
					fMax = fm;
					is[k] = i;
					js[k] = j;
				}
			}
		}

		if (is[k] != k)
		{
			f = -f;
			for (i = 0; i < n; i++)
				swap((a + k*n + i), (a + is[k] * n + i));
		}
		if (js[k] != k)
		{
			f = -f;
			for (i = 0; i < n; i++)
				swap((a + i*n + k), (a + i*n + js[k]));
		}

		/* 计算行列值 */
		fDet *= *(a + k*n + k);

		/* 计算逆矩阵 */

		/* 第二步 */
		*(a + k*n + k) = 1.0 / *(a + k*n + k);

		/* 第三步 */
		for (j = 0; j < n; j++)
		{
			if (j != k)
				*(a + k*n + j) *= *(a + k*n + k);
		}
		/* 第四步 */
		for (i = 0; i < n; i++)
		{
			if (i != k)
			{
				for (j = 0; j < n; j++)
				{
					if (j != k)
						*(a + i*n + j) = *(a + i*n + j) - *(a + i*n + k) * (*(a + k*n + j));
				}
			}
		}
		/* 第五步 */
		for (i = 0; i < n; i++)
		{
			if (i != k)
				*(a + i*n + k) *= -*(a + k*n + k);
		}
	}

	for (k = n - 1; k >= 0; k--)
	{
		if (js[k] != k)
		{
			for (i = 0; i < n; i++)
				swap((a + k*n + i), (a + js[k] * n + i));
		}
		if (is[k] != k)
		{
			for (i = 0; i < n; i++)
				swap((a + i*n + k), (a + i*n + is[k]));
		}
	}
	fDet *= f;
}

/**
* @brief  qmulv 四元数乘法
* @param  vout
*         q
*         vin
* @retval None
*/
void qmulv(double vout[3], double q[4], double vin[3])
{
	double qm[3][3] = { 0. };

	qm[0][0] = q[1] * q[1] + q[0] * q[0] - q[3] * q[3] - q[2] * q[2];
	qm[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
	qm[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);

	qm[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
	qm[1][1] = q[2] * q[2] - q[3] * q[3] + q[0] * q[0] - q[1] * q[1];
	qm[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);

	qm[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
	qm[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
	qm[2][2] = q[3] * q[3] - q[2] * q[2] - q[1] * q[1] + q[0] * q[0];

	vecmul(3, 3, vout, (double *)qm, vin);
}  // end of function qmulv

   //==== 构造Alpha矩阵 ====//
void alphaMat(double aMat[4][4], double alpha[3])
{
	aMat[0][0] = 0.;
	aMat[0][1] = -alpha[0];
	aMat[0][2] = -alpha[1];
	aMat[0][3] = -alpha[2];

	aMat[1][0] = alpha[0];
	aMat[1][1] = 0.;
	aMat[1][2] = alpha[2];
	aMat[1][3] = -alpha[1];

	aMat[2][0] = alpha[1];
	aMat[2][1] = -alpha[2];
	aMat[2][2] = 0.;
	aMat[2][3] = alpha[0];

	aMat[3][0] = alpha[2];
	aMat[3][1] = alpha[1];
	aMat[3][2] = -alpha[0];
	aMat[3][3] = 0.;
}  // end of function alphaMat

   //==== 构造Beta矩阵 ====//----2014.12.04
void betaMat(double bMat[4][4], double beta[3])
{
	bMat[0][0] = 0.;
	bMat[0][1] = -beta[0];
	bMat[0][2] = -beta[1];
	bMat[0][3] = -beta[2];

	bMat[1][0] = beta[0];
	bMat[1][1] = 0.;
	bMat[1][2] = -beta[2];
	bMat[1][3] = beta[1];

	bMat[2][0] = beta[1];
	bMat[2][1] = beta[2];
	bMat[2][2] = 0.;
	bMat[2][3] = -beta[0];

	bMat[3][0] = beta[2];
	bMat[3][1] = -beta[1];
	bMat[3][2] = beta[0];
	bMat[3][3] = 0.;
}  // end of function betaMat


void qmul(double q[4], double q1[4], double q2[4])
{
	q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	q[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
	q[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];
}  // end of function qmul

/**
* @brief  sign符号函数
* @param  None
* @retval None
*/
int sign(double x)
{
	int i = 1;
	i = (x>0) ? 1 : -1;
	return i;
}
/**
* @brief  fine_fal
* @param  None
* @retval None
*/
double fal(double x, float a, float delta)
{
	double temp = 0;
	temp = (x <= delta) ? (x / pow(delta, 1 - a)) : (sign(x)*pow(abs(x), double(a)));
	return temp;
}
/**
* @brief  fine_fhan
* @param  None
* @retval None
*/
void fhan(void)
{
}
//向量求模 num为维数
double vectormo(double *a, int num)
{
	int i = 0;
	double temp = 0;
	for (i = 0; i < num; i++)
	{
		temp += (*(a + i))*(*(a + i));
	}
	temp = sqrt(temp);
	return temp;
}
void bubblesort(double *a, int num)
{
	int i, j;
	bool donothing = 0;
	int swaptime = 0;
	for (i = 0; i < num; i++)
	{
		donothing = 1;
		for (j = 0; j < num - i - 1; j++)
		{
			if (*(a + j) > *(a + j + 1))
			{
				swap(a + j, a + j + 1);
				donothing = 0;
				//	*(b + j)= *(b + i)+1;
			}
		}
		if (donothing == 1) break;
	}
}

double white()
{
	static double	s0 = 65536.0;
	static double	w0 = 2053.0;
	static double	v0 = 13849.0;
	static double	r0 = 0.0;
	double			t0, EGx;
	int				m0, i;

	t0 = 0.0;

	for (i = 0; i < 12; i++)
	{
		r0 = w0 * r0 + v0;
		m0 = (int)(r0 / s0);
		r0 = r0 - m0 * s0;
		t0 = t0 + r0 / s0;
	}
	EGx = t0 - 6.0;
	return(EGx);
}

/* End ----------------------------------------------------------------------*/

