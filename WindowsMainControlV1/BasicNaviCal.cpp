#include "stdafx.h"
#include "BasicNaviCal.h"

/**
* @brief  rotx 绕x轴旋转DCM
* @param  c 浮点数组 返回旋转矩阵
*         x 浮点型 绕x轴旋转角度(rad)
* @retval None
*/
void rotx(double c[3][3], double x)
{
	c[0][0] = 1.;
	c[0][1] = 0.;
	c[0][2] = 0.;
	c[1][0] = 0.;
	c[1][1] = cos(x);
	c[1][2] = sin(x);
	c[2][0] = 0.;
	c[2][1] = -sin(x);
	c[2][2] = cos(x);
}

/**
* @brief  roty 绕y轴旋转DCM
* @param  c 浮点数组 返回旋转矩阵
*         x 浮点型 绕y轴旋转角度(rad)
* @retval None
*/
void roty(double c[3][3], double x)
{

	c[0][0] = cos(x);
	c[0][1] = 0.;
	c[0][2] = -sin(x);
	c[1][0] = 0.;
	c[1][1] = 1.;
	c[1][2] = 0.;
	c[2][0] = sin(x);
	c[2][1] = 0.;
	c[2][2] = cos(x);
}

/**
* @brief  rotz 绕x轴旋转DCM
* @param  c 浮点数组 返回旋转矩阵
*         x 浮点型 绕z轴旋转角度(rad)
* @retval None
*/
void rotz(double c[3][3], double x)
{

	c[0][0] = cos(x);
	c[0][1] = sin(x);
	c[0][2] = 0.;
	c[1][0] = -sin(x);
	c[1][1] = cos(x);
	c[1][2] = 0.;
	c[2][0] = 0.;
	c[2][1] = 0.;
	c[2][2] = 1.;
}

/**
* @brief  ang2cnb 由旋转角得到DCM rotating order:z->x->y 按照船用惯导安装
*	      z:yaw   x:pitch   y:roll
* @param  c 返回旋转矩阵
*         angle[2] 浮点型 绕z轴旋转角度(rad)
*         angle[1] 浮点型 绕x轴旋转角度(rad)
*         angle[0] 浮点型 绕y轴旋转角度(rad)
* @retval None
*         由于捷联惯导与载体固连，所测量的惯导数据为载体坐标系下，
*         此处旋转为导航坐标系到载体坐标系旋转C(n->b)，故在实际计
*         算中还需进行转置运算。
*/
void ang2cnb(double c[3][3], double angle[3])
{
	double cc[3][3];

	/* 由绕坐标轴z旋转的角度算出姿态矩阵 */
	rotz(c, angle[2]);

	/* 由绕坐标轴x旋转的角度算出姿态矩阵 */
	rotx(cc, angle[0]);
	mamul(3, 3, 3, (double *)c, (double *)cc, (double *)c);

	/* 由绕坐标轴y旋转的角度算出姿态矩阵*/
	roty(cc, angle[1]);
	mamul(3, 3, 3, (double *)c, (double *)cc, (double *)c);
}

/**
* @brief  qcal 比卡算法更新四元数 q(k+1) = {cos( 0.5*|w|*dt )I(4) + sin( 0.5*|w|*dt )*O(w)} * q(k)
*	      O(w) = [ 0  -wx -wy -wz;
*                  wx  0   wz -wy;
*                  wy -wz  0   wx;
*                  wz  wy -wx  0];
* @param  quart 浮点型 返回更新四元数
*         deltasita[2] 浮点型 陀螺z轴计算增量角(rad)
*         deltasita[1] 浮点型 陀螺y轴计算增量角(rad)
*         deltasita[0] 浮点型 陀螺x轴计算增量角(rad)
* @retval None
*/
void qcal(double deltasita[3], double quart[4])
{
	int		i;
	double	deltasita0, angsin, angcos;
	double	qtrans[4][4];

	deltasita0 = 0;

	for (i = 0; i < 3; i++)
	{
		deltasita0 += deltasita[i] * deltasita[i];
	}
	deltasita0 = sqrt(deltasita0);

	angcos = cos(deltasita0 / 2);
	if (deltasita0 == 0)
		angsin = 0.5;
	else
		angsin = sin(deltasita0 / 2.0) / deltasita0;

	qtrans[0][0] = qtrans[1][1] = qtrans[2][2] = qtrans[3][3] = angcos;
	qtrans[0][1] = qtrans[3][2] = -(angsin * deltasita[0]);
	qtrans[1][0] = qtrans[2][3] = -qtrans[0][1];
	qtrans[0][2] = qtrans[1][3] = -(angsin * deltasita[1]);
	qtrans[2][0] = qtrans[3][1] = -qtrans[0][2];
	qtrans[0][3] = qtrans[2][1] = -(angsin * deltasita[2]);
	qtrans[3][0] = qtrans[1][2] = -qtrans[0][3];

	vecmul(4, 4, quart, (double*)qtrans, quart);
}

/**
* @brief  cnb2q 姿态矩阵算四元数  c(n->b) to q
* @param  q 浮点型 返回四元数
*         c 浮点型 姿态矩阵
* @retval None
*/
void cnb2q(double c[3][3], double q[4])
{
	q[0] = sqrt(fabs(1. + c[0][0] + c[1][1] + c[2][2])) / 2.; /* 由姿态矩阵算出四元素1 */
	q[1] = sqrt(fabs(1. + c[0][0] - c[1][1] - c[2][2])) / 2.; /* 由姿态矩阵算出四元素2 */
	q[2] = sqrt(fabs(1. - c[0][0] + c[1][1] - c[2][2])) / 2.; /* 由姿态矩阵算出四元素3 */
	q[3] = sqrt(fabs(1. - c[0][0] - c[1][1] + c[2][2])) / 2.; /* 由姿态矩阵算出四元素4 */
	if (c[2][1]>c[1][2]) q[1] = -q[1];                  /* 判断四元素2的符号 */
	if (c[0][2]>c[2][0]) q[2] = -q[2];                  /* 判断四元素3的符号 */
	if (c[1][0]>c[0][1]) q[3] = -q[3];                  /* 判断四元素4的符号 */
}

/*******n系到n'系的转换矩阵*******/
//根据状态变量的位置做更改
void X2cnn(double c[3][3], double X_vector[3])
{
	c[0][0] = 1;
	c[0][1] = X_vector[2];
	c[0][2] = -X_vector[1];
	c[1][0] = -X_vector[2];
	c[1][1] = 1;
	c[1][2] = X_vector[0];
	c[2][0] = X_vector[1];
	c[2][1] = -X_vector[0];
	c[2][2] = 1;
	return;
}

/**
* @brief  q2cnb 四元数算姿态矩阵 q to c(n->b)
* @param  c 浮点型 返回姿态矩阵
*         q 浮点型 四元数
* @retval None
*/
void q2cnb(double c[3][3], double q[4])
{
	c[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
	c[0][1] = 2.*(q[1] * q[2] + q[0] * q[3]);
	c[0][2] = 2.*(q[1] * q[3] - q[0] * q[2]);
	c[1][0] = 2.*(q[1] * q[2] - q[0] * q[3]);
	c[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
	c[1][2] = 2.*(q[2] * q[3] + q[0] * q[1]);
	c[2][0] = 2.*(q[1] * q[3] + q[0] * q[2]);
	c[2][1] = 2.*(q[2] * q[3] - q[0] * q[1]);
	c[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

/**
* @brief  optq 四元数归一化 q = q / |q|
* @param  q 浮点型 返回四元数
* @retval None
*/
void optq(double q[4])
{
	double qq;
	int i;
	/* 求四元素的模 */
	qq = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	for (i = 0; i < 4; i++)
	{
		q[i] = q[i] / qq;                                  /* 求四元素的归一化的四个元素值 */
	}

}

/**
* @brief  cnb2ang 姿态矩阵算出姿态角
* @param  angle 浮点型 返回姿态角
*         cnb 浮点型 姿态矩阵
* @retval None
*/
void cnb2ang(double cnb[3][3], double angle[3])
{
	/* 由姿态矩阵算出航向角 */
	angle[2] = -atan2(cnb[1][0], cnb[1][1]);

	/* 由姿态矩阵算出纵摇角 */
	angle[0] = atan2(cnb[1][2], sqrt(cnb[1][0] * cnb[1][0] + cnb[1][1] * cnb[1][1]));

	/* 由姿态矩阵算出横摇角 */
	angle[1] = -atan2(cnb[0][2], cnb[2][2]);
}

/**
* @brief  ang2q 姿态角计算四元数
* @param  q 浮点型 返回四元数
angle 浮点型 输入姿态角
* @retval None
*/
void ang2q(double q[4], double angle[3])
{
	q[0] = cos(angle[2] / 2)*cos(angle[0] / 2)*cos(angle[1] / 2) - sin(angle[2] / 2)*sin(angle[0] / 2)*sin(angle[1] / 2);
	q[1] = cos(angle[2] / 2)*sin(angle[0] / 2)*cos(angle[1] / 2) - sin(angle[2] / 2)*cos(angle[0] / 2)*sin(angle[1] / 2);
	q[2] = cos(angle[2] / 2)*cos(angle[0] / 2)*sin(angle[1] / 2) + sin(angle[2] / 2)*sin(angle[0] / 2)*cos(angle[1] / 2);
	q[3] = sin(angle[2] / 2)*cos(angle[0] / 2)*cos(angle[1] / 2) + cos(angle[2] / 2)*sin(angle[0] / 2)*sin(angle[1] / 2);
	optq(q);
}

/**
* @brief  latitog 根据纬度值计算重力加速度g 基于WGS-84
* @param  lati 浮点型 纬度值
* @retval g    浮点型 重力加速度值
*         秦永元《惯性导航》 P213
*                               1 + 0.00193185138639*sin(lati)^2
*         g = 978.03267714 * ---------------------------------------
*                             sqrt(1 - 0.00669437999013*sin(lati)^2)
*/
double latitog(double lati)
{
	double gi, g;
	gi = sqrt(1 - 0.00669437999013*sin(lati)*sin(lati));/*算出所在纬度的gi */
	g = 9.7803267714*(1 + 0.00193185138639*sin(lati)*sin(lati)) / gi;/* 算出所在纬度的重力加速度 */
	return g;
}

void DeltaAtt2Phi(SYS_ELEMENT temp_infor, double phi[3], double delta_att[3])
{
	double Aw[3][3] = { 0 };
	double s1 = sin(temp_infor.att_angle[0]);
	double s3 = sin(temp_infor.att_angle[2]);
	double c1 = cos(temp_infor.att_angle[0]);
	double c3 = cos(temp_infor.att_angle[2]);
	Aw[0][0] = c3;
	Aw[0][1] = -c1*s3;
	Aw[1][0] = s3;
	Aw[1][1] = c1*c3;
	Aw[2][1] = s1;
	Aw[2][2] = 1;
	vecmul(3, 3, phi, (double *)Aw, delta_att);
}

//设备用的kalman滤波程序R阵设置
void set_R(FILTER &kal,double v1, double v2, double ang)	//scliuseu20101002
{
	kal.R_measure[0][0] = pow(v1, 2);			//scliuseu20101002
	kal.R_measure[1][1] = pow(v2, 2);
	kal.R_measure[2][2] = pow(ang*D2R, 2);
}
////member function of class CALIPMT
CALIPMT::CALIPMT()
{
	int i, j;
	memset(bias_gyro, 0, sizeof(bias_gyro));
	memset(bias_acce, 0, sizeof(bias_acce));
	memset(bias_gyro_random, 0, sizeof(bias_gyro));
	memset(bias_acce_random, 0, sizeof(bias_acce));
	memset(fix_err, 0, sizeof(fix_err));
	ang2cnb(fix_mat, fix_err);
	memset(Eg_ang, 0, sizeof(Eg_ang));
	memset(Ea_ang, 0, sizeof(Ea_ang));
	memset(Eg_mat, 0, sizeof(Eg_mat));
	memset(Ea_mat, 0, sizeof(Ea_mat));
	memset(Eg_mat_inv, 0, sizeof(Eg_mat_inv));
	memset(Ea_mat_inv, 0, sizeof(Ea_mat_inv));
	for (i = 0; i < 3; i++)
	{
		Ca[i] = 1;
		Cg[i] = 1;
		for (j = 0; j < 3; j++)
		{
			if (i == j)
			{
				Eg_mat_inv[i][j] = 1;
				Ea_mat_inv[i][j] = 1;
			}
		}
	}
	
}
void CALIPMT::Eang2mat()
{
	int i, j;
	Eg_mat_inv[0][1] = Eg_ang[0]; Eg_mat_inv[1][0] = -Eg_ang[1];
	Eg_mat_inv[0][2] = -Eg_ang[4]; Eg_mat_inv[2][0] = Eg_ang[5];
	Eg_mat_inv[1][2] = Eg_ang[2]; Eg_mat_inv[2][1] = -Eg_ang[3];

	Ea_mat_inv[0][1] = Ea_ang[0]; Ea_mat_inv[1][0] = -Ea_ang[1];
	Ea_mat_inv[0][2] = -Ea_ang[4]; Ea_mat_inv[2][0] = Ea_ang[5];
	Ea_mat_inv[1][2] = Ea_ang[2]; Ea_mat_inv[2][1] = -Ea_ang[3];
	for (i = 0; i < 3; i++)
	{
		for (j = 0; j < 3; j++)
		{
			if (i == j)
			{
				Eg_mat_inv[i][j] = 1;
				Ea_mat_inv[i][j] = 1;
			}
		}
	}
	amamul(3, 3, (double *)Eg_mat, (double *)Eg_mat_inv, 1);
	amamul(3, 3, (double *)Ea_mat, (double *)Ea_mat_inv, 1);
	mainv(3, (double *)Eg_mat_inv);
	mainv(3, (double *)Ea_mat_inv);
}
void CALIPMT::fixup(double fix_angle[3], double Cnb0[3][3])
{	
	double Cnb[3][3];
	ang2cnb(fix_mat, fix_err);
	mamul(3, 3, 3, (double *)Cnb, (double *)fix_mat, (double *)Cnb0);
	cnb2ang(Cnb, fix_angle);
}
void CALIPMT::IMUcalibrate(SYS_ELEMENT &infor)
{
	double temp_g[3], temp_a[3];
	for (int i = 0; i < 3; i++)
	{
		temp_a[i] = infor.acce_b[i] / Ca[i] - bias_acce[i];
		temp_g[i] = infor.gyro_wib_b[i] / Cg[i] - bias_gyro[i];
	}
	vecmul(3, 3, infor.gyro_wib_b, (double *)Eg_mat_inv, temp_g);
	vecmul(3, 3, infor.acce_b, (double *)Ea_mat_inv, temp_a);
}



