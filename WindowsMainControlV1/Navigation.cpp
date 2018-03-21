#include "stdafx.h"
#include "Navigation.h"


// 无高度阻尼纯捷联解算，原始方法
void sinscal_zundamp(double quart_del )
{
	double	gi, g, Re, Rn;
	double dlati, dlongi;
	double win_n[3], win_b[3], wnb_b[3]; 
	double a_wib_b[3];
	double sita[3];
	double fw[3];

	/* 导航系下g值计算，可以优化，放到全局变量，统一初始化计算 */
	gi=sqrt(1-0.00669437999013*sin(infor.pos[0])*sin(infor.pos[0]));
	g=9.7803267714*(1+0.00193185138639*sin(infor.pos[0])*sin(infor.pos[0]))/gi;
	infor.g=g;
	/* 计算曲率半径，可以优化 */
	Rn = RE*(1.0-2.0*AEE+3.0*AEE*sin(infor.pos[0])*sin(infor.pos[0]));//子午圈
	Re = RE*(1.0+AEE*sin(infor.pos[0])*sin(infor.pos[0]));

	/* 经纬度微分 惯性导航8.1.6 */
	dlati  = infor.vel_n[1] / (Rn + infor.pos[2]);
 	dlongi = infor.vel_n[0] / ((Re + infor.pos[2])* cos(infor.pos[0]));
	
	/* Ci->n 地球自转角速度在n系上的投影，同时考虑导航系的变化 */
	win_n[0] = -dlati;
	win_n[1] = WIE * cos(infor.pos[0]) + dlongi * cos(infor.pos[0]);
	win_n[2] = WIE * sin(infor.pos[0]) + dlongi * sin(infor.pos[0]);

	/* Ci->n 在b系上的投影，Cn->b 由粗对准计算 */
	vecmul(3,3,win_b,(double*)infor.cnb_mat,win_n);
	if(sysc.Fs==100)//haishi设备用
		memset(win_b, 0, sizeof(win_b));
//	vecadd(3,a_wib_b,infor.gyro_wib_b,infor.gyro_old);
//	avecmul(3,a_wib_b,a_wib_b,0.5);
	avecmul(3, a_wib_b, infor.gyro_wib_b, 1);  //yucia调试用

	vecsub(3,wnb_b,a_wib_b,win_b);
	avecmul(3,sita,wnb_b,quart_del);

	/* 通过四元数更新Cn->b */
	qcal(sita,infor.quart);
	optq(infor.quart);
	q2cnb(infor.cnb_mat,infor.quart);
	cnb2ang(infor.cnb_mat,infor.att_angle);          // 计算姿态角
	
	/* 转置Cn->b，将加表两侧转到导航系 */
	maturn(3,3,(double*)infor.cbn_mat,(double*)infor.cnb_mat);
	vecmul(3,3,infor.acce_n,(double*)infor.cbn_mat,infor.acce_b);

	/* 利用比力公式计算 */
	fw[0] = -dlati;
	fw[1] = 2 * WIE * cos(infor.pos[0]) + dlongi * cos(infor.pos[0]);
	fw[2] = 2 * WIE * sin(infor.pos[0]) + dlongi * sin(infor.pos[0]);

	/* 惯性导航式8.1.4 */
	infor.dvel_n[0] = infor.acce_n[0] + fw[2] * infor.vel_n[1] - fw[1] * infor.vel_n[2];
	infor.dvel_n[1] = infor.acce_n[1] - fw[2] * infor.vel_n[0] + fw[0] * infor.vel_n[2];
	infor.dvel_n[2] = infor.acce_n[2] + fw[1] * infor.vel_n[0] - fw[0] * infor.vel_n[1] - g;   //+
	
	infor.vel_n[0] += 	infor.dvel_n[0] * quart_del;
	infor.vel_n[1] += 	infor.dvel_n[1] * quart_del;
	infor.vel_n[2] +=   infor.dvel_n[2] * quart_del;
	
	/********* 经度 & 纬度 **********/
	infor.pos[0] += dlati * quart_del;
	infor.pos[1] += dlongi * quart_del;	
    infor.pos[2] += infor.vel_n[2] * quart_del;      //+
}
//无高度阻尼，旋转矢量法，补偿一些误差
void sinscal_rv(double quart_del)
{
	double	gi, g, Re, Rn;
	double dlati, dlongi;
	double win_n[3];	
	double fw[3];
	double rvin[3];//旋转矢量
	double rvib_old[3] = { 0 }, rvib_bu[3];
	double tempq[4];
	double vrot[3],vscull[3],tempscull[3];
	double df[3], fn[3], an[3];
	double dsita[3], dsita_old[3], dsb[3], dsb_old[3];

	/* 导航系下g值计算，可以优化，放到全局变量，统一初始化计算 */
	gi = sqrt(1 - 0.00669437999013*sin(infor.pos[0])*sin(infor.pos[0]));
	g = 9.7803267714*(1 + 0.00193185138639*sin(infor.pos[0])*sin(infor.pos[0])) / gi;
	infor.g = g;
	/* 计算曲率半径，可以优化 */
	Rn = RE*(1.0 - 2.0*AEE + 3.0*AEE*sin(infor.pos[0])*sin(infor.pos[0]));//子午圈
	Re = RE*(1.0 + AEE*sin(infor.pos[0])*sin(infor.pos[0]));

	/* 经纬度微分 惯性导航8.1.6 */
	dlati = infor.vel_n[1] / (Rn + infor.pos[2]);
	dlongi = infor.vel_n[0] / ((Re + infor.pos[2])* cos(infor.pos[0]));

	/* Ci->n 地球自转角速度在n系上的投影，同时考虑导航系的变化 */
	win_n[0] = -dlati;
	win_n[1] = WIE * cos(infor.pos[0]) + dlongi * cos(infor.pos[0]);
	win_n[2] = WIE * sin(infor.pos[0]) + dlongi * sin(infor.pos[0]);
	avecmul(3, rvin, win_n, -quart_del);
	/*memcpy(rvib_old, infor.rvib, sizeof(infor.rvib));
	vecadd(3, infor.rvib, infor.gyro_wib_b, infor.gyro_old);
	avecmul(3, infor.rvib, infor.rvib, quart_del/2);
	if (1 == sysc.data_cnt)
		memcpy(rvib_old, infor.rvib, sizeof(infor.rvib));
	cvecmul(rvib_bu, rvib_old, infor.rvib);
	avecmul(3, rvib_bu, rvib_bu, 1.0/12.0);
	vecadd(3, rvib_bu, infor.rvib, rvib_bu);
	rv2q(tempq, rvib_bu);
	qmul(infor.quart, infor.quart, tempq);
	if (sysc.Fs == 100)//haishi设备用
		memset(rvin, 0, sizeof(rvin));
	rv2q(tempq, rvin);
	qmul(infor.quart, tempq, infor.quart);
	//optq(infor.quart);
	q2cnb(infor.cnb_mat, infor.quart);
	cnb2ang(infor.cnb_mat, infor.att_angle);          // 计算姿态角*/
	double  win_b[3], wnb_b[3], sita[3];
	double a_wib_b[3];
	win_n[0] = -dlati;
	win_n[1] = WIE * cos(infor.pos[0]) + dlongi * cos(infor.pos[0]);
	win_n[2] = WIE * sin(infor.pos[0]) + dlongi * sin(infor.pos[0]);

	/* Ci->n 在b系上的投影，Cn->b 由粗对准计算 */
	vecmul(3, 3, win_b, (double*)infor.cnb_mat, win_n);
	vecadd(3, a_wib_b, infor.gyro_wib_b, infor.gyro_old);
	avecmul(3, a_wib_b, a_wib_b, 0.5);

	vecsub(3, wnb_b, a_wib_b, win_b);
	avecmul(3, sita, wnb_b, quart_del);

	/* 通过四元数更新Cn->b */
	qcal(sita, infor.quart);
	optq(infor.quart);
	q2cnb(infor.cnb_mat, infor.quart);
	cnb2ang(infor.cnb_mat, infor.att_angle);          // 计算姿态角

	///////速度更新												  		
	avecmul(3, dsita, infor.gyro_wib_b, quart_del);
	avecmul(3, dsb, infor.acce_b, quart_del);
	cvecmul(vrot,dsita, dsb);
	avecmul(3, vrot, vrot, 0.5);

	avecmul(3, dsita_old, infor.gyro_old, quart_del);
	avecmul(3, dsb_old, infor.acce_old, quart_del);
	cvecmul(vscull, dsita_old, dsb);
	cvecmul(tempscull, dsita, dsb_old);
	vecadd(3, tempscull, vscull, tempscull);
	avecmul(3, vscull, tempscull, 1.0 / 12.0);

	vecadd(3, df, vscull, vrot);
	avecmul(3, df, df, 1/ quart_del);

	vecadd(3, fn, df, infor.acce_b);
	maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
	vecmul(3, 3, fn, (double*)infor.cbn_mat, fn);

	avecmul(3, rvin, rvin, 0.5);	
	rv2q(tempq, rvin);
	qmulv(an, tempq, fn);

	/* 利用比力公式计算 */
	fw[0] = -dlati;
	fw[1] = 2 * WIE * cos(infor.pos[0]) + dlongi * cos(infor.pos[0]);
	fw[2] = 2 * WIE * sin(infor.pos[0]) + dlongi * sin(infor.pos[0]);

	/* 惯性导航式8.1.4 */
	infor.dvel_n[0] = an[0] + fw[2] * infor.vel_n[1];// -fw[1] * infor.vel_n[2];
	infor.dvel_n[1] = an[1] - fw[2] * infor.vel_n[0];// +fw[0] * infor.vel_n[2];
 	infor.dvel_n[2] = an[2] + fw[1] * infor.vel_n[0] - fw[0] * infor.vel_n[1] - g;   //+
	memcpy(infor.old_v, infor.vel_n, sizeof(infor.vel_n));
	infor.vel_n[0] += infor.dvel_n[0] * quart_del;
	infor.vel_n[1] += infor.dvel_n[1] * quart_del;
	infor.vel_n[2] += infor.dvel_n[2] * quart_del;

	/********* 经度 & 纬度 **********/
	infor.pos[0] += dlati * quart_del;
	infor.pos[1] += dlongi * quart_del;
	infor.pos[2] += infor.vel_n[2] * quart_del;      //+

	vecmul(3, 3, infor.acce_n, (double*)infor.cbn_mat, infor.acce_b);
}

#pragma region HAISHI
//设备用姿态更新算法
void attUpdate()
{
	double Re, Rn;
	double dlati, dlongi;
	double win_n[3], win_b[3], wnb_b[2][3];
	double sita[3];
	double fw[3];
	double gyro[2][3];
	double quart_del = 0.01;
	int i;
	//scliuseu modified for 二维 2010
	//	for(i=0;i<2;i++)
	for (i = 0; i<3; i++)
	{
		gyro[0][i] = infor.gyro_old[i];// - kal.gyro_comps[i];
		gyro[1][i] = infor.gyro_wib_b[i];// - kal.gyro_comps[i];
	}
	//scliuseu modified for 二维 2010，此处修改
	//gyro[0][2] = infor.gyro_old[2];// - kal.gyro_comps[2];//-0.05*D2R/3600;//
	//gyro[1][2] = infor.gyro_wib_b[2];// - kal.gyro_comps[2];//-0.05*D2R/3600
	//scliuseu modified for 二维 2010

	Rn = RE*(1.0 - 2.0*AEE + 3.0*AEE*sin(infor.pos[0])*sin(infor.pos[0]));
	Re = RE*(1.0 + AEE*sin(infor.pos[0])*sin(infor.pos[0]));

	dlati = infor.vel_n[1] / Rn;
	dlongi = infor.vel_n[0] / (Re * cos(infor.pos[0]));

	/************* 陀螺 ***************/
	win_n[0] = -dlati;
	win_n[1] = WIE * cos(infor.pos[0]) + dlongi * cos(infor.pos[0]);
	win_n[2] = WIE * sin(infor.pos[0]) + dlongi * sin(infor.pos[0]);
	//haishi录数录的就是wnbb，故不需要win_b进行一次补偿
//	vecmul(3, 3, win_b, (double*)infor.cnb_mat, win_n);
//	if (TEST_MODE == 2)
//	{
		win_b[0] = 0;
		win_b[1] = 0;
		win_b[2] = 0;
//	}
	for (i = 0; i < 2; i++)
		vecsub(3, wnb_b[i], gyro[i], win_b);

	vecadd(3, sita, wnb_b[0], wnb_b[1]);

	avecmul(3, sita, sita, quart_del / 2.);

	//printf("sita %lf",sita[2]);
	if (infor.flagFire)
	{
		if (sita[2]>0.000034) sita[2] = 0.000034;	/*0.2度=0.0034弧度*/
		if (sita[2]<-0.000034) sita[2] = -0.000034;
	}

	//杆臂效应有关量
	for (i = 0; i<3; i++)
	{
		infor.wnb_b[i] = sita[i] * 100;
		infor.wnb_b_arm[i] = infor.wnb_b_arm[i] * 0.2 + infor.wnb_b[i] * 0.8;  /*   20141016 xxs   */
	}

	qcal(sita, infor.quart);
	optq(infor.quart);
	q2cnb(infor.cnb_mat, infor.quart);
	cnb2ang(infor.cnb_mat, infor.att_angle);
	/*********** accelerator ***********/
	maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
	vecmul(3, 3, infor.acce_n, (double*)infor.cbn_mat, infor.acce_b);

	vecadd(3, infor.acce_n_sum, infor.acce_n_sum, infor.acce_n);
	if ((sysc.data_cnt % infor.acce_n_t) == 0)
	{
		for (i = 0; i < 3; i++)
			infor.acce_n[i] = infor.acce_n_sum[i] / infor.acce_n_t;

		for (i = 0; i < 3; i++)
			infor.acce_n_sum[i] = 0.0;

		fw[0] = -dlati;
		fw[1] = 2 * WIE * cos(infor.pos[0]) + dlongi * cos(infor.pos[0]);
		fw[2] = 2 * WIE * sin(infor.pos[0]) + dlongi * sin(infor.pos[0]);

		infor.dvel_n[0] = infor.acce_n[0] + fw[2] * infor.vel_n[1] - fw[1] * infor.vel_n[2];
		infor.dvel_n[1] = infor.acce_n[1] - fw[2] * infor.vel_n[0] + fw[0] * infor.vel_n[2];
		//infor.dvel_n[2] = 0;//infor.acce_n[2] + fw[1] * infor.vel_n[0] - fw[0] * infor.vel_n[1] - GRAVITY;
		//scliuseu modified for 二维 2010
		infor.dvel_n[2] = 0;//infor.acce_n[2] + fw[1] * infor.vel_n[0] - fw[0] * infor.vel_n[1] - GRAVITY;

		infor.vel_n[0] += infor.dvel_n[0] * quart_del * infor.acce_n_t;
		infor.vel_n[1] += infor.dvel_n[1] * quart_del * infor.acce_n_t;
		infor.vel_n[2] += infor.dvel_n[2] * quart_del * infor.acce_n_t;
		infor.vel_n[2] = 0;
		vecmul(3, 3, infor.vel_b, (double*)infor.cnb_mat, infor.vel_n);

	}
	/*********** lati & longi *************/
	infor.pos[0] += dlati * quart_del;
	infor.pos[1] += dlongi * quart_del;
	infor.pos[2] += infor.vel_n[2] * quart_del;      //+
	//infor.pos[2] = 0;
}
//设备用F阵计算
void F_matrix()
{
	int	i, j;
	double	w0[5], w1[3], w2[3];
	double	Re, Rn;

	Rn = RE*(1.0 - 2.0*AEE + 3.0*AEE*sin(infor.pos[0])*sin(infor.pos[0]));
	Re = RE*(1.0 + AEE*sin(infor.pos[0])*sin(infor.pos[0]));

	w0[0] = (2 * WIE * cos(infor.pos[0]) + infor.vel_n[0] / cos(infor.pos[0]) / cos(infor.pos[0]) / Re) * infor.vel_n[1];
	w0[1] = (2 * WIE * cos(infor.pos[0]) + infor.vel_n[0] / cos(infor.pos[0]) / cos(infor.pos[0]) / Re) * infor.vel_n[0];
	w0[2] = WIE * cos(infor.pos[0]) + infor.vel_n[0] / cos(infor.pos[0]) / cos(infor.pos[0]) / Re;
	w0[3] = infor.vel_n[0] * sin(infor.pos[0]) / cos(infor.pos[0]) / cos(infor.pos[0]) / Re;
	w0[4] = WIE * sin(infor.pos[0]);

	w1[0] = -infor.vel_n[1] / Re;
	w1[1] = 2 * WIE * cos(infor.pos[0]) + infor.vel_n[0] / Re;
	w1[2] = 2 * WIE * sin(infor.pos[0]) + infor.vel_n[0] * tan(infor.pos[0]) / Re;

	w2[0] = -infor.vel_n[1] / Rn;
	w2[1] = WIE * cos(infor.pos[0]) + infor.vel_n[0] / Re;
	w2[2] = WIE * sin(infor.pos[0]) + infor.vel_n[0] * tan(infor.pos[0]) / Re;

	for (i = 0; i < sta_num; i++)
		for (j = 0; j < sta_num; j++)
			kal.state_matrix[i][j] = 0.0;

	kal.state_matrix[0][0] = infor.vel_n[1] / Rn * tan(infor.pos[0]);
	kal.state_matrix[0][1] = w1[2];
	kal.state_matrix[0][3] = -infor.acce_n[2];
	kal.state_matrix[0][4] = infor.acce_n[1];
	kal.state_matrix[0][5] = w0[0];
	kal.state_matrix[0][7] = -infor.cbn_mat[0][0];
	kal.state_matrix[0][8] = -infor.cbn_mat[0][1];

	kal.state_matrix[1][0] = -2 * w2[2];
	kal.state_matrix[1][2] = infor.acce_n[2];
	kal.state_matrix[1][4] = -infor.acce_n[0];
	kal.state_matrix[1][5] = -w0[1];
	kal.state_matrix[1][7] = -infor.cbn_mat[1][0];
	kal.state_matrix[1][8] = -infor.cbn_mat[1][1];

	kal.state_matrix[2][1] = -1 / Rn;
	kal.state_matrix[2][3] = w2[2];
	kal.state_matrix[2][4] = -w2[1];
	kal.state_matrix[2][9] = -infor.cbn_mat[0][0];
	kal.state_matrix[2][10] = -infor.cbn_mat[0][1];
	kal.state_matrix[2][11] = -infor.cbn_mat[0][2];

	kal.state_matrix[3][0] = 1 / Re;
	kal.state_matrix[3][2] = -w2[2];
	kal.state_matrix[3][4] = w2[0];
	kal.state_matrix[3][5] = -w0[4];
	kal.state_matrix[3][9] = -infor.cbn_mat[1][0];
	kal.state_matrix[3][10] = -infor.cbn_mat[1][1];
	kal.state_matrix[3][11] = -infor.cbn_mat[1][2];

	kal.state_matrix[4][0] = tan(infor.pos[0]) / Re;
	kal.state_matrix[4][2] = w2[1];
	kal.state_matrix[4][3] = -w2[0];
	kal.state_matrix[4][5] = w0[2];
	kal.state_matrix[4][9] = -infor.cbn_mat[2][0];
	kal.state_matrix[4][10] = -infor.cbn_mat[2][1];
	kal.state_matrix[4][11] = -infor.cbn_mat[2][2];

	kal.state_matrix[5][1] = 1 / Rn;
	kal.state_matrix[6][0] = 1 / cos(infor.pos[0]) / Re;
	kal.state_matrix[6][5] = w0[3];
}
//设备用kalman滤波
void kal_algo()//+
{
	int		i, j;
	double	fil_del = 1;
	double	state_dis[sta_num][sta_num], state_transpose[sta_num][sta_num], Q_dis[sta_num][sta_num], H_transpose[sta_num][mea_num], R_dis[mea_num][mea_num];
	double	X_forecast[sta_num], P_forecast[sta_num][sta_num], K_matrix[sta_num][mea_num], K_matrix1[sta_num][mea_num];
	double	unit_matr[sta_num][sta_num];
	double	tmp1[sta_num][sta_num], tmp2[mea_num][sta_num], tmp3[mea_num][mea_num], tmp4[sta_num][mea_num], tmp5[mea_num], tmp6[sta_num];

	for (i = 0; i < sta_num; i++)
		for (j = 0; j < sta_num; j++)
		{
			if (i == j)
				unit_matr[i][j] = 1.;
			else
				unit_matr[i][j] = 0.;
		}

	//一步预测 X
	maadd(sta_num, sta_num, (double *)state_dis, (double *)unit_matr, (double *)kal.F_kal);
	vecmul(sta_num, sta_num, X_forecast, (double *)state_dis, kal.X_vector);

	//离散化 quart
	mamul(sta_num, sta_num, sta_num, (double *)tmp1, (double *)state_dis, (double *)kal.Q_state);
	maturn(sta_num, sta_num, (double *)state_transpose, (double *)state_dis);
	mamul(sta_num, sta_num, sta_num, (double *)tmp1, (double *)tmp1, (double *)state_transpose);
	maadd(sta_num, sta_num, (double *)Q_dis, (double *)kal.Q_state, (double *)tmp1);
	amamul(sta_num, sta_num, (double *)Q_dis, (double *)Q_dis, fil_del / 2);

	//scliuseu modified for 二维 2010 //scliuseu20100912
	if (kal.P_matrix[0][0] < 5.0e-4)//+
		kal.P_matrix[0][0] = 5.0e-4;

	if (kal.P_matrix[1][1] < 30.0e-4)//+
		kal.P_matrix[1][1] = 30.0e-4;

	if (kal.P_matrix[2][2] < 2.0e-4)//+
		kal.P_matrix[2][2] = 2.0e-4;

	if (kal.P_matrix[3][3] < 6.0e-4)//+
		kal.P_matrix[3][3] = 6.0e-4;

	if (kal.P_matrix[4][4] < 6.0e-8)//+
		kal.P_matrix[4][4] = 6.0e-8;

	if (kal.P_matrix[11][11] < 6.0e-4)//+
		kal.P_matrix[11][11] = 6.0e-4;

	//预测 P
	mamul(sta_num, sta_num, sta_num, (double *)tmp1, (double *)state_dis, (double *)kal.P_matrix);
	mamul(sta_num, sta_num, sta_num, (double *)tmp1, (double *)tmp1, (double *)state_transpose);
	maadd(sta_num, sta_num, (double *)P_forecast, (double *)tmp1, (double *)Q_dis);

	//滤波增益 K
	mamul(mea_num, sta_num, sta_num, (double *)tmp2, (double *)kal.H_matrix, (double *)P_forecast);
	maturn(mea_num, sta_num, (double *)H_transpose, (double *)kal.H_matrix);
	mamul(mea_num, sta_num, mea_num, (double *)tmp3, (double *)tmp2, (double *)H_transpose);
	amamul(mea_num, mea_num, (double *)R_dis, (double *)kal.R_measure, fil_del);
	maadd(mea_num, mea_num, (double *)tmp3, (double *)tmp3, (double *)R_dis);
	mainv(mea_num, (double *)tmp3);
	//inv33((double *)tmp3);
	mamul(sta_num, sta_num, mea_num, (double *)tmp4, (double *)P_forecast, (double *)H_transpose);
	mamul(sta_num, mea_num, mea_num, (double *)K_matrix, (double *)tmp4, (double *)tmp3);

	for (i = 0; i < sta_num; i++)
		for (j = 0; j < mea_num; j++)
			K_matrix1[i][j] = K_matrix[i][j];

	K_matrix1[0][2] = 0.0;
	K_matrix1[1][2] = 0.0;
	K_matrix1[2][2] = 0.0;
	K_matrix1[3][2] = 0.0;

	//更新 P
	mamul(sta_num, mea_num, sta_num, (double *)tmp1, (double *)K_matrix, (double *)kal.H_matrix);
	masub(sta_num, sta_num, (double *)tmp1, (double *)unit_matr, (double *)tmp1);
	mamul(sta_num, sta_num, sta_num, (double *)kal.P_matrix, (double *)tmp1, (double *)P_forecast);

	//更新 X
	vecmul(mea_num, sta_num, tmp5, (double *)kal.H_matrix, X_forecast);
	vecsub(mea_num, tmp5, kal.Mea_vector, tmp5);

	vecmul(sta_num, mea_num, tmp6, (double *)K_matrix1, tmp5);  //新算法
																//	vecmul(sta_num,mea_num,tmp6,(double *)K_matrix,tmp5);  老算法
	vecadd(sta_num, kal.X_vector, X_forecast, tmp6);

	for (i = 0; i < sta_num; i++)
		for (j = 0; j < sta_num; j++)
			kal.F_kal[i][j] = 0.0;
}
//闭环校正
void attCorrect_HeadOpen_test()
{
	double cnn_mat[3][3];
	int i;
	double ang0[3], ang1[3];
	cnb2ang(infor.cnb_mat, ang0);
	for (i = 0; i < 2; i++)//+	//此处有问题：速度二维，已经修改
		infor.vel_n[i] -= kal.X_vector[i];
	X2cnn(cnn_mat, kal.X_vector+2);
	mamul(3, 3, 3, (double *)infor.cnb_mat, (double *)infor.cnb_mat, (double *)cnn_mat);
	cnb2ang(infor.cnb_mat, ang1);
	ang1[2] = ang0[2] + kal.X_vector[4];
	ang2cnb(infor.cnb_mat, ang1);
	maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
	cnb2q(infor.cnb_mat, infor.quart);
	cnb2ang(infor.cnb_mat, infor.att_angle);

	for (i = 0; i < 2; i++)
	{
		kal.gyro_comps[i] += kal.X_vector[i + 9];
	}
	if (infor.flagComps == 1)/*infor.flagComps为1，对航向陀螺漂移进行补偿*/
		kal.gyro_comps[2] += kal.X_vector[11];

	fosn.recnum++;
}
//速度+航向匹配，输入为东向北向水平速度和航向姿态角,mode参数用来调整组合模式
void navigation(double o_vel_e,double o_vel_n,double o_ang,char mode)
{
	int i,j;
	static int CountOfCorrect = 100;

	//////////////////
	if (sysc.data_cnt>10000)
		set_R(kal, 4.5, 6.0, 3.0);
	if (0 == sysc.data_cnt%sysc.Fs)
	{	
		memcpy(kal.F_kal, kal.F_sum, sizeof(kal.F_sum));
		memset(kal.F_sum, 0, sizeof(kal.F_sum));
	
		kal.Mea_vector[0] = infor.vel_n[0] - o_vel_e;// + 0.4 * white());
		kal.Mea_vector[1] = infor.vel_n[1] - o_vel_n;//+ 0.4 * white());
		kal.Mea_vector[2] = infor.att_angle[2] - (o_ang);
		cvecmul(infor.vel_arm, infor.wnb_b_arm, infor.rp);		
		vecmul(3, 3, infor.vel_arm, (double *)infor.cbn_mat, infor.vel_arm);
		kal.Mea_vector[0] = kal.Mea_vector[0] - infor.vel_arm[0];
		kal.Mea_vector[1] = kal.Mea_vector[1] - infor.vel_arm[1];
		if (sysc.data_cnt>16800)
		{
			if (kal.Mea_vector[0] > 1.0) kal.Mea_vector[0] = 1.0;
			if (kal.Mea_vector[0] < -1.0) kal.Mea_vector[0] = -1.0;

			if (kal.Mea_vector[1] > 1.0) kal.Mea_vector[1] = 1.0;
			if (kal.Mea_vector[1] < -1.0) kal.Mea_vector[1] = -1.0;
		}

		if (kal.Mea_vector[2] > PAI)
			kal.Mea_vector[2] -= 2 * PAI;
		if (kal.Mea_vector[2] < -PAI)
			kal.Mea_vector[2] += 2 * PAI;
		if (fabs(kal.Mea_vector[2]) > 0.5*D2R)
			infor.flagComps = 0;
		else
			infor.flagComps = 1;
		for (i = 0; i < sta_num; i++)
			kal.X_vector[i] = 0.0;	
		kal_algo();
		if (mode == NAVI_HAISHI_JZ)
		{
			kal.X_vector[0] = kal.Mea_vector[0];
			kal.X_vector[1] = kal.Mea_vector[1];
		}	
		for (i = 0; i<12; i++)
		{
			kal.X_vector[i] = kal.X_vector[i] / 100.0;
		}		
		CountOfCorrect = 0;
		for (i = 0; i < 3; i++)
		{
			infor.gyro_bias_esti[i] = kal.X_vector[9 + i] * 180 / 3.14 * 3600;
			infor.acce_bias_esti[i] = kal.X_vector[7 + i] * 1000000 / 9.78;
		}
		infor.acce_bias_esti[2] = 0;
	}
	if (CountOfCorrect < 100)
	{
		attCorrect_HeadOpen_test();
		CountOfCorrect++;
	}
	attUpdate();
	F_matrix();
	amamul(sta_num, sta_num, (double *)kal.state_matrix, (double *)kal.state_matrix, 0.01);//0.01 means quart_del = 10ms
	maadd(sta_num, sta_num, (double *)kal.F_sum, (double *)kal.F_sum, (double *)kal.state_matrix);


}
#pragma endregion HAISHI

#pragma region Transverse
// 地理系参数转横向坐标系
void g2S(struct SYS_ELEMENTtransverse &inforS, struct SYS_ELEMENT &infor)
{
	//横向初始经纬度
	inforS.lati_S = atan(cos(infor.pos[0])*sin(infor.pos[1]) / sqrt(sin(infor.pos[0])*sin(infor.pos[0])
		+ cos(infor.pos[0])*cos(infor.pos[0])*cos(infor.pos[1])*cos(infor.pos[1])));
	inforS.longi_S = atan2(cos(infor.pos[0])*cos(infor.pos[1]), sin(infor.pos[0]));
	inforS.high_S = infor.pos[2];
	inforS.P = atan2(cos(infor.pos[1]), -sin(infor.pos[0]) * sin(infor.pos[1]));
	if (inforS.P< 0)
		inforS.P += 2 * PAI;
	//初始横轴姿态角
	inforS.att_angle_S[0] = infor.att_angle[0];
	inforS.att_angle_S[1] = infor.att_angle[1];
	inforS.att_angle_S[2] = infor.att_angle[2] + inforS.P;
	//初始横轴速度
	inforS.vel_S[0] = infor.vel_n[0] * cos(inforS.P) - infor.vel_n[1] * sin(inforS.P);
	inforS.vel_S[1] = infor.vel_n[0] * sin(inforS.P) + infor.vel_n[1] * cos(inforS.P);
	inforS.vel_S[2] = infor.vel_n[2];
}
//横向坐标系转地理系参数，只针对inforS对象
void S2g()
{
	double C[3][3];
	//传统经纬度
	inforS.lati = atan(cos(inforS.lati_S)*cos(inforS.longi_S) / sqrt(cos(inforS.lati_S)*cos(inforS.lati_S)*sin(inforS.longi_S)*sin(inforS.longi_S) +
		sin(inforS.lati_S)*sin(inforS.lati_S)));
	inforS.longi = atan2(sin(inforS.lati_S), cos(inforS.lati_S)*sin(inforS.longi_S));
	inforS.high = inforS.high_S;
	inforS.P = atan2(sin(inforS.longi_S), -sin(inforS.lati_S)*cos(inforS.longi_S));
	if (inforS.P < 0)
		inforS.P += 2 * PAI;
	//传统姿态角
	inforS.att_angle[0] = inforS.att_angle_S[0];
	inforS.att_angle[1] = inforS.att_angle_S[1];
	inforS.att_angle[2] = inforS.att_angle_S[2] - inforS.P;
	ang2cnb(C, inforS.att_angle); cnb2ang(C, inforS.att_angle);
	//传统速度
	inforS.vel_n[0] = inforS.vel_S[0] * cos(inforS.P) + inforS.vel_S[1] * sin(inforS.P);
	inforS.vel_n[1] = -inforS.vel_S[0] * sin(inforS.P) + inforS.vel_S[1] * cos(inforS.P);
	inforS.vel_n[2] = inforS.vel_S[2];
}
//横向导航解算方法
/**********************************************横向捷联解算*****************************************/
void sinscal_TRANSVERSE(SYS_ELEMENTtransverse &inforS, double quart_del, double gyro[2][3])  //20180319修改，一些中间量保存至结构体
{
	double cla2, clo2;
	double	gi, g, RMh, RNh;
	int i;
	double dlati, dlongi, temp, temp2;
	double sita[3];
	double wis_b[3], wsb_b[2][3];
	double fw[3];
	static bool isfirst = 0;
	if (isfirst == 0)      //仅在此函数中判断与更新，即第一秒发挥初始赋值的作用
	{
		g2S(inforS, infor);
		ang2cnb(inforS.csb_mat, inforS.att_angle_S);
		cnb2q(inforS.csb_mat, inforS.quart_S);
		maturn(3, 3, (double *)inforS.cbs_mat, (double *)inforS.csb_mat);
		isfirst = 1;
	}

	cla2 = cos(inforS.lati_S) * cos(inforS.lati_S);
	clo2 = cos(inforS.longi_S) * cos(inforS.longi_S);
	gi = sqrt(1 - 0.00669437999013 * cla2 * clo2);
	g = 9.7803267714 * (1 + 0.00193185138639 * cla2 * clo2) / gi;
	//
	temp = (1 - E2* cla2* clo2); temp2 = sqrt(temp);
	RMh = RE*(1 - E2) / temp / temp2 + inforS.high_S;//20180319修改
	RNh = RE / temp2 + inforS.high_S;
	inforS.P = atan2(sin(inforS.longi_S), -sin(inforS.lati_S)*cos(inforS.longi_S));
	inforS.t_1 = (1 / RMh - 1 / RNh)*sin(inforS.P)*cos(inforS.P);
	inforS.Rx_1 = sin(inforS.P)*sin(inforS.P) / RMh + cos(inforS.P)*cos(inforS.P) / RNh;
	inforS.Ry_1 = sin(inforS.P)*sin(inforS.P) / RNh + cos(inforS.P)*cos(inforS.P) / RMh;

	dlati = -inforS.vel_S[0] * inforS.t_1 + inforS.vel_S[1] * inforS.Ry_1;
	dlongi = inforS.vel_S[0] * inforS.Rx_1 / cos(inforS.lati_S) - inforS.vel_S[1] * inforS.t_1 / cos(inforS.lati_S);
	//
	inforS.wie_s[0] = -WIE * sin(inforS.longi_S);
	inforS.wie_s[1] = -WIE * sin(inforS.lati_S) * cos(inforS.longi_S);
	inforS.wie_s[2] = WIE * cos(inforS.lati_S) * cos(inforS.longi_S);

	inforS.wis_s[0] = inforS.wie_s[0] - dlati;
	inforS.wis_s[1] = inforS.wie_s[1] + dlongi * cos(inforS.lati_S);
	inforS.wis_s[2] = inforS.wie_s[2] + dlongi * sin(inforS.lati_S);

	vecmul(3, 3, wis_b, (double *)inforS.csb_mat, inforS.wis_s);
	for (i = 0; i < 2; i++)
		vecsub(3, wsb_b[i], gyro[i], wis_b);
//梯形算法
	//vecadd(3, sita, wsb_b[0], wsb_b[1]);
	//avecmul(3, sita, sita, quart_del / 2.0);
//矩形算法（matlab）
	vecadd(3, sita, wsb_b[1], wsb_b[1]);
	avecmul(3, sita, sita, quart_del / 2.0);

	qcal(sita, inforS.quart_S);
	optq(inforS.quart_S);
	q2cnb(inforS.csb_mat, inforS.quart_S);
	cnb2ang(inforS.csb_mat, inforS.att_angle_S); //先更新姿态，用更新后的姿态更新速度位置（和用上一刻的姿态差别应该不大）

												 /********** 加表 ************/
	maturn(3, 3, (double*)inforS.cbs_mat, (double*)inforS.csb_mat);
	vecmul(3, 3, inforS.acce_S, (double*)inforS.cbs_mat, inforS.acce_b);

	fw[0] = inforS.wie_s[0] + inforS.wis_s[0];
	fw[1] = inforS.wie_s[1] + inforS.wis_s[1];
	fw[2] = inforS.wie_s[2] + inforS.wis_s[2];

	inforS.dvel_S[0] = inforS.acce_S[0] + fw[2] * inforS.vel_S[1] - fw[1] * inforS.vel_S[2];
	inforS.dvel_S[1] = inforS.acce_S[1] - fw[2] * inforS.vel_S[0] + fw[0] * inforS.vel_S[2];
	inforS.dvel_S[2] = inforS.acce_S[2] + fw[1] * inforS.vel_S[0] - fw[0] * inforS.vel_S[1] - g;//;
	inforS.vel_S[0] += inforS.dvel_S[0] * quart_del;  //更新速度
	inforS.vel_S[1] += inforS.dvel_S[1] * quart_del;
	inforS.vel_S[2] += inforS.dvel_S[2] * quart_del;

	inforS.lati_S += dlati * quart_del;           //更新位置
	inforS.longi_S += dlongi * quart_del;
	inforS.high_S += inforS.vel_S[2] * quart_del;

	//将解得的横向导航参数转换到传统地理坐标系中
	S2g();
	//将横向导航结果赋给infor，用于显示；
	infor.att_angle[0] = inforS.att_angle[0]; infor.att_angle[1] = inforS.att_angle[1]; infor.att_angle[2] = inforS.att_angle[2];
	ang2cnb(infor.cnb_mat, infor.att_angle);
	infor.vel_n[0] = inforS.vel_n[0]; infor.vel_n[1] = inforS.vel_n[1]; infor.vel_n[2] = inforS.vel_n[2];
	infor.pos[0] = inforS.lati; infor.pos[1] = inforS.longi; infor.pos[2] = inforS.high;
}
/**********************************************横向捷联解算*****************************************/
#pragma endregion Transverse