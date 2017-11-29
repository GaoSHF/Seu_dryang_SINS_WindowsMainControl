#include "stdafx.h"
#include "FineAlign.h"

#pragma region compass method
/**
* @brief  cmp1_algo 罗经法水平精对准
* @param  cmp       罗经法参数
*         quart_del 算法解算周期
* @retval None
*/
void cmp1_algo(COMPALIGN &cmp,double quart_del)
{
	cmp.wc_n[0] = -(infor.vel_n[1]*(1+cmp.k1[1])/RE + infor.vel_n[1]*cmp.k1[2]*quart_del);
	cmp.wc_n[1] =   infor.vel_n[0]*(1+cmp.k1[1])/RE + infor.vel_n[0]*cmp.k1[2]*quart_del;
	vecmul(3,3,cmp.wc_b,(double*)infor.cnb_mat,cmp.wc_n);

	cmp.fc_n[0] = cmp.k1[0]*infor.vel_n[0];
	cmp.fc_n[1] = cmp.k1[0]*infor.vel_n[1];
	vecmul(3,3,cmp.fc_b,(double*)infor.cnb_mat,cmp.fc_n);
}
/**
* @brief  cmp2_algo 罗经法方位精对准
* @param  cmp       罗经法参数
*         quart_del 算法解算周期
* @retval None
*/
void cmp2_algo(COMPALIGN &cmp,double quart_del)
{
	cmp.wc_n[0] = -infor.vel_n[1]*(1+cmp.k2[1])/RE-cmp.wc_n[2]*quart_del*WIE*cos(infor.pos[0]);
	cmp.wc_n[1] =   infor.vel_n[0]*(1+cmp.k2[1])/RE + infor.vel_n[0]*cmp.k2[2]*quart_del;  
	cmp.wc_n[2] = (-infor.vel_n[1]*cmp.k2[4]*quart_del/(WIE*cos(infor.pos[0]))+cmp.wc_n[2]) / (1+cmp.k2[3]*quart_del);
	vecmul(3,3,cmp.wc_b,(double*)infor.cnb_mat,cmp.wc_n);

	cmp.fc_n[0] = cmp.k2[0]*infor.vel_n[0];
	cmp.fc_n[1] = cmp.k2[0]*infor.vel_n[1];
	vecmul(3,3,cmp.fc_b,(double*)infor.cnb_mat,cmp.fc_n);
} 
/**
* @brief  fine_cmp 罗经法精对准 静基座
* @param  None
* @retval None
*/
void fine_cmps(void)
{
	int i;
	if(sysc.cnt_s>=sysc.coarse_time&& sysc.cnt_s<=(sysc.coarse_time+sysc.fine_level))
	{
		cmp1_algo(cmp,sysc.Ts);
	}
	else if(sysc.cnt_s>(sysc.coarse_time+sysc.fine_level) && sysc.cnt_s<=sysc.algn_time)
	{ 
		 cmp2_algo(cmp,sysc.Ts);
 	}
 
	if(sysc.cnt_s==sysc.algn_time)
	{
	    sysc.f_fine_over = TRUE;                        // 精对准结束
		for(i=0;i<3;i++)
		{
			cmp.wc_b[i] = 0.0;
			cmp.fc_b[i] = 0.0;
		}
	}
}
#pragma endregion compass method


/**
* @brief  fine_adrc 自抗扰对准 
* @param  None
* @retval None
*/
void fine_adrc(void)
{

	int i=0;
	if(sysc.cnt_s==sysc.algn_time)
	{
		sysc.f_fine_over = TRUE;                        // 精对准结束
		for(i=0;i<3;i++)
		{
			cmp.wc_b[i] = 0.0;
			cmp.fc_b[i] = 0.0;
		}
	}
	if(sysc.cnt_s>=sysc.coarse_time&&sysc.cnt_s<sysc.algn_time)
	{
		adrc.y1e=infor.vel_n[0];
		adrc.y1n=infor.vel_n[1];

		double u0e=adrc.p1*fal(-adrc.z1e,0.5,adrc.h2)+adrc.p2*fal(-adrc.z2e,1.1,adrc.h2);
		if (sysc.cnt_s>sysc.coarse_time+8)
		   if (abs(u0e)>0.2)
			  u0e=0.2*sign(u0e);

		adrc.ue=(u0e-adrc.z3e)/adrc.b0;
		adrc.ufe=adrc.pf*fal(-adrc.z1e,0.5,adrc.h2);

		double ee=adrc.z1e-adrc.y1e;
		adrc.z1e=adrc.z1e+(adrc.z2e-adrc.b01*ee)*adrc.h;
		adrc.z2e=adrc.z2e+(adrc.z3e-adrc.b02*fal(ee,0.5,adrc.delta)+adrc.ue*adrc.b0)*adrc.h;
		adrc.z3e=adrc.z3e-adrc.b03*fal(ee,0.25,adrc.delta)*adrc.h;

	
		double u0n=adrc.p1*fal(-adrc.z1n,0.5,adrc.h2)+adrc.p2*fal(-adrc.z2n,1.1,adrc.h2);
		if (sysc.cnt_s>sysc.coarse_time+8)
		   if (abs(u0n)>0.2)
			  u0n=0.2*sign(u0n);

		adrc.un=(u0n-adrc.z3n)/adrc.b0;
		adrc.ufn=adrc.pf*fal(-adrc.z1n,0.5,adrc.h2);
		adrc.uu=(adrc.p11*fal(-adrc.z1n,0.5,adrc.h2)+adrc.p12*fal(-adrc.z2n,1.1,adrc.h2)+adrc.p13*fal(-adrc.z3n,1.8,adrc.h2)-adrc.z3n+infor.g/RE*adrc.z1n)/adrc.b0;
		
		if(sysc.cnt_s<(sysc.coarse_time+18)&&sysc.cnt_s>=sysc.coarse_time)//<15s
		{
			adrc.uu=0;
			adrc.un=adrc.un-adrc.z2n;
		}
		else
			adrc.ufn=0;

		if(sysc.cnt_s>=(sysc.coarse_time+150)&&sysc.cnt_s<(sysc.coarse_time+300))//时变
			adrc.azi5=0.3/30000*(sysc.data_cnt-sysc.coarse_time*200)+0.1;
		

		if(sysc.cnt_s>=(sysc.coarse_time+18) && sysc.cnt_s<sysc.algn_time)//>15s
		{	
			adrc.un=fal(-adrc.z1n,0.5,adrc.h2)*(1+adrc.azi1)/RE*infor.g;
			cmp.wc_n[2] =(fal(-adrc.z1n,0.5,adrc.h2)*(-adrc.azi2)*sysc.Ts/WIE/cos(infor.pos[0])+cmp.wc_n[2])/(1+adrc.azi3*sysc.Ts);
			adrc.ufn=adrc.azi4*fal(-adrc.z1n,0.5,adrc.h2);
		}
		else 
			cmp.wc_n[2] =0;

		double en=adrc.z1n-adrc.y1n;
		adrc.z1n=adrc.z1n+(adrc.z2n-adrc.b11*en)*adrc.h;
		adrc.z2n=adrc.z2n+(adrc.z3n-adrc.b12*fal(en,0.5,adrc.delta)+adrc.un*adrc.b0)*adrc.h;
		adrc.z3n=adrc.z3n+(adrc.z4n-adrc.b13*fal(en,0.25,adrc.delta)-infor.g/RE*adrc.z2n+adrc.uu*adrc.b0)*adrc.h;
		adrc.z4n=adrc.z4n-adrc.b14*fal(en,0.125,adrc.delta)*adrc.h;

		cmp.wc_n[0] = adrc.un/infor.g;
		cmp.wc_n[1] = -adrc.ue/infor.g;		
	
		vecmul(3,3,cmp.wc_b,(double*)infor.cnb_mat,cmp.wc_n);

		cmp.fc_n[0] = -adrc.ufe;
		cmp.fc_n[1] = -adrc.ufn;
		vecmul(3,3,cmp.fc_b,(double*)infor.cnb_mat,cmp.fc_n);
	}
}

#pragma region yucia_Kalman
//@brief  根据puresins结构体参数求取15维F阵
void F_matrix_15(SYS_ELEMENT temp_infor,double F_15[15][15])
{
	int		i, j;
	double	w0[8], w1[3], w2[3];
	double temp = (1 - E2* sin(temp_infor.pos[0])* sin(temp_infor.pos[0]));
	double RMh = RE*(1 - E2) / sqrt(temp*temp*temp) + temp_infor.pos[2];
	double RNh = RE / sqrt(1 - E2* sin(temp_infor.pos[0])* sin(temp_infor.pos[0])) + temp_infor.pos[2];

	w0[0] = 2 * WIE  * (temp_infor.vel_n[1] * cos(temp_infor.pos[0]) + temp_infor.vel_n[2] * sin(temp_infor.pos[0])) + temp_infor.vel_n[1] * temp_infor.vel_n[0] / cos(temp_infor.pos[0]) / cos(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);
	w0[1] = (2 * WIE * cos(temp_infor.pos[0]) + temp_infor.vel_n[0] / cos(temp_infor.pos[0]) / cos(temp_infor.pos[0]) / (RNh + temp_infor.pos[2])) * temp_infor.vel_n[0];
	w0[2] = WIE * cos(temp_infor.pos[0]) + temp_infor.vel_n[0] / cos(temp_infor.pos[0]) / cos(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);
	w0[3] = temp_infor.vel_n[0] * tan(temp_infor.pos[0]) / cos(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);
	w0[4] = WIE * sin(temp_infor.pos[0]);
	w0[5] = temp_infor.vel_n[0] * (temp_infor.vel_n[2] - temp_infor.vel_n[1] * tan(temp_infor.pos[0])) / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);
	w0[6] = temp_infor.vel_n[1] * temp_infor.vel_n[2] / (RMh + temp_infor.pos[2]) / (RMh + temp_infor.pos[2]) + temp_infor.vel_n[0] * temp_infor.vel_n[0] * tan(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);
	w0[7] = temp_infor.vel_n[1] * temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]) / (RMh + temp_infor.pos[2]) + temp_infor.vel_n[0] * temp_infor.vel_n[0] / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);

	w1[0] = -temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]);
	w1[1] = 2 * WIE * cos(temp_infor.pos[0]) + temp_infor.vel_n[0] / (RNh + temp_infor.pos[2]);
	w1[2] = 2 * WIE * sin(temp_infor.pos[0]) + temp_infor.vel_n[0] * tan(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);

	w2[0] = -temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]);
	w2[1] = WIE * cos(temp_infor.pos[0]) + temp_infor.vel_n[0] / (RNh + temp_infor.pos[2]);
	w2[2] = WIE * sin(temp_infor.pos[0]) + temp_infor.vel_n[0] * tan(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);

	for (i = 0; i < 15; i++)
		for (j = 0; j < 15; j++)
			F_15[i][j] = 0.0;

	F_15[0][0] = (temp_infor.vel_n[1] * tan(temp_infor.pos[0]) - temp_infor.vel_n[2]) / (RNh + temp_infor.pos[2]);
	F_15[0][1] = w1[2];
	F_15[0][2] = -w1[1];
	F_15[0][4] = -temp_infor.acce_n[2];
	F_15[0][5] = temp_infor.acce_n[1];
	F_15[0][6] = w0[0];
	F_15[0][8] = w0[5];
	F_15[0][9] = temp_infor.cbn_mat[0][0];
	F_15[0][10] = temp_infor.cbn_mat[0][1];
	F_15[0][11] = temp_infor.cbn_mat[0][2];

	F_15[1][0] = -w1[2];
	F_15[1][1] = -temp_infor.vel_n[2] / (RMh + temp_infor.pos[2]);
	F_15[1][2] = -temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]);
	F_15[1][3] = temp_infor.acce_n[2];
	F_15[1][5] = -temp_infor.acce_n[0];
	F_15[1][6] = -w0[1];
	F_15[1][8] = w0[6];
	F_15[1][9] = temp_infor.cbn_mat[1][0];
	F_15[1][10] = temp_infor.cbn_mat[1][1];
	F_15[1][11] = temp_infor.cbn_mat[1][2];

	F_15[2][0] = 2 * w2[1];
	F_15[2][1] = 2 * temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]);
	F_15[2][3] = -temp_infor.acce_n[1];
	F_15[2][4] = temp_infor.acce_n[0];
	F_15[2][6] = -2 * temp_infor.vel_n[0] * w0[4];
	F_15[2][8] = -w0[7];
	F_15[2][9] = temp_infor.cbn_mat[2][0];
	F_15[2][10] = temp_infor.cbn_mat[2][1];
	F_15[2][11] = temp_infor.cbn_mat[2][2];

	F_15[3][1] = -1 / (RMh + temp_infor.pos[2]);
	F_15[3][4] = w2[2];
	F_15[3][5] = -w2[1];
	F_15[3][8] = temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]) / (RMh + temp_infor.pos[2]);
	F_15[3][12] = -temp_infor.cbn_mat[0][0];
	F_15[3][13] = -temp_infor.cbn_mat[0][1];
	F_15[3][14] = -temp_infor.cbn_mat[0][2];

	F_15[4][0] = 1 / (RNh + temp_infor.pos[2]);
	F_15[4][3] = -w2[2];
	F_15[4][5] = w2[0];
	F_15[4][6] = -w0[4];
	F_15[4][8] = -temp_infor.vel_n[0] / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);
	F_15[4][12] = -temp_infor.cbn_mat[1][0];
	F_15[4][13] = -temp_infor.cbn_mat[1][1];
	F_15[4][14] = -temp_infor.cbn_mat[1][2];

	F_15[5][0] = tan(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);
	F_15[5][3] = w2[1];
	F_15[5][4] = -w2[0];
	F_15[5][6] = w0[2];
	F_15[5][8] = -temp_infor.vel_n[0] * tan(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);
	F_15[5][12] = -temp_infor.cbn_mat[2][0];
	F_15[5][13] = -temp_infor.cbn_mat[2][1];
	F_15[5][14] = -temp_infor.cbn_mat[2][2];

	F_15[6][1] = 1 / (RMh + temp_infor.pos[2]);
	F_15[6][8] = -temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]) / (RMh + temp_infor.pos[2]);
	F_15[7][0] = 1 / cos(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);
	F_15[7][6] = w0[3];
	F_15[7][8] = -temp_infor.vel_n[0] / cos(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);
	F_15[8][2] = 1;

}

void Kal_update_15_3(SKALMAN_15_3& temp_kal, double kal_Ts)                   //20171115
{
	int		i, j;
	double	unit_matr[15][15], tmp2[3][15], H_transpose[15][3], tmp3[3][3], R_dis[3][3], tmp4[15][3], K_matrix[15][3], tmp1[15][15], tmp5[3], tmp6[15];

	for (i = 0; i < 15; i++)
		for (j = 0; j < 15; j++)
		{
			if (i == j)
				unit_matr[i][j] = 1.;
			else
				unit_matr[i][j] = 0.;
		}
	amamul(15, 15, (double *)temp_kal.P_forecast, (double *)temp_kal.P_matrix, 1);
	amamul(15, 1, (double *)temp_kal.X_forecast, (double *)temp_kal.X_vector, 1);

	//滤波增益 K
	mamul(3, 15, 15, (double *)tmp2, (double *)temp_kal.H_matrix, (double *)temp_kal.P_forecast);
	maturn(3, 15, (double *)H_transpose, (double *)temp_kal.H_matrix);
	mamul(3, 15, 3, (double *)tmp3, (double *)tmp2, (double *)H_transpose);
	amamul(3, 3, (double *)R_dis, (double *)temp_kal.R_measure, kal_Ts);
	maadd(3, 3, (double *)tmp3, (double *)tmp3, (double *)R_dis);
	mainv(3, (double *)tmp3);//Pzz^-1

	mamul(15, 15, 3, (double *)tmp4, (double *)temp_kal.P_forecast, (double *)H_transpose);
	mamul(15, 3, 3, (double *)K_matrix, (double *)tmp4, (double *)tmp3);

	//更新 P
	mamul(15, 3, 15, (double *)tmp1, (double *)K_matrix, (double *)temp_kal.H_matrix);
	masub(15, 15, (double *)tmp1, (double *)unit_matr, (double *)tmp1);
	mamul(15, 15, 15, (double *)temp_kal.P_matrix, (double *)tmp1, (double *)temp_kal.P_forecast);

	//更新 X
	vecmul(3, 15, tmp5, (double *)temp_kal.H_matrix, temp_kal.X_forecast);
	vecsub(3, tmp5, temp_kal.Mea_vector, tmp5);
	vecmul(15, 3, tmp6, (double *)K_matrix, tmp5);
	vecadd(15, temp_kal.X_vector, temp_kal.X_forecast, tmp6);
}
void Kal_forecast_15(SKALMAN_15_3& temp_kal, double fil_del, double F_15[15][15])              //20171115
{
	int		i, j;
	double	unit_matr[15][15], tmp1[15][15], state_transpose[15][15], Q_dis[15][15];

	for (i = 0; i < 15; i++)
		for (j = 0; j < 15; j++)
		{
			if (i == j)
				unit_matr[i][j] = 1.;
			else
				unit_matr[i][j] = 0.;
		}

	//一步预测 X,只预测时，预测的X值即认为是X估计值
	amamul(15, 15, (double *)F_15, (double *)F_15, fil_del);
	maadd(15, 15, (double *)temp_kal.state_dis, (double *)unit_matr, (double *)F_15);
	vecmul(15, 15, temp_kal.X_vector, (double *)temp_kal.state_dis, temp_kal.X_vector);

	//离散化 Q
	mamul(15, 15, 15, (double *)tmp1, (double *)temp_kal.state_dis, (double *)temp_kal.Q_state);
	maturn(15, 15, (double *)state_transpose, (double *)temp_kal.state_dis);
	mamul(15, 15, 15, (double *)tmp1, (double *)tmp1, (double *)state_transpose);
	maadd(15, 15, (double *)Q_dis, (double *)temp_kal.Q_state, (double *)tmp1);
	amamul(15, 15, (double *)Q_dis, (double *)Q_dis, fil_del / 2.0);

	//预测 P
	mamul(15, 15, 15, (double *)tmp1, (double *)temp_kal.state_dis, (double *)temp_kal.P_matrix);
	mamul(15, 15, 15, (double *)tmp1, (double *)tmp1, (double *)state_transpose);
	maadd(15, 15, (double *)temp_kal.P_matrix, (double *)tmp1, (double *)Q_dis);
}
//传入参数为参考位置。
void fine_yucia(SKALMAN_15_3& temp_kal, double observer[3],char mode)
{
	int i = 0;
	double F_15[15][15] = { 0 };
	if (sysc.cnt_s == sysc.algn_time)
	{
		sysc.f_fine_over = TRUE;                        // 精对准结束
	}
	if (sysc.cnt_s >= sysc.coarse_time&&sysc.cnt_s < sysc.algn_time)
	{
		F_matrix_15(infor,F_15);                       //20171115   通过当前pureins结构体参数计算15维F阵给F_15
		Kal_forecast_15(temp_kal,sysc.Ts, F_15);                   //20171115   15维一步预测通用算法 适用于所有 SKALMAN_15_3结构体
		if (1 == sysc.data_cnt%(sysc.Fs/ sysc.Kal_fr))
		{		

			vecsub(3, temp_kal.Mea_vector, infor.pos, observer);
			Kal_update_15_3(temp_kal, sysc.Kal_fr);
			if (sysc.cnt_s >= sysc.coarse_time + 20)//20s之后开始校正
			{
				vecsub(3, infor.vel_n, infor.vel_n, temp_kal.X_vector);
				vecsub(3, infor.pos, infor.pos, temp_kal.X_vector+6);
				double cnn[3][3] = { 0.0 };
				X2cnn(cnn, temp_kal.X_vector + 3);
				mamul(3, 3, 3, (double *)infor.cnb_mat, (double *)infor.cnb_mat, (double *)cnn);
				maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
				cnb2ang(infor.cnb_mat, infor.att_angle);
				cnb2q(infor.cnb_mat, infor.quart);
				for ( i = 0; i < 9; i++)
					temp_kal.X_vector[i] = 0.0;
				for (i = 0; i < 3; i++)
				{
					infor.gyro_bias_esti[i] = temp_kal.X_vector[12 + i] *180/3.14*3600;
					infor.acce_bias_esti[i] = temp_kal.X_vector[9 + i]  *1000000/9.78;
				}
			}
		}
	}
}
//1,没有时间限制；2，输入的观测量是差值
void navi_Kal_15_3(SKALMAN_15_3& temp_kal, double observer[3], char mode)
{
	int i = 0;
	double F_15[15][15] = { 0 };

	F_matrix_15(infor, F_15);                       //20171115   通过当前pureins结构体参数计算15维F阵给F_15
	Kal_forecast_15(temp_kal, sysc.Ts, F_15);                   //20171115   15维一步预测通用算法 适用于所有 SKALMAN_15_3结构体
	if (0 == sysc.data_cnt % (sysc.Fs / sysc.Kal_fr))
	{
		avecmul(3,temp_kal.Mea_vector, observer, 1.0);
		Kal_update_15_3(temp_kal, 1);
		if (sysc.cnt_s >= sysc.algn_time + 20)//20s之后开始校正
		{
			vecsub(3, infor.vel_n, infor.vel_n, temp_kal.X_vector);
			vecsub(3, infor.pos, infor.pos, temp_kal.X_vector + 6);
			double cnn[3][3] = { 0.0 };
			X2cnn(cnn, temp_kal.X_vector + 3);
			mamul(3, 3, 3, (double *)infor.cnb_mat, (double *)infor.cnb_mat, (double *)cnn);
			maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
			cnb2ang(infor.cnb_mat, infor.att_angle);
			cnb2q(infor.cnb_mat, infor.quart);
			for (i = 0; i < 9; i++)
				temp_kal.X_vector[i] = 0.0;
			for (i = 0; i < 3; i++)
			{
				infor.gyro_bias_esti[i] = temp_kal.X_vector[12 + i] * 180 / 3.14 * 3600;
				infor.acce_bias_esti[i] = temp_kal.X_vector[9 + i] * 1000000 / 9.78;
			}
		}
	}
	
}
#pragma endregion DP yucia_Kalman