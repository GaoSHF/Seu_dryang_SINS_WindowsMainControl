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

	F_15[1][0] = -(2 * w1[2] - 2 * WIE * sin(temp_infor.pos[0]));//-w1[2];  这项好像错了
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

//@brief  根据puresins结构体参数求取16维F阵
void F_matrix_16(SYS_ELEMENT temp_infor, double F_16[16][16])
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

	for (i = 0; i < 16; i++)
	for (j = 0; j < 16; j++)
		F_16[i][j] = 0.0;

	F_16[0][0] = (temp_infor.vel_n[1] * tan(temp_infor.pos[0]) - temp_infor.vel_n[2]) / (RNh + temp_infor.pos[2]);
	F_16[0][1] = w1[2];
	F_16[0][2] = -w1[1];
	F_16[0][4] = -temp_infor.acce_n[2];
	F_16[0][5] = temp_infor.acce_n[1];
	F_16[0][6] = w0[0];
	F_16[0][8] = w0[5];
	F_16[0][9] = temp_infor.cbn_mat[0][0];
	F_16[0][10] = temp_infor.cbn_mat[0][1];
	F_16[0][11] = temp_infor.cbn_mat[0][2];

	F_16[1][0] = -w1[2];
	F_16[1][1] = -temp_infor.vel_n[2] / (RMh + temp_infor.pos[2]);
	F_16[1][2] = -temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]);
	F_16[1][3] = temp_infor.acce_n[2];
	F_16[1][5] = -temp_infor.acce_n[0];
	F_16[1][6] = -w0[1];
	F_16[1][8] = w0[6];
	F_16[1][9] = temp_infor.cbn_mat[1][0];
	F_16[1][10] = temp_infor.cbn_mat[1][1];
	F_16[1][11] = temp_infor.cbn_mat[1][2];

	F_16[2][0] = 2 * w2[1];
	F_16[2][1] = 2 * temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]);
	F_16[2][3] = -temp_infor.acce_n[1];
	F_16[2][4] = temp_infor.acce_n[0];
	F_16[2][6] = -2 * temp_infor.vel_n[0] * w0[4];
	F_16[2][8] = -w0[7];
	F_16[2][9] = temp_infor.cbn_mat[2][0];
	F_16[2][10] = temp_infor.cbn_mat[2][1];
	F_16[2][11] = temp_infor.cbn_mat[2][2];

	F_16[3][1] = -1 / (RMh + temp_infor.pos[2]);
	F_16[3][4] = w2[2];
	F_16[3][5] = -w2[1];
	F_16[3][8] = temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]) / (RMh + temp_infor.pos[2]);
	F_16[3][12] = -temp_infor.cbn_mat[0][0];
	F_16[3][13] = -temp_infor.cbn_mat[0][1];
	F_16[3][14] = -temp_infor.cbn_mat[0][2];

	F_16[4][0] = 1 / (RNh + temp_infor.pos[2]);
	F_16[4][3] = -w2[2];
	F_16[4][5] = w2[0];
	F_16[4][6] = -w0[4];
	F_16[4][8] = -temp_infor.vel_n[0] / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);
	F_16[4][12] = -temp_infor.cbn_mat[1][0];
	F_16[4][13] = -temp_infor.cbn_mat[1][1];
	F_16[4][14] = -temp_infor.cbn_mat[1][2];

	F_16[5][0] = tan(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);
	F_16[5][3] = w2[1];
	F_16[5][4] = -w2[0];
	F_16[5][6] = w0[2];
	F_16[5][8] = -temp_infor.vel_n[0] * tan(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);
	F_16[5][12] = -temp_infor.cbn_mat[2][0];
	F_16[5][13] = -temp_infor.cbn_mat[2][1];
	F_16[5][14] = -temp_infor.cbn_mat[2][2];

	F_16[6][1] = 1 / (RMh + temp_infor.pos[2]);
	F_16[6][8] = -temp_infor.vel_n[1] / (RMh + temp_infor.pos[2]) / (RMh + temp_infor.pos[2]);
	F_16[7][0] = 1 / cos(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]);
	F_16[7][6] = w0[3];
	F_16[7][8] = -temp_infor.vel_n[0] / cos(temp_infor.pos[0]) / (RNh + temp_infor.pos[2]) / (RNh + temp_infor.pos[2]);
	F_16[8][2] = 1;

}
//@brief  根据puresins结构体参数求取16维F阵
void F_matrix_16_transverse(SYS_ELEMENTtransverse temp_infor, double F_16[16][16])//20180319
{
	int		i, j;
	double	cm, sm, cl, sl, tl, secl, secl2;
	double BB[3];

	cm = cos(temp_infor.longi_S); sm = sin(temp_infor.longi_S);
	cl = cos(temp_infor.lati_S);  sl = sin(temp_infor.lati_S); tl = tan(temp_infor.lati_S);
	secl = 1 / cl; secl2 = secl*secl;
	BB[0] = temp_infor.wie_s[0] + temp_infor.wis_s[0];
	BB[1] = temp_infor.wie_s[1] + temp_infor.wis_s[1];
	BB[2] = temp_infor.wie_s[2] + temp_infor.wis_s[2];

	for (i = 0; i < 16; i++)
	for (j = 0; j < 16; j++)
		F_16[i][j] = 0.0;

	F_16[0][0] = temp_infor.Rx_1*(-temp_infor.vel_S[2] + tl*temp_infor.vel_S[1]);
	F_16[0][1] = temp_infor.t_1*(temp_infor.vel_S[2] - tl*temp_infor.vel_S[1])+BB[2];
	F_16[0][2] = -BB[1];
	F_16[0][4] = -temp_infor.acce_S[2];
	F_16[0][5] = temp_infor.acce_S[1];
	F_16[0][6] = 2 * WIE*cl*cm*temp_infor.vel_S[2] - 2 * WIE*sl*cm*temp_infor.vel_S[1] + temp_infor.Rx_1*secl2*temp_infor.vel_S[0] * temp_infor.vel_S[1] - temp_infor.t_1*secl2*temp_infor.vel_S[1] * temp_infor.vel_S[1];
	F_16[0][7] = -2 * WIE*sl*sm*temp_infor.vel_S[2] - 2 * WIE*cl*sm*temp_infor.vel_S[1];
	F_16[0][9] = temp_infor.cbs_mat[0][0];
	F_16[0][10] = temp_infor.cbs_mat[0][1];
	F_16[0][11] = temp_infor.cbs_mat[0][2];

	F_16[1][0] = temp_infor.t_1*temp_infor.vel_S[2] - temp_infor.Rx_1*tl*temp_infor.vel_S[0]-BB[2];
	F_16[1][1] = -temp_infor.Ry_1*temp_infor.vel_S[2] + temp_infor.t_1*tl*temp_infor.vel_S[0];
	F_16[1][2] = BB[0];
	F_16[1][3] = temp_infor.acce_S[2];
	F_16[1][5] = -temp_infor.acce_S[0];
	F_16[1][6] = 2 * WIE*sl*cm*temp_infor.vel_S[0] - temp_infor.Rx_1*secl2*temp_infor.vel_S[0]*temp_infor.vel_S[0] + temp_infor.t_1*secl2*temp_infor.vel_S[1] * temp_infor.vel_S[0];
	F_16[1][7] = -2 * WIE*cm*temp_infor.vel_S[2] + 2 * WIE*cl*sm*temp_infor.vel_S[0];
	F_16[1][9] = temp_infor.cbs_mat[1][0];
	F_16[1][10] = temp_infor.cbs_mat[1][1];
	F_16[1][11] = temp_infor.cbs_mat[1][2];

	F_16[2][0] = -temp_infor.t_1*temp_infor.vel_S[1] + temp_infor.Rx_1*temp_infor.vel_S[0]+BB[1];
	F_16[2][1] = temp_infor.Ry_1*temp_infor.vel_S[1] - temp_infor.t_1*temp_infor.vel_S[0]-BB[0];
	F_16[2][3] = -temp_infor.acce_S[1];
	F_16[2][4] = temp_infor.acce_S[0];
	F_16[2][6] = -2 * WIE*cl*cm*temp_infor.vel_S[0];
	F_16[2][7] = 2 * WIE*cm*temp_infor.vel_S[1] + 2 * WIE*sl*sm*temp_infor.vel_S[0];
	F_16[2][9] = temp_infor.cbs_mat[2][0];
	F_16[2][10] = temp_infor.cbs_mat[2][1];
	F_16[2][11] = temp_infor.cbs_mat[2][2];

	F_16[3][0] = temp_infor.t_1;
	F_16[3][1] = -temp_infor.Ry_1;
	F_16[3][4] = temp_infor.wis_s[2];
	F_16[3][5] = -temp_infor.wis_s[1];
	F_16[3][7] = -WIE*cm;
	F_16[3][12] = -temp_infor.cbs_mat[0][0];
	F_16[3][13] = -temp_infor.cbs_mat[0][1];
	F_16[3][14] = -temp_infor.cbs_mat[0][2];

	F_16[4][0] = temp_infor.Rx_1;
	F_16[4][1] = -temp_infor.t_1;
	F_16[4][3] = -temp_infor.wis_s[2];
	F_16[4][5] = temp_infor.wis_s[0];
	F_16[4][6] = -WIE*cl*cm;
	F_16[4][7] = WIE*sl*sm;
	F_16[4][12] = -temp_infor.cbs_mat[1][0];
	F_16[4][13] = -temp_infor.cbs_mat[1][1];
	F_16[4][14] = -temp_infor.cbs_mat[1][2];

	F_16[5][0] = temp_infor.Rx_1*tl;
	F_16[5][1] = -temp_infor.t_1*tl;
	F_16[5][3] = temp_infor.wis_s[1];
	F_16[5][4] = -temp_infor.wis_s[0];
	F_16[5][6] = -WIE*sl*cm + temp_infor.Rx_1*secl2*temp_infor.vel_S[0] - temp_infor.t_1*secl2*temp_infor.vel_S[1];
	F_16[5][7] = -WIE*cl*sm;
	F_16[5][12] = -temp_infor.cbs_mat[2][0];
	F_16[5][13] = -temp_infor.cbs_mat[2][1];
	F_16[5][14] = -temp_infor.cbs_mat[2][2];

	F_16[6][0] = -temp_infor.t_1;
	F_16[6][1] = temp_infor.Ry_1;

	F_16[7][0] = temp_infor.Rx_1*secl;
	F_16[7][1] = -temp_infor.t_1*secl;
	F_16[7][6] = temp_infor.Rx_1*temp_infor.vel_S[0] * tl*secl - temp_infor.t_1*temp_infor.vel_S[1]*tl*secl;
	
	F_16[8][2] = 1;
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
void Kal_update_16_3(SKALMAN_16_3& temp_kal, double kal_Ts)                   //20171128
{
	int		i, j;
	double	unit_matr[16][16], tmp2[3][16], H_transpose[16][3], tmp3[3][3], R_dis[3][3], tmp4[16][3], K_matrix[16][3], tmp1[16][16], tmp5[3], tmp6[16];

	for (i = 0; i < 16; i++)
	for (j = 0; j < 16; j++)
	{
		if (i == j)
			unit_matr[i][j] = 1.;
		else
			unit_matr[i][j] = 0.;
	}
	amamul(16, 16, (double *)temp_kal.P_forecast, (double *)temp_kal.P_matrix, 1);
	amamul(16, 1, (double *)temp_kal.X_forecast, (double *)temp_kal.X_vector, 1);

	//滤波增益 K
	mamul(3, 16, 16, (double *)tmp2, (double *)temp_kal.H_matrix, (double *)temp_kal.P_forecast);
	maturn(3, 16, (double *)H_transpose, (double *)temp_kal.H_matrix);
	mamul(3, 16, 3, (double *)tmp3, (double *)tmp2, (double *)H_transpose);
	amamul(3, 3, (double *)R_dis, (double *)temp_kal.R_measure, kal_Ts);
	maadd(3, 3, (double *)tmp3, (double *)tmp3, (double *)R_dis);
	mainv(3, (double *)tmp3);//Pzz^-1

	mamul(16, 16, 3, (double *)tmp4, (double *)temp_kal.P_forecast, (double *)H_transpose);
	mamul(16, 3, 3, (double *)K_matrix, (double *)tmp4, (double *)tmp3);

	//更新 P
	mamul(16, 3, 16, (double *)tmp1, (double *)K_matrix, (double *)temp_kal.H_matrix);
	masub(16, 16, (double *)tmp1, (double *)unit_matr, (double *)tmp1);
	mamul(16, 16, 16, (double *)temp_kal.P_matrix, (double *)tmp1, (double *)temp_kal.P_forecast);

	//更新 X
	vecmul(3, 16, tmp5, (double *)temp_kal.H_matrix, temp_kal.X_forecast);
	vecsub(3, tmp5, temp_kal.Mea_vector, tmp5);
	vecmul(16, 3, tmp6, (double *)K_matrix, tmp5);
	vecadd(16, temp_kal.X_vector, temp_kal.X_forecast, tmp6);
}



void Kal_update_15_1(SKALMAN_15_1& temp_kal, double kal_Ts)                   //20180116
{
	int		i, j;
	double	unit_matr[15][15], tmp2[1][15], H_transpose[15][1], tmp3[1][1], R_dis[1][1], tmp4[15][1], K_matrix[15][1], tmp1[15][15], tmp5[1], tmp6[15];

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
	mamul(1, 15, 15, (double *)tmp2, (double *)temp_kal.H_matrix, (double *)temp_kal.P_forecast);
	maturn(1, 15, (double *)H_transpose, (double *)temp_kal.H_matrix);
	mamul(1, 15, 1, (double *)tmp3, (double *)tmp2, (double *)H_transpose);
	amamul(1, 1, (double *)R_dis, (double *)temp_kal.R_measure, kal_Ts);
	maadd(1, 1, (double *)tmp3, (double *)tmp3, (double *)R_dis);
	mainv(1, (double *)tmp3);//Pzz^-1

	mamul(15, 15, 1, (double *)tmp4, (double *)temp_kal.P_forecast, (double *)H_transpose);
	mamul(15, 1, 1, (double *)K_matrix, (double *)tmp4, (double *)tmp3);

	//更新 P
	mamul(15, 1, 15, (double *)tmp1, (double *)K_matrix, (double *)temp_kal.H_matrix);
	masub(15, 15, (double *)tmp1, (double *)unit_matr, (double *)tmp1);
	mamul(15, 15, 15, (double *)temp_kal.P_matrix, (double *)tmp1, (double *)temp_kal.P_forecast);

	//更新 X
	vecmul(1, 15, tmp5, (double *)temp_kal.H_matrix, temp_kal.X_forecast);
	vecsub(1, tmp5, temp_kal.Mea_vector, tmp5);
	vecmul(15, 1, tmp6, (double *)K_matrix, tmp5);
	vecadd(15, temp_kal.X_vector, temp_kal.X_forecast, tmp6);
}

void Kal_update_19_6(SKALMAN_19_6& temp_kal, double kal_Ts)                   //20180116
{
	int		i, j;
	int   m = 6; //Zk维数
	double	unit_matr[19][19], tmp2[6][19], H_transpose[19][6], tmp3[6][6], R_dis[6][6], tmp4[19][6], K_matrix[19][6], tmp1[19][19], tmp5[6], tmp6[19];
	double tmp5_transpose[1][6],Pzz[6][6];
	double a, b, c, d;

	for (i = 0; i < 19; i++)
	for (j = 0; j < 19; j++)
	{
		if (i == j)
			unit_matr[i][j] = 1.;
		else
			unit_matr[i][j] = 0.;
	}
	amamul(19, 19, (double *)temp_kal.P_forecast, (double *)temp_kal.P_matrix, 1);
	amamul(19, 1, (double *)temp_kal.X_forecast, (double *)temp_kal.X_vector, 1);

	//滤波增益 K
	mamul(6, 19, 19, (double *)tmp2, (double *)temp_kal.H_matrix, (double *)temp_kal.P_forecast);
	maturn(6, 19, (double *)H_transpose, (double *)temp_kal.H_matrix);
	mamul(6, 19, 6, (double *)tmp3, (double *)tmp2, (double *)H_transpose);
	amamul(6, 6, (double *)R_dis, (double *)temp_kal.R_measure, kal_Ts);
	maadd(6, 6, (double *)tmp3, (double *)tmp3, (double *)R_dis);//Pzz
	memcpy(Pzz, tmp3, sizeof(tmp3));
	mainv(6, (double *)tmp3);//Pzz^-1

	mamul(19, 19, 6, (double *)tmp4, (double *)temp_kal.P_forecast, (double *)H_transpose);
	mamul(19, 6, 6, (double *)K_matrix, (double *)tmp4, (double *)tmp3);

	//更新 P
	mamul(19, 6, 19, (double *)tmp1, (double *)K_matrix, (double *)temp_kal.H_matrix);
	masub(19, 19, (double *)tmp1, (double *)unit_matr, (double *)tmp1);
	mamul(19, 19, 19, (double *)temp_kal.P_matrix, (double *)tmp1, (double *)temp_kal.P_forecast);

	//更新 X
	vecmul(6, 19, tmp5, (double *)temp_kal.H_matrix, temp_kal.X_forecast);
	vecsub(6, tmp5, temp_kal.Mea_vector, tmp5);//rk
	vecmul(19, 6, tmp6, (double *)K_matrix, tmp5);
	vecadd(19, temp_kal.X_vector, temp_kal.X_forecast, tmp6);

	//概率
	mamul(6, 6, 1, (double *)tmp3, (double *)tmp3, (double *)tmp5);
	maturn(6, 1, (double *)tmp5_transpose, (double *)tmp5);
	mamul(1, 6, 1, &a, (double *)tmp5_transpose, (double *)tmp3);
	b = det((double *)Pzz, 6);
	c = pow(Exp, -0.5*a); d = pow(2 * PAI, m);
	dvlkalman.f = c/ sqrt(d*b);

}
void Kal_robust_update_19_6(SKALMAN_19_6& temp_kal, SDVLCmpDepth& dObserver, double kal_Ts)                   //20180116
{
	int		i, j;
	int   m = 6; //Zk维数
	double	unit_matr[19][19], tmp2[6][19], H_transpose[19][6], tmp3[6][6], R_dis[6][6], tmp4[19][6], K_matrix[19][6], tmp1[19][19], tmp5[6], tmp6[19];
	double tmp5_transpose[1][6], Pzz[6][6], ek_transpose[1][6],tmp7[1][6],lamda[1][1];
	double a, b, c, d;

	for (i = 0; i < 19; i++)
	for (j = 0; j < 19; j++)
	{
		if (i == j)
			unit_matr[i][j] = 1.;
		else
			unit_matr[i][j] = 0.;
	}
	amamul(19, 19, (double *)temp_kal.P_forecast, (double *)temp_kal.P_matrix, 1);
	amamul(19, 1, (double *)temp_kal.X_forecast, (double *)temp_kal.X_vector, 1);
	
	//滤波增益 K
	mamul(6, 19, 19, (double *)tmp2, (double *)temp_kal.H_matrix, (double *)temp_kal.P_forecast); //Hk*Pkk_1
	maturn(6, 19, (double *)H_transpose, (double *)temp_kal.H_matrix);//Hk'
	mamul(6, 19, 6, (double *)tmp3, (double *)tmp2, (double *)H_transpose);//Hk*Pkk_1*Hk'
	amamul(6, 6, (double *)R_dis, (double *)temp_kal.R_measure, kal_Ts);
	maadd(6, 6, (double *)tmp3, (double *)tmp3, (double *)R_dis);//Pzz
	memcpy(Pzz, tmp3, sizeof(tmp3));
	mainv(6, (double *)tmp3);//Pzz^-1

	mamul(19, 19, 6, (double *)tmp4, (double *)temp_kal.P_forecast, (double *)H_transpose);//Pxz=Pkk_1*Hk'
	mamul(19, 6, 6, (double *)K_matrix, (double *)tmp4, (double *)tmp3);//Kk_1 = Pxz*Pzz^-1

	//更新 P
	mamul(19, 6, 19, (double *)tmp1, (double *)K_matrix, (double *)temp_kal.H_matrix);//Kk*Hk
	masub(19, 19, (double *)tmp1, (double *)unit_matr, (double *)tmp1);//(eye(19) - Kk*Hk)
	mamul(19, 19, 19, (double *)temp_kal.P_matrix, (double *)tmp1, (double *)temp_kal.P_forecast);// (eye(19) - Kk*Hk)*Pkk_1

	//更新 X
	vecmul(6, 19, tmp5, (double *)temp_kal.H_matrix, temp_kal.X_forecast);
	vecsub(6, tmp5, temp_kal.Mea_vector, tmp5);//ek
	  //监测野值
	maturn(6, 1, (double *)ek_transpose, (double *)tmp5);//ek'
	mamul(1, 6, 6, (double *)tmp7, (double *)ek_transpose, (double *)tmp3);//ek'*Pzz_1^(-1)
	mamul(1, 6, 1, (double *)lamda, (double *)tmp7, (double *)tmp5);//lamda_A=ek'*Pzz_1^(-1)*ek;
	if (lamda[0][0] >= 100) memset(tmp5, 0, sizeof(tmp5));//令新息为0
	vecmul(19, 6, tmp6, (double *)K_matrix, tmp5);
	vecadd(19, temp_kal.X_vector, temp_kal.X_forecast, tmp6);

	//概率
	mamul(6, 6, 1, (double *)tmp3, (double *)tmp3, (double *)tmp5);
	maturn(6, 1, (double *)tmp5_transpose, (double *)tmp5);
	mamul(1, 6, 1, &a, (double *)tmp5_transpose, (double *)tmp3);
	b = det((double *)Pzz, 6);
	c = pow(Exp, -0.5*a); d = pow(2 * PAI, m);
	dvlkalman.f = c / sqrt(d*b);

	dObserver.lamda = lamda[0][0];
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

void Kal_forecast_16(SKALMAN_16_3& temp_kal, double fil_del, double F_16[16][16])              //20171128
{
	int		i, j;
	double	unit_matr[16][16], tmp1[16][16], state_transpose[16][16], Q_dis[16][16];

	for (i = 0; i < 16; i++)
	for (j = 0; j < 16; j++)
	{
		if (i == j)
			unit_matr[i][j] = 1.;
		else
			unit_matr[i][j] = 0.;
	}

	//一步预测 X,只预测时，预测的X值即认为是X估计值
	amamul(16, 16, (double *)F_16, (double *)F_16, fil_del);
	maadd(16, 16, (double *)temp_kal.state_dis, (double *)unit_matr, (double *)F_16);
	vecmul(16, 16, temp_kal.X_vector, (double *)temp_kal.state_dis, temp_kal.X_vector);

	//离散化 Q
	mamul(16, 16, 16, (double *)tmp1, (double *)temp_kal.state_dis, (double *)temp_kal.Q_state);
	maturn(16, 16, (double *)state_transpose, (double *)temp_kal.state_dis);
	mamul(16, 16, 16, (double *)tmp1, (double *)tmp1, (double *)state_transpose);
	maadd(16, 16, (double *)Q_dis, (double *)temp_kal.Q_state, (double *)tmp1);
	amamul(16, 16, (double *)Q_dis, (double *)Q_dis, fil_del / 2);

	//预测 P
	mamul(16, 16, 16, (double *)tmp1, (double *)temp_kal.state_dis, (double *)temp_kal.P_matrix);
	mamul(16, 16, 16, (double *)tmp1, (double *)tmp1, (double *)state_transpose);
	maadd(16, 16, (double *)temp_kal.P_matrix, (double *)tmp1, (double *)Q_dis);
}

void Kal_forecast_19(SKALMAN_19_6& temp_kal, double fil_del, double F_19[19][19])              //20180116
{
	int		i, j;
	double	unit_matr[19][19], tmp1[19][19], state_transpose[19][19], Q_dis[19][19];

	for (i = 0; i < 19; i++)
	for (j = 0; j < 19; j++)
	{
		if (i == j)
			unit_matr[i][j] = 1.;
		else
			unit_matr[i][j] = 0.;
	}

	//一步预测 X,只预测时，预测的X值即认为是X估计值
	amamul(19, 19, (double *)F_19, (double *)F_19, fil_del);
	maadd(19, 19, (double *)temp_kal.state_dis, (double *)unit_matr, (double *)F_19);
	vecmul(19, 19, temp_kal.X_vector, (double *)temp_kal.state_dis, temp_kal.X_vector);

	//离散化 Q
	mamul(19, 19, 19, (double *)tmp1, (double *)temp_kal.state_dis, (double *)temp_kal.Q_state);
	maturn(19, 19, (double *)state_transpose, (double *)temp_kal.state_dis);
	mamul(19, 19, 19, (double *)tmp1, (double *)tmp1, (double *)state_transpose);
	maadd(19, 19, (double *)Q_dis, (double *)temp_kal.Q_state, (double *)tmp1);
	amamul(19, 19, (double *)Q_dis, (double *)Q_dis, fil_del / 2.0);

	//预测 P
	mamul(19, 19, 19, (double *)tmp1, (double *)temp_kal.state_dis, (double *)temp_kal.P_matrix);
	mamul(19, 19, 19, (double *)tmp1, (double *)tmp1, (double *)state_transpose);
	maadd(19, 19, (double *)temp_kal.P_matrix, (double *)tmp1, (double *)Q_dis);
}

void Kal_forecast_15_1(SKALMAN_15_1& temp_kal, double fil_del, double F_15[15][15])              //20180116
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
void fine_yucia(SKALMAN_15_3& temp_kal, double observer[3], char mode)
{
	int i = 0;
	double F_15[15][15] = { 0 };
	double tempq[4] = { 0 }, tempphi[3] = { 0 };
	double temp_bias[3];
	if (sysc.cnt_s == sysc.algn_time)
	{
		sysc.f_fine_over = TRUE;                        // 精对准结束
		//零偏估计补偿//直接写入注释了此处
		//	memcpy(temp_bias, temp_kal.X_vector + 9, sizeof(temp_bias));
		//	vecadd(3, calipara.bias_acce, temp_bias, calipara.bias_acce);//增量计算	
	}
	if (sysc.cnt_s >= sysc.coarse_time&&sysc.cnt_s < sysc.algn_time)
	{
		F_matrix_15(infor, F_15);                       //20171115   通过当前pureins结构体参数计算15维F阵给F_15
		Kal_forecast_15(temp_kal, sysc.Ts, F_15);                   //20171115   15维一步预测通用算法 适用于所有 SKALMAN_15_3结构体
		if (0 == sysc.data_cnt % (sysc.Fs / sysc.Kal_fr)) //1    //20171212
		{
			if (mode == YA_POS)
				vecsub(3, temp_kal.Mea_vector, infor.pos, observer);
			if (mode == YA_VEL)
				vecsub(3, temp_kal.Mea_vector, infor.vel_n, observer);
			Kal_update_15_3(temp_kal, sysc.Kal_fr);
			if (sysc.cnt_s >= sysc.coarse_time + 20)//20s之后开始校正
			{
				vecsub(3, infor.vel_n, infor.vel_n, temp_kal.X_vector);
				vecsub(3, infor.pos, infor.pos, temp_kal.X_vector + 6);
				memcpy(tempphi, temp_kal.X_vector + 3, sizeof(tempphi));
				rv2q(tempq, tempphi);
				qmul(infor.quart, tempq, infor.quart);
				q2cnb(infor.cnb_mat, infor.quart);
				cnb2ang(infor.cnb_mat, infor.att_angle);
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
}
//1,没有时间限制；2，输入的观测量是差值
void navi_Kal_15_3(SKALMAN_15_3& temp_kal, double observer[3], char mode)
{
	int i = 0;
	double F_15[15][15] = { 0 };
	double tempq[4] = { 0 }, tempphi[3] = { 0 };
	F_matrix_15(infor, F_15);                       //20171115   通过当前pureins结构体参数计算15维F阵给F_15
	Kal_forecast_15(temp_kal, sysc.Ts, F_15);                   //20171115   15维一步预测通用算法 适用于所有 SKALMAN_15_3结构体
	if (0 == sysc.data_cnt % (sysc.Fs / sysc.Kal_fr))
	{
		avecmul(3,temp_kal.Mea_vector, observer, 1.0);
		Kal_update_15_3(temp_kal, sysc.Kal_fr);

		if (sysc.cnt_s >= sysc.algn_time + 20)//20s之后开始校正
		{
			vecsub(3, infor.vel_n, infor.vel_n, temp_kal.X_vector);
			vecsub(3, infor.pos, infor.pos, temp_kal.X_vector + 6);
			memcpy(tempphi, temp_kal.X_vector + 3, sizeof(tempphi));
			rv2q(tempq, tempphi);
			qmul(infor.quart, tempq, infor.quart);
			q2cnb(infor.cnb_mat, infor.quart);
			cnb2ang(infor.cnb_mat, infor.att_angle);
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

void navi_Kal_16_3(SKALMAN_16_3& temp_kal, double observer[3])       //20171128  后期看一下里面的子函数是否可以和15维统一，优化
{
	int i = 0;
	double F_16[16][16] = { 0 };

	F_matrix_16(infor, F_16);
	Kal_forecast_16(temp_kal, sysc.Ts, F_16);
	if (0 == sysc.data_cnt % (sysc.Fs / sysc.Kal_fr))
	{
		vecmul(3, 3, infor.vel_b, (double*)infor.cnb_mat, infor.vel_n);
		vecsub(3, temp_kal.Mea_vector, infor.vel_b, observer);  //Zk

		temp_kal.H_matrix[0][0] = infor.cnb_mat[0][0]; temp_kal.H_matrix[0][1] = infor.cnb_mat[0][1]; temp_kal.H_matrix[0][2] = infor.cnb_mat[0][2];
		temp_kal.H_matrix[1][0] = infor.cnb_mat[1][0]; temp_kal.H_matrix[1][1] = infor.cnb_mat[1][1]; temp_kal.H_matrix[1][2] = infor.cnb_mat[1][2];
		temp_kal.H_matrix[2][0] = infor.cnb_mat[2][0]; temp_kal.H_matrix[2][1] = infor.cnb_mat[2][1]; temp_kal.H_matrix[2][2] = infor.cnb_mat[2][2];
		temp_kal.H_matrix[0][3] = infor.cnb_mat[0][2] * infor.vel_n[1] - infor.cnb_mat[0][1] * infor.vel_n[2];
		temp_kal.H_matrix[0][4] = infor.cnb_mat[0][0] * infor.vel_n[2] - infor.cnb_mat[0][2] * infor.vel_n[0];
		temp_kal.H_matrix[0][5] = infor.cnb_mat[0][1] * infor.vel_n[0] - infor.cnb_mat[0][0] * infor.vel_n[1];
		temp_kal.H_matrix[1][3] = infor.cnb_mat[1][2] * infor.vel_n[1] - infor.cnb_mat[1][1] * infor.vel_n[2];
		temp_kal.H_matrix[1][4] = infor.cnb_mat[1][0] * infor.vel_n[2] - infor.cnb_mat[1][2] * infor.vel_n[0];
		temp_kal.H_matrix[1][5] = infor.cnb_mat[1][1] * infor.vel_n[0] - infor.cnb_mat[1][0] * infor.vel_n[1];
		temp_kal.H_matrix[2][3] = infor.cnb_mat[2][2] * infor.vel_n[1] - infor.cnb_mat[2][1] * infor.vel_n[2];
		temp_kal.H_matrix[2][4] = infor.cnb_mat[2][0] * infor.vel_n[2] - infor.cnb_mat[2][2] * infor.vel_n[0];
		temp_kal.H_matrix[2][5] = infor.cnb_mat[2][1] * infor.vel_n[0] - infor.cnb_mat[2][0] * infor.vel_n[1];
		temp_kal.H_matrix[0][15] = -infor.vel_b[0];
		temp_kal.H_matrix[1][15] = -infor.vel_b[1];
		temp_kal.H_matrix[2][15] = -infor.vel_b[2];

		Kal_update_16_3(temp_kal, 1);
		if (sysc.cnt_s >= sysc.algn_time + 3)//20s之后开始校正
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
void navi_Kal_16_3_transverse(SKALMAN_16_3& temp_kal, double observer[3])       //20171128  后期看一下里面的子函数是否可以和15维统一，优化
{
	int i = 0;
	double F_16[16][16] = { 0 };

	F_matrix_16_transverse(inforS, F_16);
	Kal_forecast_16(temp_kal, sysc.Ts, F_16);
	if (0 == sysc.data_cnt % (sysc.Fs / sysc.Kal_fr))
	{
		vecmul(3, 3, inforS.vel_b, (double*)inforS.csb_mat, inforS.vel_S);
		vecsub(3, temp_kal.Mea_vector, inforS.vel_b, observer);  //Zk

		temp_kal.H_matrix[0][0] = inforS.csb_mat[0][0]; temp_kal.H_matrix[0][1] = inforS.csb_mat[0][1]; temp_kal.H_matrix[0][2] = inforS.csb_mat[0][2];
		temp_kal.H_matrix[1][0] = inforS.csb_mat[1][0]; temp_kal.H_matrix[1][1] = inforS.csb_mat[1][1]; temp_kal.H_matrix[1][2] = inforS.csb_mat[1][2];
		temp_kal.H_matrix[2][0] = inforS.csb_mat[2][0]; temp_kal.H_matrix[2][1] = inforS.csb_mat[2][1]; temp_kal.H_matrix[2][2] = inforS.csb_mat[2][2];
		temp_kal.H_matrix[0][3] = inforS.csb_mat[0][2] * inforS.vel_S[1] - inforS.csb_mat[0][1] * inforS.vel_S[2];
		temp_kal.H_matrix[0][4] = inforS.csb_mat[0][0] * inforS.vel_S[2] - inforS.csb_mat[0][2] * inforS.vel_S[0];
		temp_kal.H_matrix[0][5] = inforS.csb_mat[0][1] * inforS.vel_S[0] - inforS.csb_mat[0][0] * inforS.vel_S[1];
		temp_kal.H_matrix[1][3] = inforS.csb_mat[1][2] * inforS.vel_S[1] - inforS.csb_mat[1][1] * inforS.vel_S[2];
		temp_kal.H_matrix[1][4] = inforS.csb_mat[1][0] * inforS.vel_S[2] - inforS.csb_mat[1][2] * inforS.vel_S[0];
		temp_kal.H_matrix[1][5] = inforS.csb_mat[1][1] * inforS.vel_S[0] - inforS.csb_mat[1][0] * inforS.vel_S[1];
		temp_kal.H_matrix[2][3] = inforS.csb_mat[2][2] * inforS.vel_S[1] - inforS.csb_mat[2][1] * inforS.vel_S[2];
		temp_kal.H_matrix[2][4] = inforS.csb_mat[2][0] * inforS.vel_S[2] - inforS.csb_mat[2][2] * inforS.vel_S[0];
		temp_kal.H_matrix[2][5] = inforS.csb_mat[2][1] * inforS.vel_S[0] - inforS.csb_mat[2][0] * inforS.vel_S[1];
		temp_kal.H_matrix[0][15] = -inforS.vel_b[0];
		temp_kal.H_matrix[1][15] = -inforS.vel_b[1];
		temp_kal.H_matrix[2][15] = -inforS.vel_b[2];

		Kal_update_16_3(temp_kal, 1);
		if (sysc.cnt_s >= sysc.algn_time + 0)//20s之后开始校正
		{
			vecsub(3, inforS.vel_S, inforS.vel_S, temp_kal.X_vector);
			inforS.lati_S = inforS.lati_S - temp_kal.X_vector[6];
			inforS.longi_S = inforS.longi_S - temp_kal.X_vector[7];
			inforS.high_S = inforS.high_S - temp_kal.X_vector[8];
			double cnn[3][3] = { 0.0 };
			X2cnn(cnn, temp_kal.X_vector + 3);
			mamul(3, 3, 3, (double *)inforS.csb_mat, (double *)inforS.csb_mat, (double *)cnn);
			maturn(3, 3, (double*)inforS.cbs_mat, (double*)inforS.csb_mat);
			cnb2ang(inforS.csb_mat, inforS.att_angle_S);
			cnb2q(inforS.csb_mat, inforS.quart_S);
			for (i = 0; i < 9; i++)
				temp_kal.X_vector[i] = 0.0;
			for (i = 0; i < 3; i++)   //这边仅是将陀螺加表估计输出显示，所以给infor进行显示
			{
				infor.gyro_bias_esti[i] = temp_kal.X_vector[12 + i] * 180 / 3.14 * 3600;
				infor.acce_bias_esti[i] = temp_kal.X_vector[9 + i] * 1000000 / 9.78;
			}
		}
	}

}
void initial_mode0_2(SDVLCmpDepth& dObserver)   //20180116
{
	memset(dObserver.R1_esti, 0, sizeof(dObserver.R1_esti));
	dObserver.R1_esti[0][0] = powl(0.1, 2); dObserver.R1_esti[1][1] = powl(0.1, 2);	dObserver.R1_esti[2][2] = powl(0.1, 2);
	dObserver.R1_esti[3][3] = powl(0.1, 2); dObserver.R1_esti[4][4] = powl(0.1, 2);	dObserver.R1_esti[5][5] = powl(0.1, 2);
	memset(dObserver.R2_esti, 0, sizeof(dObserver.R2_esti));
	dObserver.R2_esti[0][0] = powl(0.2, 2); dObserver.R2_esti[1][1] = powl(0.2, 2);	dObserver.R2_esti[2][2] = powl(0.2, 2);
	dObserver.R2_esti[3][3] = powl(0.2, 2); dObserver.R2_esti[4][4] = powl(0.2, 2);	dObserver.R2_esti[5][5] = powl(0.2, 2);
	memset(dObserver.R3_esti, 0, sizeof(dObserver.R3_esti));
	dObserver.R3_esti[0][0] = powl(0.05, 2); dObserver.R3_esti[1][1] = powl(0.05, 2);	dObserver.R3_esti[2][2] = powl(0.05, 2);
	dObserver.R3_esti[3][3] = powl(0.05, 2); dObserver.R3_esti[4][4] = powl(0.05, 2);	dObserver.R3_esti[5][5] = powl(0.05, 2);
	memset(dObserver.R_adaptive, 0, sizeof(dObserver.R_adaptive));
	dObserver.R_adaptive[0][0] = powl(0.1, 2); dObserver.R_adaptive[1][1] = powl(0.1, 2);	dObserver.R_adaptive[2][2] = powl(0.1, 2);
	dObserver.R_adaptive[3][3] = powl(0.1, 2); dObserver.R_adaptive[4][4] = powl(0.1, 2);	dObserver.R_adaptive[5][5] = powl(0.1, 2);
	memset(dObserver.R_A_store, 0, sizeof(dObserver.R_A_store));
	dObserver.k_0 = sysc.cnt_s;
}
void initial_mode1(SDVLCmpDepth& dObserver)   //20180116
{
	memset(dObserver.R1_esti, 0, sizeof(dObserver.R1_esti));
	dObserver.R1_esti[0][0] = powl(10, 2); dObserver.R1_esti[1][1] = powl(10, 2);	dObserver.R1_esti[2][2] = powl(10, 2);
	dObserver.R1_esti[3][3] = powl(0.1, 2); dObserver.R1_esti[4][4] = powl(0.1, 2);	dObserver.R1_esti[5][5] = powl(0.1, 2);
	memset(dObserver.R2_esti, 0, sizeof(dObserver.R2_esti));
	dObserver.R2_esti[0][0] = powl(10, 2); dObserver.R2_esti[1][1] = powl(10, 2);	dObserver.R2_esti[2][2] = powl(10, 2);
	dObserver.R2_esti[3][3] = powl(0.2, 2); dObserver.R2_esti[4][4] = powl(0.2, 2);	dObserver.R2_esti[5][5] = powl(0.2, 2);
	memset(dObserver.R3_esti, 0, sizeof(dObserver.R3_esti));
	dObserver.R3_esti[0][0] = powl(10, 2); dObserver.R3_esti[1][1] = powl(10, 2);	dObserver.R3_esti[2][2] = powl(10, 2);
	dObserver.R3_esti[3][3] = powl(0.05, 2); dObserver.R3_esti[4][4] = powl(0.05, 2);	dObserver.R3_esti[5][5] = powl(0.05, 2);
	memset(dObserver.R_adaptive, 0, sizeof(dObserver.R_adaptive));
	dObserver.R_adaptive[0][0] = powl(0.1, 2); dObserver.R_adaptive[1][1] = powl(0.1, 2);	dObserver.R_adaptive[2][2] = powl(0.1, 2);
	dObserver.R_adaptive[3][3] = powl(0.1, 2); dObserver.R_adaptive[4][4] = powl(0.1, 2);	dObserver.R_adaptive[5][5] = powl(0.1, 2);
	memset(dObserver.R_A_store, 0, sizeof(dObserver.R_A_store));
	dObserver.k_0 = sysc.cnt_s;
}
void initial_mode3to0(SKALMAN_19_6&dvlkalman, SKALMAN_15_1& cmpkalman, SKALMAN_15_1&depkalman,double deltaK)
{
	memset(dvlkalman.P_matrix, 0, sizeof(dvlkalman.P_matrix));
	dvlkalman.P_matrix[0][0] = powl(0.1, 2); dvlkalman.P_matrix[1][1] = dvlkalman.P_matrix[0][0]; dvlkalman.P_matrix[2][2] = dvlkalman.P_matrix[0][0];
	dvlkalman.P_matrix[3][3] = powl(0.01 * D2R, 2); dvlkalman.P_matrix[4][4] = dvlkalman.P_matrix[3][3]; dvlkalman.P_matrix[5][5] = powl(0.1 * D2R, 2);
	dvlkalman.P_matrix[6][6] = powl(10.0 / RE, 2); dvlkalman.P_matrix[7][7] = dvlkalman.P_matrix[6][6]; dvlkalman.P_matrix[8][8] = powl(10, 2);
	dvlkalman.P_matrix[9][9] = powl(100 * ug, 2); dvlkalman.P_matrix[10][10] = dvlkalman.P_matrix[9][9]; dvlkalman.P_matrix[11][11] = dvlkalman.P_matrix[9][9];
	dvlkalman.P_matrix[12][12] = powl(0.02*dph, 2); dvlkalman.P_matrix[13][13] = dvlkalman.P_matrix[12][12]; dvlkalman.P_matrix[14][14] = dvlkalman.P_matrix[12][12];
	dvlkalman.P_matrix[15][15] = powl(0.001, 2);   //DVL 刻度因子参数对应Pk设小，不重新估计
	dvlkalman.P_matrix[16][16] = powl(5, 2); dvlkalman.P_matrix[17][17] = powl(5, 2); dvlkalman.P_matrix[18][18] = powl(5, 2);   //水流速度

	memset(cmpkalman.P_matrix, 0, sizeof(cmpkalman.P_matrix));
	cmpkalman.P_matrix[0][0] = powl(0.1, 2); cmpkalman.P_matrix[1][1] = cmpkalman.P_matrix[0][0]; cmpkalman.P_matrix[2][2] = cmpkalman.P_matrix[0][0];
	cmpkalman.P_matrix[3][3] = powl(0.01 * D2R, 2); cmpkalman.P_matrix[4][4] = cmpkalman.P_matrix[3][3]; cmpkalman.P_matrix[5][5] = powl(0.1 * D2R, 2);
	cmpkalman.P_matrix[6][6] = powl(10.0 / RE, 2); cmpkalman.P_matrix[7][7] = cmpkalman.P_matrix[6][6]; cmpkalman.P_matrix[8][8] = powl(10, 2);
	cmpkalman.P_matrix[9][9] = powl(100 * ug, 2); cmpkalman.P_matrix[10][10] = cmpkalman.P_matrix[9][9]; cmpkalman.P_matrix[11][11] = cmpkalman.P_matrix[9][9];
	cmpkalman.P_matrix[12][12] = powl(0.02*dph, 2); cmpkalman.P_matrix[13][13] = cmpkalman.P_matrix[12][12]; cmpkalman.P_matrix[14][14] = cmpkalman.P_matrix[12][12];
	
	memcpy(depkalman.P_matrix, cmpkalman.P_matrix, sizeof(cmpkalman.P_matrix));

	memset(dvlkalman.X_vector, 0, sizeof(dvlkalman.X_vector)); dvlkalman.X_vector[15] = deltaK;
	memset(cmpkalman.X_vector, 0, sizeof(cmpkalman.X_vector));
	memset(depkalman.X_vector, 0, sizeof(depkalman.X_vector));
}
void initial_mode3to2(SKALMAN_19_6&dvlkalman, SKALMAN_15_1& cmpkalman, SKALMAN_15_1&depkalman, double deltaK)
{
	memset(dvlkalman.P_matrix, 0, sizeof(dvlkalman.P_matrix));
	dvlkalman.P_matrix[0][0] = powl(0.1, 2); dvlkalman.P_matrix[1][1] = dvlkalman.P_matrix[0][0]; dvlkalman.P_matrix[2][2] = dvlkalman.P_matrix[0][0];
	dvlkalman.P_matrix[3][3] = powl(0.01 * D2R, 2); dvlkalman.P_matrix[4][4] = dvlkalman.P_matrix[3][3]; dvlkalman.P_matrix[5][5] = powl(0.1 * D2R, 2);
	dvlkalman.P_matrix[6][6] = powl(10.0 / RE, 2); dvlkalman.P_matrix[7][7] = dvlkalman.P_matrix[6][6]; dvlkalman.P_matrix[8][8] = powl(10, 2);
	dvlkalman.P_matrix[9][9] = powl(100 * ug, 2); dvlkalman.P_matrix[10][10] = dvlkalman.P_matrix[9][9]; dvlkalman.P_matrix[11][11] = dvlkalman.P_matrix[9][9];
	dvlkalman.P_matrix[12][12] = powl(0.02*dph, 2); dvlkalman.P_matrix[13][13] = dvlkalman.P_matrix[12][12]; dvlkalman.P_matrix[14][14] = dvlkalman.P_matrix[12][12];
	dvlkalman.P_matrix[15][15] = powl(0.001, 2);   //DVL 刻度因子参数对应Pk设小，不重新估计
	dvlkalman.P_matrix[16][16] = powl(0.1, 2); dvlkalman.P_matrix[17][17] = powl(0.1, 2); dvlkalman.P_matrix[18][18] = powl(0.1, 2);   //水流速度

	memset(cmpkalman.P_matrix, 0, sizeof(cmpkalman.P_matrix));
	cmpkalman.P_matrix[0][0] = powl(0.1, 2); cmpkalman.P_matrix[1][1] = cmpkalman.P_matrix[0][0]; cmpkalman.P_matrix[2][2] = cmpkalman.P_matrix[0][0];
	cmpkalman.P_matrix[3][3] = powl(0.01 * D2R, 2); cmpkalman.P_matrix[4][4] = cmpkalman.P_matrix[3][3]; cmpkalman.P_matrix[5][5] = powl(0.1 * D2R, 2);
	cmpkalman.P_matrix[6][6] = powl(10.0 / RE, 2); cmpkalman.P_matrix[7][7] = cmpkalman.P_matrix[6][6]; cmpkalman.P_matrix[8][8] = powl(10, 2);
	cmpkalman.P_matrix[9][9] = powl(100 * ug, 2); cmpkalman.P_matrix[10][10] = cmpkalman.P_matrix[9][9]; cmpkalman.P_matrix[11][11] = cmpkalman.P_matrix[9][9];
	cmpkalman.P_matrix[12][12] = powl(0.02*dph, 2); cmpkalman.P_matrix[13][13] = cmpkalman.P_matrix[12][12]; cmpkalman.P_matrix[14][14] = cmpkalman.P_matrix[12][12];

	memcpy(depkalman.P_matrix, cmpkalman.P_matrix, sizeof(cmpkalman.P_matrix));

	memset(dvlkalman.X_vector, 0, sizeof(dvlkalman.X_vector)); dvlkalman.X_vector[15] = deltaK;
	memset(cmpkalman.X_vector, 0, sizeof(cmpkalman.X_vector));
	memset(depkalman.X_vector, 0, sizeof(depkalman.X_vector));
}
void IMM(SKALMAN_19_6& dvlkalman, SDVLCmpDepth& dObserver, int flag_Rkf) //20180320修改
{

	double Xk0[19], Pk0[19][19], Xk_1[19], Pk_1[19][19], Xk_2[19], Pk_2[19][19], Xk_3[19], Pk_3[19][19];
	double f[3];
	double tmp[6], residual1, residual2, tmp1[3], sum, tmp2[3], tmp3[19], Xk_1_tmp[19], Xk_2_tmp[19], Xk_3_tmp[19];
	double Xk_1_tmp_transpose[19], Xk_2_tmp_transpose[19], Xk_3_tmp_transpose[19], tmp4[19][19], tmp5[19][19];

	double ek[6], ek_transpose[1][6], tmp6[6][19], H_transpose[19][6];
	dObserver.lamda = 0;

	memcpy(Xk0, dvlkalman.X_vector, sizeof(dvlkalman.X_vector));
	memcpy(Pk0, dvlkalman.P_matrix, sizeof(dvlkalman.P_matrix));

	//滤波器1 RKF
	if (flag_Rkf == 1)
	{
		memcpy(dvlkalman.R_measure, dObserver.R1_esti, sizeof(dObserver.R1_esti));
		Kal_robust_update_19_6(dvlkalman,dObserver, 1);
	}
    else
   {
		memcpy(dvlkalman.R_measure, dObserver.R1_esti, sizeof(dObserver.R1_esti));
		Kal_update_19_6(dvlkalman, 1);
    }
	f[0] = dvlkalman.f;
	memcpy(Xk_1, dvlkalman.X_vector, sizeof(dvlkalman.X_vector));
	memcpy(Pk_1, dvlkalman.P_matrix, sizeof(dvlkalman.P_matrix));
	//滤波器2 
	memcpy(dvlkalman.X_vector, Xk0, sizeof(Xk0));
	memcpy(dvlkalman.P_matrix, Pk0, sizeof(Pk0));
	memcpy(dvlkalman.R_measure, dObserver.R2_esti, sizeof(dObserver.R2_esti));
	Kal_update_19_6(dvlkalman, 1);
	f[1] = dvlkalman.f;
	memcpy(Xk_2, dvlkalman.X_vector, sizeof(dvlkalman.X_vector));
	memcpy(Pk_2, dvlkalman.P_matrix, sizeof(dvlkalman.P_matrix));
	//滤波器3 
	memcpy(dvlkalman.X_vector, Xk0, sizeof(Xk0));
	memcpy(dvlkalman.P_matrix, Pk0, sizeof(Pk0));
	memcpy(dvlkalman.R_measure, dObserver.R3_esti, sizeof(dObserver.R3_esti));
	Kal_update_19_6(dvlkalman, 1);
	f[2] = dvlkalman.f;
	memcpy(Xk_3, dvlkalman.X_vector, sizeof(dvlkalman.X_vector));
	memcpy(Pk_3, dvlkalman.P_matrix, sizeof(dvlkalman.P_matrix));

	//模型概率更新
	mamul(1, 3, 3, tmp1, dObserver.u, (double *)dObserver.P);
	tmp2[0] = f[0] * tmp1[0]; tmp2[1] = f[1] * tmp1[1]; tmp2[2] = f[2] * tmp1[2];
	sum = tmp2[0] + tmp2[1] + tmp2[2];
	dObserver.u[0] = tmp2[0] / sum; dObserver.u[1] = tmp2[1] / sum; dObserver.u[2] = tmp2[2] / sum;
	
	//组合输出
	//Xk
	avecmul(19, Xk_1_tmp, Xk_1, dObserver.u[0]);
	avecmul(19, Xk_2_tmp, Xk_2, dObserver.u[1]);
	avecmul(19, Xk_3_tmp, Xk_3, dObserver.u[2]);
	vecadd(19, tmp3, Xk_1_tmp, Xk_2_tmp);
	vecadd(19, dvlkalman.X_vector, tmp3, Xk_3_tmp);
	//Pk
	vecsub(19, Xk_1_tmp, Xk_1, dvlkalman.X_vector);
	vecsub(19, Xk_2_tmp, Xk_2, dvlkalman.X_vector);
	vecsub(19, Xk_3_tmp, Xk_3, dvlkalman.X_vector);
	maturn(19, 1, Xk_1_tmp_transpose, Xk_1_tmp);
	maturn(19, 1, Xk_2_tmp_transpose, Xk_2_tmp);
	maturn(19, 1, Xk_3_tmp_transpose, Xk_3_tmp);

	mamul(19, 1, 19, (double *) tmp4, Xk_1_tmp, Xk_1_tmp_transpose);
	maadd(19, 19, (double *)tmp5, (double *)Pk_1, (double *)tmp4);
	amamul(19, 19, (double *)dvlkalman.P_matrix, (double *)tmp5, dObserver.u[0]);

	mamul(19, 1, 19, (double *)tmp4, Xk_2_tmp, Xk_2_tmp_transpose);
	maadd(19, 19, (double *)tmp5, (double *)Pk_2, (double *)tmp4);
	amamul(19, 19, (double *)tmp4, (double *)tmp5, dObserver.u[1]);
	maadd(19, 19, (double *)dvlkalman.P_matrix, (double *)dvlkalman.P_matrix, (double *)tmp4);

	mamul(19, 1, 19, (double *)tmp4, Xk_3_tmp, Xk_3_tmp_transpose);
	maadd(19, 19, (double *)tmp5, (double *)Pk_3, (double *)tmp4);
	amamul(19, 19, (double *)tmp4, (double *)tmp5, dObserver.u[2]);
	maadd(19, 19, (double *)dvlkalman.P_matrix, (double *)dvlkalman.P_matrix, (double *)tmp4);

}

void navi_DVL_Cmp_Depth(SKALMAN_19_6& dvlkalman, SKALMAN_15_1& cmpkalman, SKALMAN_15_1& depkalman, SKALMAN_15_3& zupkalman, SDVLCmpDepth& dObserver) //20180116
{
	int i = 0, j = 0, flag_Rkf = 0;
	double F_15[15][15] = { 0 }, F_19[19][19] = { 0 }, F_ZUPT[15][15] = { 0 }, F_15_Cmp[15][15], F_15_Dep[15][15];
	double ek[6], ek_transpose[1][6], tmp1[6][6], tmp2[6][19], H_transpose[19][6], tmp3[6][6];
	double sum, a, b, c, u1, sum1, sum2, sum3, sum4;
	double Pg[15][15], tmp_Pk[15][15], Pk_transpose[15][15], PC_transpose[15][15], PD_transpose[15][15];
	double Xk[15], Xg[15], Zk_ZUPT1[3], Zk_ZUPT2[3], Zk_ZUPT3[3];
	double cnn[3][3] = { 0.0 };
	double DVL_n[3], kernal_AI[1][1899], INPUT_AI[12], Input_tmp[12], s1, s2, T11[1][1], T22[1][1], T33[1][1], tmp4[3], tmp5[3], tmp6[12];
	sum = 0; sum1 = 0; sum2 = 0; sum3 = 0; sum4 = 0;
	double tmp_R[6][6], tmp7[1][6], lamda_A[1][1], tmp8[6][6];
	double tmp_Z1[1][100], tmp_Z2[1][1], tmp_Z3[1][1];


	F_matrix_15(infor, F_15);
	memcpy(F_ZUPT, F_15, sizeof(F_15));
	for (i = 0; i < 15; i++)
	{
		for (j = 0; j < 15; j++)
		{
			F_19[i][j] = F_15[i][j];
		}
	}
	F_19[16][16] = -infor.vel_n[0] / dObserver.L; F_19[17][17] = -infor.vel_n[1] / dObserver.L; F_19[18][18] = -infor.vel_n[2] / dObserver.L;
	//开辟空间存储F阵！否则预测函数会对其进行修改！
	memcpy(F_15_Cmp, F_15, sizeof(F_15_Cmp));
	memcpy(F_15_Dep, F_15, sizeof(F_15_Dep));

	Kal_forecast_15_1(cmpkalman, sysc.Ts, F_15_Cmp);//参数传进去以后，函数会对参数进行改变！！！！！！所以不能连续用此参数！
	Kal_forecast_15_1(depkalman, sysc.Ts, F_15_Dep);
	Kal_forecast_19(dvlkalman, sysc.Ts, F_19);

	if (dObserver.flag_mode == 3 && dObserver.flag_pureins == 1)
	{
		Kal_forecast_15(zupkalman, sysc.Ts, F_ZUPT);
	}

	//每1s进行组合
	if (0 == sysc.data_cnt % (sysc.Fs / sysc.Kal_fr))
	{
		//模式赋值
		switch (dObserver.flag_DVL)
		{
		case 0:dObserver.flag_mode = 0; break;
		case 1:
			if (dObserver.flag_current == 0)
				dObserver.flag_mode = 1;
			else
				dObserver.flag_mode = 3;
			break;
		case 2:dObserver.flag_mode = 2; break;
		case 3:
			if (dObserver.flag_AI == 1 && dObserver.flag_AI_Ready == 1 && (dObserver.flag_mode_1 == 0 || dObserver.flag_mode_1 == 2 || dObserver.flag_mode_1 == 4))
				dObserver.flag_mode = 4;
			else
				dObserver.flag_mode = 3; 
			break;
		default:break;
		}
		//模式判断,进行操作
		switch (dObserver.flag_mode)
		{
/**/	case 0:
			if (dObserver.flag_mode_1 != 0) initial_mode0_2(dObserver);//用来初始化模式0和2的函数
			switch (dObserver.flag_mode_1)
			{
			case 1: dvlkalman.P_matrix[16][16] = powl(5, 2); dvlkalman.P_matrix[17][17] = powl(5, 2); dvlkalman.P_matrix[18][18] = powl(5, 2); break;
			case 2: dvlkalman.P_matrix[16][16] = powl(5, 2); dvlkalman.P_matrix[17][17] = powl(5, 2); dvlkalman.P_matrix[18][18] = powl(5, 2); break;
			case 3: if (dObserver.flag_pureinsLong == 0){ dvlkalman.P_matrix[16][16] = powl(5, 2); dvlkalman.P_matrix[17][17] = powl(5, 2); dvlkalman.P_matrix[18][18] = powl(5, 2); }
					else
					{
						initial_mode3to0(dvlkalman, cmpkalman, depkalman,dObserver.deltaK);	
					}break;
			default:break;
			}
			dObserver.flag_mode_1 = 0; dObserver.flag_current = 0;
			//INS/DVL滤波器
			vecmul(3, 3, infor.vel_b, (double*)infor.cnb_mat, infor.vel_n);
			dvlkalman.Mea_vector[0] = infor.vel_b[0] - dObserver.DVL_d[0]; dvlkalman.Mea_vector[1] = infor.vel_b[1] - dObserver.DVL_d[1]; dvlkalman.Mea_vector[2] = infor.vel_b[2] - dObserver.DVL_d[2];
			dvlkalman.Mea_vector[3] = infor.vel_b[0] - dObserver.DVL_c[0]; dvlkalman.Mea_vector[4] = infor.vel_b[1] - dObserver.DVL_c[1]; dvlkalman.Mea_vector[5] = infor.vel_b[2] - dObserver.DVL_c[2];
			memset(dvlkalman.H_matrix, 0, sizeof(dvlkalman.H_matrix));
			dvlkalman.H_matrix[0][0] = infor.cnb_mat[0][0]; dvlkalman.H_matrix[0][1] = infor.cnb_mat[0][1]; dvlkalman.H_matrix[0][2] = infor.cnb_mat[0][2];
			dvlkalman.H_matrix[1][0] = infor.cnb_mat[1][0]; dvlkalman.H_matrix[1][1] = infor.cnb_mat[1][1]; dvlkalman.H_matrix[1][2] = infor.cnb_mat[1][2];
			dvlkalman.H_matrix[2][0] = infor.cnb_mat[2][0]; dvlkalman.H_matrix[2][1] = infor.cnb_mat[2][1]; dvlkalman.H_matrix[2][2] = infor.cnb_mat[2][2];
			dvlkalman.H_matrix[0][3] = infor.cnb_mat[0][2] * infor.vel_n[1] - infor.cnb_mat[0][1] * infor.vel_n[2];
			dvlkalman.H_matrix[0][4] = infor.cnb_mat[0][0] * infor.vel_n[2] - infor.cnb_mat[0][2] * infor.vel_n[0];
			dvlkalman.H_matrix[0][5] = infor.cnb_mat[0][1] * infor.vel_n[0] - infor.cnb_mat[0][0] * infor.vel_n[1];
			dvlkalman.H_matrix[1][3] = infor.cnb_mat[1][2] * infor.vel_n[1] - infor.cnb_mat[1][1] * infor.vel_n[2];
			dvlkalman.H_matrix[1][4] = infor.cnb_mat[1][0] * infor.vel_n[2] - infor.cnb_mat[1][2] * infor.vel_n[0];
			dvlkalman.H_matrix[1][5] = infor.cnb_mat[1][1] * infor.vel_n[0] - infor.cnb_mat[1][0] * infor.vel_n[1];
			dvlkalman.H_matrix[2][3] = infor.cnb_mat[2][2] * infor.vel_n[1] - infor.cnb_mat[2][1] * infor.vel_n[2];
			dvlkalman.H_matrix[2][4] = infor.cnb_mat[2][0] * infor.vel_n[2] - infor.cnb_mat[2][2] * infor.vel_n[0];
			dvlkalman.H_matrix[2][5] = infor.cnb_mat[2][1] * infor.vel_n[0] - infor.cnb_mat[2][0] * infor.vel_n[1];
			dvlkalman.H_matrix[0][15] = -infor.vel_b[0]; dvlkalman.H_matrix[1][15] = -infor.vel_b[1]; dvlkalman.H_matrix[2][15] = -infor.vel_b[2];
			for (i = 0; i < 3; i++)
			for (j = 0; j < 15; j++)
			{
				dvlkalman.H_matrix[i + 3][j] = dvlkalman.H_matrix[i][j];
			}
			dvlkalman.H_matrix[3][15] = -dObserver.DVL_c[0]; dvlkalman.H_matrix[4][15] = -dObserver.DVL_c[1]; dvlkalman.H_matrix[5][15] = -dObserver.DVL_c[2];
			dvlkalman.H_matrix[3][16] = infor.cnb_mat[0][0]; dvlkalman.H_matrix[3][17] = infor.cnb_mat[0][1]; dvlkalman.H_matrix[3][18] = infor.cnb_mat[0][2];
			dvlkalman.H_matrix[4][16] = infor.cnb_mat[1][0]; dvlkalman.H_matrix[4][17] = infor.cnb_mat[1][1]; dvlkalman.H_matrix[4][18] = infor.cnb_mat[1][2];
			dvlkalman.H_matrix[5][16] = infor.cnb_mat[2][0]; dvlkalman.H_matrix[5][17] = infor.cnb_mat[2][1]; dvlkalman.H_matrix[5][18] = infor.cnb_mat[2][2];
			//AKF外信息质量监测与Rk调节
			if (sysc.cnt_s >= dObserver.k_0 + 20)//20s之后开始自适应调节Rk阵
			{
				//统计外信息噪声
				vecmul(6, 19, ek, (double *)dvlkalman.H_matrix, dvlkalman.X_vector);//Hk*Xkk_1_A
				vecsub(6, ek, dvlkalman.Mea_vector, ek);//  ek=(Zk-Hk*Xkk_1_A);
				maturn(6, 1, (double *)ek_transpose, (double *)ek);//ek'
				mamul(6, 19, 19, (double *)tmp2, (double *)dvlkalman.H_matrix, (double *)dvlkalman.P_matrix);//Hk*Pkk_1
				maturn(6, 19, (double *)H_transpose, (double *)dvlkalman.H_matrix);//Hk'
				mamul(6, 19, 6, (double *)tmp8, (double *)tmp2, (double *)H_transpose);//Hk*Pkk_1*Hk'
				amamul(6, 6, (double *)tmp_R, (double *)dObserver.R1_esti, 3);//3*Rt_1_esti
				maadd(6, 6, (double *)tmp3, (double *)tmp8, (double *)tmp_R);//Pzz= Hk*Pkk_1*Hk'+ 3*Rt_1_esti;
				mainv(6, (double *)tmp3);//Pzz^-1
				mamul(1, 6, 6, (double *)tmp7, (double *)ek_transpose, (double *)tmp3);
				mamul(1, 6, 1, (double *)lamda_A, (double *)tmp7, (double *)ek);//lamda_A=ek'*Pzz_1^(-1)*ek;

				if (lamda_A[0][0]<= 100)
				{
					mamul(6, 1, 6, (double *)tmp1, (double *)ek, (double *)ek_transpose);//ek*ek'
					masub(6, 6, (double *)dObserver.R_adaptive, (double *)tmp1, (double *)tmp8);//ek*ek'-Hk*Pkk_1_A*Hk'
				}

				for (i = 0; i < 19; i++)
				for (j = 0; j < 6; j++)
				{
					dObserver.R_A_store[i][j] = dObserver.R_A_store[i + 1][j];
					sum = sum + dObserver.R_A_store[i][j];
				}
				dObserver.R_A_store[19][0] = dObserver.R_adaptive[0][0]; dObserver.R_A_store[19][1] = dObserver.R_adaptive[1][1]; dObserver.R_A_store[19][2] = dObserver.R_adaptive[2][2];
				dObserver.R_A_store[19][3] = dObserver.R_adaptive[3][3]; dObserver.R_A_store[19][4] = dObserver.R_adaptive[4][4]; dObserver.R_A_store[19][5] = dObserver.R_adaptive[5][5];
				sum = sum + dObserver.R_A_store[19][0] + dObserver.R_A_store[19][1] + dObserver.R_A_store[19][2] + dObserver.R_A_store[19][3] + dObserver.R_A_store[19][4] + dObserver.R_A_store[19][5];
				sum = sum / 120;
				//自适应调节R阵
				if (sum < 1) //大于，就应该让RKF去处理  令R1_esti不超过1
				{
					if (sqrt(sum) > sqrt(dObserver.R1_esti[0][0]) + 0.08)
					{
						a = sqrt(dObserver.R1_esti[0][0]); a = a + 0.1; b = a + 0.1; c = a - 0.1;
						dObserver.R1_esti[0][0] = powl(a, 2); dObserver.R1_esti[1][1] = powl(a, 2);	dObserver.R1_esti[2][2] = powl(a, 2);
						dObserver.R1_esti[3][3] = powl(a, 2); dObserver.R1_esti[4][4] = powl(a, 2);	dObserver.R1_esti[5][5] = powl(a, 2);
						dObserver.R2_esti[0][0] = powl(b, 2); dObserver.R2_esti[1][1] = powl(b, 2);	dObserver.R2_esti[2][2] = powl(b, 2);
						dObserver.R2_esti[3][3] = powl(b, 2); dObserver.R2_esti[4][4] = powl(b, 2);	dObserver.R2_esti[5][5] = powl(b, 2);
						dObserver.R3_esti[0][0] = powl(c, 2); dObserver.R3_esti[1][1] = powl(c, 2);	dObserver.R3_esti[2][2] = powl(c, 2);
						dObserver.R3_esti[3][3] = powl(c, 2); dObserver.R3_esti[4][4] = powl(c, 2);	dObserver.R3_esti[5][5] = powl(c, 2);
						u1 = dObserver.u[2]; dObserver.u[2] = dObserver.u[0]; dObserver.u[0] = dObserver.u[1]; dObserver.u[1] = u1;
					}
					else if (sqrt(sum) < sqrt(dObserver.R1_esti[0][0]) - 0.08)
					{
						a = sqrt(dObserver.R1_esti[0][0]); a = a - 0.1; b = a + 0.1; c = a - 0.1;
						if (c <= 0.05) 
						{ 
							c = 0.05; a = 0.1; b = 0.2;
						}
						else
						{
							u1 = dObserver.u[1]; dObserver.u[1] = dObserver.u[0]; dObserver.u[0] = dObserver.u[2]; dObserver.u[2] = u1;
						}
						dObserver.R1_esti[0][0] = powl(a, 2); dObserver.R1_esti[1][1] = powl(a, 2);	dObserver.R1_esti[2][2] = powl(a, 2);
						dObserver.R1_esti[3][3] = powl(a, 2); dObserver.R1_esti[4][4] = powl(a, 2);	dObserver.R1_esti[5][5] = powl(a, 2);
						dObserver.R2_esti[0][0] = powl(b, 2); dObserver.R2_esti[1][1] = powl(b, 2);	dObserver.R2_esti[2][2] = powl(b, 2);
						dObserver.R2_esti[3][3] = powl(b, 2); dObserver.R2_esti[4][4] = powl(b, 2);	dObserver.R2_esti[5][5] = powl(b, 2);
						dObserver.R3_esti[0][0] = powl(c, 2); dObserver.R3_esti[1][1] = powl(c, 2);	dObserver.R3_esti[2][2] = powl(c, 2);
						dObserver.R3_esti[3][3] = powl(c, 2); dObserver.R3_esti[4][4] = powl(c, 2);	dObserver.R3_esti[5][5] = powl(c, 2);		
					}
				}
			}
			//50s之后开始可以用Rkf
			if (sysc.cnt_s >= dObserver.k_0 + 50) flag_Rkf = 1; else flag_Rkf = 0;
			//IMM 滤波
			IMM(dvlkalman, dObserver, flag_Rkf);

			//高度计 默认一直存在
			depkalman.Mea_vector[0] = infor.pos[2] - dObserver.Depth;
			memset(depkalman.H_matrix, 0, sizeof(depkalman.H_matrix));
			depkalman.H_matrix[0][8] = 1;
			Kal_update_15_1(depkalman, 1);

			//磁罗经 判断是否有磁罗经；联邦滤波
			if (dObserver.flag_C == 0)  //有磁罗经
			{
				cmpkalman.Mea_vector[0] = infor.att_angle[2] - dObserver.Cmp;
				memset(cmpkalman.H_matrix, 0, sizeof(cmpkalman.H_matrix));
				cmpkalman.H_matrix[0][5] = -1;
				Kal_update_15_1(cmpkalman, 1);

					//DVL_Depth_Cmp三个子滤波器联邦滤波
					for (i = 0; i < 15; i++)
					for (j = 0; j < 15; j++)
					{
						Pk_transpose[i][j] = dvlkalman.P_matrix[i][j];
					}
					//计算Pg
					mainv(15, (double *)Pk_transpose);
					memcpy(PC_transpose, cmpkalman.P_matrix, sizeof(cmpkalman.P_matrix));
					mainv(15, (double *)PC_transpose);
					memcpy(PD_transpose, depkalman.P_matrix, sizeof(depkalman.P_matrix));
					mainv(15, (double *)PD_transpose);
					maadd(15, 15, (double *)tmp_Pk, (double *)PC_transpose, (double *)PD_transpose);
					maadd(15, 15, (double *)Pg, (double *)Pk_transpose, (double *)tmp_Pk);
					mainv(15, (double *)Pg);
					//计算Xg
					memcpy(Xk, dvlkalman.X_vector, sizeof(Xk));
					mamul(15, 15, 1, Xk, (double *)Pk_transpose, Xk);
					mamul(15, 15, 1, depkalman.X_vector, (double *)PD_transpose, depkalman.X_vector);
					mamul(15, 15, 1, cmpkalman.X_vector, (double *)PC_transpose, cmpkalman.X_vector);
					vecadd(15, Xg, Xk, depkalman.X_vector);
					vecadd(15, Xg, Xg, cmpkalman.X_vector);
					mamul(15, 15, 1, Xg, (double *)Pg, Xg);
			}
			else
			{   //DVL_Depth两个子滤波器联邦滤波
				for (i = 0; i < 15; i++)
				for (j = 0; j < 15; j++)
				{
					Pk_transpose[i][j] = dvlkalman.P_matrix[i][j];
				}
				//计算Pg
				mainv(15, (double *)Pk_transpose);
				memcpy(PD_transpose, depkalman.P_matrix, sizeof(depkalman.P_matrix));
				mainv(15, (double *)PD_transpose);
				maadd(15, 15, (double *)Pg, (double *)Pk_transpose, (double *)PD_transpose);
				mainv(15, (double *)Pg);
				//计算Xg
				memcpy(Xk, dvlkalman.X_vector, sizeof(Xk));
				mamul(15, 15, 1, Xk, (double *)Pk_transpose, Xk);
				mamul(15, 15, 1, depkalman.X_vector, (double *)PD_transpose, depkalman.X_vector);
				vecadd(15, Xg, Xk, depkalman.X_vector);
				mamul(15, 15, 1, Xg, (double *)Pg, Xg);
			}

			//反馈校正
			vecsub(3, infor.vel_n, infor.vel_n, Xg);
			vecsub(3, infor.pos, infor.pos, Xg + 6);
			X2cnn(cnn, Xg + 3);
			mamul(3, 3, 3, (double *)infor.cnb_mat, (double *)infor.cnb_mat, (double *)cnn);
			maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
			cnb2ang(infor.cnb_mat, infor.att_angle);
			cnb2q(infor.cnb_mat, infor.quart);
			for (i = 0; i < 9; i++)
				Xg[i] = 0.0;
			for (i = 0; i < 3; i++)
			{
				infor.gyro_bias_esti[i] = Xg[12 + i] * 180 / 3.14 * 3600;
				infor.acce_bias_esti[i] = Xg[9 + i] * 1000000 / 9.78;
			}
			memcpy(dvlkalman.X_vector, Xg, sizeof(Xg));
			memcpy(cmpkalman.X_vector, Xg, sizeof(Xg));
			memcpy(depkalman.X_vector, Xg, sizeof(Xg));

			break;


/**/	case 1:
			if (dObserver.flag_mode_1 != 1) initial_mode1(dObserver);//用来初始化模式1的函数
			dObserver.flag_mode_1 = 1; dObserver.flag_current = 0; //认为可以持续进行水流辅助
			//INS/DVL滤波器
			vecmul(3, 3, infor.vel_b, (double*)infor.cnb_mat, infor.vel_n);
			dvlkalman.Mea_vector[0] = 0; dvlkalman.Mea_vector[1] = 0; dvlkalman.Mea_vector[2] = 0;
			dvlkalman.Mea_vector[3] = infor.vel_b[0] - dObserver.DVL_c[0]; dvlkalman.Mea_vector[4] = infor.vel_b[1] - dObserver.DVL_c[1]; dvlkalman.Mea_vector[5] = infor.vel_b[2] - dObserver.DVL_c[2];
			memset(dvlkalman.H_matrix, 0, sizeof(dvlkalman.H_matrix));
			dvlkalman.H_matrix[0][0] = infor.cnb_mat[0][0]; dvlkalman.H_matrix[0][1] = infor.cnb_mat[0][1]; dvlkalman.H_matrix[0][2] = infor.cnb_mat[0][2];
			dvlkalman.H_matrix[1][0] = infor.cnb_mat[1][0]; dvlkalman.H_matrix[1][1] = infor.cnb_mat[1][1]; dvlkalman.H_matrix[1][2] = infor.cnb_mat[1][2];
			dvlkalman.H_matrix[2][0] = infor.cnb_mat[2][0]; dvlkalman.H_matrix[2][1] = infor.cnb_mat[2][1]; dvlkalman.H_matrix[2][2] = infor.cnb_mat[2][2];
			dvlkalman.H_matrix[0][3] = infor.cnb_mat[0][2] * infor.vel_n[1] - infor.cnb_mat[0][1] * infor.vel_n[2];
			dvlkalman.H_matrix[0][4] = infor.cnb_mat[0][0] * infor.vel_n[2] - infor.cnb_mat[0][2] * infor.vel_n[0];
			dvlkalman.H_matrix[0][5] = infor.cnb_mat[0][1] * infor.vel_n[0] - infor.cnb_mat[0][0] * infor.vel_n[1];
			dvlkalman.H_matrix[1][3] = infor.cnb_mat[1][2] * infor.vel_n[1] - infor.cnb_mat[1][1] * infor.vel_n[2];
			dvlkalman.H_matrix[1][4] = infor.cnb_mat[1][0] * infor.vel_n[2] - infor.cnb_mat[1][2] * infor.vel_n[0];
			dvlkalman.H_matrix[1][5] = infor.cnb_mat[1][1] * infor.vel_n[0] - infor.cnb_mat[1][0] * infor.vel_n[1];
			dvlkalman.H_matrix[2][3] = infor.cnb_mat[2][2] * infor.vel_n[1] - infor.cnb_mat[2][1] * infor.vel_n[2];
			dvlkalman.H_matrix[2][4] = infor.cnb_mat[2][0] * infor.vel_n[2] - infor.cnb_mat[2][2] * infor.vel_n[0];
			dvlkalman.H_matrix[2][5] = infor.cnb_mat[2][1] * infor.vel_n[0] - infor.cnb_mat[2][0] * infor.vel_n[1];
			dvlkalman.H_matrix[0][15] = -infor.vel_b[0]; dvlkalman.H_matrix[1][15] = -infor.vel_b[1]; dvlkalman.H_matrix[2][15] = -infor.vel_b[2];
			for (i = 0; i < 3; i++)
			for (j = 0; j < 15; j++)
			{
				dvlkalman.H_matrix[i + 3][j] = dvlkalman.H_matrix[i][j];
			}
			dvlkalman.H_matrix[3][15] = -dObserver.DVL_c[0]; dvlkalman.H_matrix[4][15] = -dObserver.DVL_c[1]; dvlkalman.H_matrix[5][15] = -dObserver.DVL_c[2];
			dvlkalman.H_matrix[3][16] = infor.cnb_mat[0][0]; dvlkalman.H_matrix[3][17] = infor.cnb_mat[0][1]; dvlkalman.H_matrix[3][18] = infor.cnb_mat[0][2];
			dvlkalman.H_matrix[4][16] = infor.cnb_mat[1][0]; dvlkalman.H_matrix[4][17] = infor.cnb_mat[1][1]; dvlkalman.H_matrix[4][18] = infor.cnb_mat[1][2];
			dvlkalman.H_matrix[5][16] = infor.cnb_mat[2][0]; dvlkalman.H_matrix[5][17] = infor.cnb_mat[2][1]; dvlkalman.H_matrix[5][18] = infor.cnb_mat[2][2];
			//AKF外信息质量监测与Rk调节
			if (sysc.cnt_s >= dObserver.k_0 + 20)//20s之后开始自适应调节Rk阵
			{
				//统计外信息噪声
				vecmul(6, 19, ek, (double *)dvlkalman.H_matrix, dvlkalman.X_vector);//Hk*Xkk_1_A
				vecsub(6, ek, dvlkalman.Mea_vector, ek);//  ek=(Zk-Hk*Xkk_1_A);
				maturn(6, 1, (double *)ek_transpose, (double *)ek);//ek'
				mamul(6, 19, 19, (double *)tmp2, (double *)dvlkalman.H_matrix, (double *)dvlkalman.P_matrix);//Hk*Pkk_1
				maturn(6, 19, (double *)H_transpose, (double *)dvlkalman.H_matrix);//Hk'
				mamul(6, 19, 6, (double *)tmp8, (double *)tmp2, (double *)H_transpose);//Hk*Pkk_1*Hk'
				amamul(6, 6, (double *)tmp_R, (double *)dObserver.R1_esti, 3);//3*Rt_1_esti
				maadd(6, 6, (double *)tmp3, (double *)tmp8, (double *)tmp_R);//Pzz= Hk*Pkk_1*Hk'+ 3*Rt_1_esti;
				mainv(6, (double *)tmp3);//Pzz^-1
				mamul(1, 6, 6, (double *)tmp7, (double *)ek_transpose, (double *)tmp3);
				mamul(1, 6, 1, (double *)lamda_A, (double *)tmp7, (double *)ek);//lamda_A=ek'*Pzz_1^(-1)*ek;

				if (lamda_A[0][0] <= 100)
				{
					mamul(6, 1, 6, (double *)tmp1, (double *)ek, (double *)ek_transpose);//ek*ek'
					masub(6, 6, (double *)dObserver.R_adaptive, (double *)tmp1, (double *)tmp8);//ek*ek'-Hk*Pkk_1_A*Hk'
				}

				for (i = 0; i < 19; i++)
				for (j = 0; j < 6; j++)
				{
					dObserver.R_A_store[i][j] = dObserver.R_A_store[i + 1][j];
					if (j>2)
					{
						sum1 = sum1 + dObserver.R_A_store[i][j];
					}
				}
				dObserver.R_A_store[19][0] = dObserver.R_adaptive[0][0]; dObserver.R_A_store[19][1] = dObserver.R_adaptive[1][1]; dObserver.R_A_store[19][2] = dObserver.R_adaptive[2][2];
				dObserver.R_A_store[19][3] = dObserver.R_adaptive[3][3]; dObserver.R_A_store[19][4] = dObserver.R_adaptive[4][4]; dObserver.R_A_store[19][5] = dObserver.R_adaptive[5][5];
				sum1 = sum1 + dObserver.R_A_store[19][3] + dObserver.R_A_store[19][4] + dObserver.R_A_store[19][5];
				sum1 = sum1 / 60;
				//自适应调节R阵
				if (sum1 < 1) //大于，就应该让RKF去处理  令R1_esti不超过1
				{
					if (sqrt(sum1) > sqrt(dObserver.R1_esti[3][3]) + 0.08)
					{
						a = sqrt(dObserver.R1_esti[3][3]); a = a + 0.1; b = a + 0.1; c = a - 0.1;
						dObserver.R1_esti[0][0] = powl(10, 2); dObserver.R1_esti[1][1] = powl(10, 2);	dObserver.R1_esti[2][2] = powl(10, 2);
						dObserver.R1_esti[3][3] = powl(a, 2); dObserver.R1_esti[4][4] = powl(a, 2);	dObserver.R1_esti[5][5] = powl(a, 2);
						dObserver.R2_esti[0][0] = powl(10, 2); dObserver.R2_esti[1][1] = powl(10, 2);	dObserver.R2_esti[2][2] = powl(10, 2);
						dObserver.R2_esti[3][3] = powl(b, 2); dObserver.R2_esti[4][4] = powl(b, 2);	dObserver.R2_esti[5][5] = powl(b, 2);
						dObserver.R3_esti[0][0] = powl(10, 2); dObserver.R3_esti[1][1] = powl(10, 2);	dObserver.R3_esti[2][2] = powl(10, 2);
						dObserver.R3_esti[3][3] = powl(c, 2); dObserver.R3_esti[4][4] = powl(c, 2);	dObserver.R3_esti[5][5] = powl(c, 2);
						u1 = dObserver.u[2]; dObserver.u[2] = dObserver.u[0]; dObserver.u[0] = dObserver.u[1]; dObserver.u[1] = u1;
					}
					else if (sqrt(sum1) < sqrt(dObserver.R1_esti[3][3]) - 0.08)
					{
						a = sqrt(dObserver.R1_esti[3][3]); a = a - 0.1; b = a + 0.1; c = a - 0.1;
						if (c <= 0.05)
						{
							c = 0.05; a = 0.1; b = 0.2;
						}
						else
						{
							u1 = dObserver.u[1]; dObserver.u[1] = dObserver.u[0]; dObserver.u[0] = dObserver.u[2]; dObserver.u[2] = u1;
						}
						dObserver.R1_esti[0][0] = powl(10, 2); dObserver.R1_esti[1][1] = powl(10, 2);	dObserver.R1_esti[2][2] = powl(10, 2);
						dObserver.R1_esti[3][3] = powl(a, 2); dObserver.R1_esti[4][4] = powl(a, 2);	dObserver.R1_esti[5][5] = powl(a, 2);
						dObserver.R2_esti[0][0] = powl(10, 2); dObserver.R2_esti[1][1] = powl(10, 2);	dObserver.R2_esti[2][2] = powl(10, 2);
						dObserver.R2_esti[3][3] = powl(b, 2); dObserver.R2_esti[4][4] = powl(b, 2);	dObserver.R2_esti[5][5] = powl(b, 2);
						dObserver.R3_esti[0][0] = powl(10, 2); dObserver.R3_esti[1][1] = powl(10, 2);	dObserver.R3_esti[2][2] = powl(10, 2);
						dObserver.R3_esti[3][3] = powl(c, 2); dObserver.R3_esti[4][4] = powl(c, 2);	dObserver.R3_esti[5][5] = powl(c, 2);
					}
				}
			}
			//50s之后开始可以用Rkf
			if (sysc.cnt_s >= dObserver.k_0 + 50) flag_Rkf = 1; else flag_Rkf = 0;
			//IMM 滤波
			IMM(dvlkalman, dObserver, flag_Rkf);
			//高度计 默认一直存在
			depkalman.Mea_vector[0] = infor.pos[2] - dObserver.Depth;
			memset(depkalman.H_matrix, 0, sizeof(depkalman.H_matrix));
			depkalman.H_matrix[0][8] = 1;
			Kal_update_15_1(depkalman, 1);

			//磁罗经 判断是否有磁罗经；联邦滤波
			if (dObserver.flag_C == 0)  //有磁罗经
			{
				cmpkalman.Mea_vector[0] = infor.att_angle[2] - dObserver.Cmp;
				memset(cmpkalman.H_matrix, 0, sizeof(cmpkalman.H_matrix));
				cmpkalman.H_matrix[0][5] = -1;
				Kal_update_15_1(cmpkalman, 1);

				//DVL_Depth_Cmp三个子滤波器联邦滤波
				for (i = 0; i < 15; i++)
				for (j = 0; j < 15; j++)
				{
					Pk_transpose[i][j] = dvlkalman.P_matrix[i][j];
				}
				//计算Pg
				mainv(15, (double *)Pk_transpose);
				memcpy(PC_transpose, cmpkalman.P_matrix, sizeof(cmpkalman.P_matrix));
				mainv(15, (double *)PC_transpose);
				memcpy(PD_transpose, depkalman.P_matrix, sizeof(depkalman.P_matrix));
				mainv(15, (double *)PD_transpose);
				maadd(15, 15, (double *)tmp_Pk, (double *)PC_transpose, (double *)PD_transpose);
				maadd(15, 15, (double *)Pg, (double *)Pk_transpose, (double *)tmp_Pk);
				mainv(15, (double *)Pg);
				//计算Xg
				memcpy(Xk, dvlkalman.X_vector, sizeof(Xk));
				mamul(15, 15, 1, Xk, (double *)Pk_transpose, Xk);
				mamul(15, 15, 1, depkalman.X_vector, (double *)PD_transpose, depkalman.X_vector);
				mamul(15, 15, 1, cmpkalman.X_vector, (double *)PC_transpose, cmpkalman.X_vector);
				vecadd(15, Xg, Xk, depkalman.X_vector);
				vecadd(15, Xg, Xg, cmpkalman.X_vector);
				mamul(15, 15, 1, Xg, (double *)Pg, Xg);
			}
			else
			{   //DVL_Depth两个子滤波器联邦滤波
				for (i = 0; i < 15; i++)
				for (j = 0; j < 15; j++)
				{
					Pk_transpose[i][j] = dvlkalman.P_matrix[i][j];
				}
				//计算Pg
				mainv(15, (double *)Pk_transpose);
				memcpy(PD_transpose, depkalman.P_matrix, sizeof(depkalman.P_matrix));
				mainv(15, (double *)PD_transpose);
				maadd(15, 15, (double *)Pg, (double *)Pk_transpose, (double *)PD_transpose);
				mainv(15, (double *)Pg);
				//计算Xg
				memcpy(Xk, dvlkalman.X_vector, sizeof(Xk));
				mamul(15, 15, 1, Xk, (double *)Pk_transpose, Xk);
				mamul(15, 15, 1, depkalman.X_vector, (double *)PD_transpose, depkalman.X_vector);
				vecadd(15, Xg, Xk, depkalman.X_vector);
				mamul(15, 15, 1, Xg, (double *)Pg, Xg);
			}

			//反馈校正
			vecsub(3, infor.vel_n, infor.vel_n, Xg);
			vecsub(3, infor.pos, infor.pos, Xg + 6);
			X2cnn(cnn, Xg + 3);
			mamul(3, 3, 3, (double *)infor.cnb_mat, (double *)infor.cnb_mat, (double *)cnn);
			maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
			cnb2ang(infor.cnb_mat, infor.att_angle);
			cnb2q(infor.cnb_mat, infor.quart);
			for (i = 0; i < 9; i++)
				Xg[i] = 0.0;
			for (i = 0; i < 3; i++)
			{
				infor.gyro_bias_esti[i] = Xg[12 + i] * 180 / 3.14 * 3600;
				infor.acce_bias_esti[i] = Xg[9 + i] * 1000000 / 9.78;
			}
			memcpy(dvlkalman.X_vector, Xg, sizeof(Xg));
			memcpy(cmpkalman.X_vector, Xg, sizeof(Xg));
			memcpy(depkalman.X_vector, Xg, sizeof(Xg));

			break;

/**/	case 2:
			if (dObserver.flag_mode_1 != 2) initial_mode0_2(dObserver);//用来初始化模式0和2的函数
			switch (dObserver.flag_mode_1)
			{
			case 3:case 4: if (dObserver.flag_pureinsLong != 0)
					{
						initial_mode3to2(dvlkalman, cmpkalman, depkalman, dObserver.deltaK);
					}break;
			default:break;
			}
			dObserver.flag_mode_1 = 2; dObserver.flag_current = 1; //水速无法估计

			//AI信息存储
			//infor.acce_n在sinscal_zundamp()中以被计算、赋值
			if (sysc.cnt_s >= dObserver.k_0 + 20&&dObserver.flag_AI_store==1)
			{
				dObserver.AI_fn_store[0][dObserver.kk_AI] = infor.acce_n[0]; dObserver.AI_fn_store[1][dObserver.kk_AI] = infor.acce_n[1]; dObserver.AI_fn_store[2][dObserver.kk_AI] = infor.acce_n[2];
				dObserver.AI_vn_store[0][dObserver.kk_AI] = infor.vel_n[0];  dObserver.AI_vn_store[1][dObserver.kk_AI] = infor.vel_n[1];  dObserver.AI_vn_store[2][dObserver.kk_AI] = infor.vel_n[2];
				vecmul(3, 3, DVL_n, (double*)infor.cbn_mat, dObserver.DVL_d);
				vecsub(3, DVL_n, infor.vel_n, DVL_n);
				dObserver.AI_deltaVn_store[0][dObserver.kk_AI] = DVL_n[0] * 10; dObserver.AI_deltaVn_store[1][dObserver.kk_AI] = DVL_n[1] * 10; dObserver.AI_deltaVn_store[2][dObserver.kk_AI] = DVL_n[2] * 10;
				dObserver.kk_AI = dObserver.kk_AI + 1;
				if (dObserver.kk_AI >= dObserver.sum_k_AI)//和matlab不同，这里有=号，因为kk_AI从0开始
				{
					dObserver.flag_AI_train = 1; dObserver.flag_AI_store = 0;
				}
			} 
			//记录历史输入信息（作为下一时刻的上一时刻数据）
			memcpy(dObserver.AI_fn_k_1, infor.acce_n, sizeof(infor.acce_n));
			memcpy(dObserver.AI_vns_k_1, infor.vel_n, sizeof(infor.vel_n));
			//AI信息存储 end

			//INS/DVL滤波器
			vecmul(3, 3, infor.vel_b, (double*)infor.cnb_mat, infor.vel_n);
			dvlkalman.Mea_vector[0] = infor.vel_b[0] - dObserver.DVL_d[0]; dvlkalman.Mea_vector[1] = infor.vel_b[1] - dObserver.DVL_d[1]; dvlkalman.Mea_vector[2] = infor.vel_b[2] - dObserver.DVL_d[2];
			dvlkalman.Mea_vector[3] = 0; dvlkalman.Mea_vector[4] =0; dvlkalman.Mea_vector[5] = 0;
			memset(dvlkalman.H_matrix, 0, sizeof(dvlkalman.H_matrix));
			dvlkalman.H_matrix[0][0] = infor.cnb_mat[0][0]; dvlkalman.H_matrix[0][1] = infor.cnb_mat[0][1]; dvlkalman.H_matrix[0][2] = infor.cnb_mat[0][2];
			dvlkalman.H_matrix[1][0] = infor.cnb_mat[1][0]; dvlkalman.H_matrix[1][1] = infor.cnb_mat[1][1]; dvlkalman.H_matrix[1][2] = infor.cnb_mat[1][2];
			dvlkalman.H_matrix[2][0] = infor.cnb_mat[2][0]; dvlkalman.H_matrix[2][1] = infor.cnb_mat[2][1]; dvlkalman.H_matrix[2][2] = infor.cnb_mat[2][2];
			dvlkalman.H_matrix[0][3] = infor.cnb_mat[0][2] * infor.vel_n[1] - infor.cnb_mat[0][1] * infor.vel_n[2];
			dvlkalman.H_matrix[0][4] = infor.cnb_mat[0][0] * infor.vel_n[2] - infor.cnb_mat[0][2] * infor.vel_n[0];
			dvlkalman.H_matrix[0][5] = infor.cnb_mat[0][1] * infor.vel_n[0] - infor.cnb_mat[0][0] * infor.vel_n[1];
			dvlkalman.H_matrix[1][3] = infor.cnb_mat[1][2] * infor.vel_n[1] - infor.cnb_mat[1][1] * infor.vel_n[2];
			dvlkalman.H_matrix[1][4] = infor.cnb_mat[1][0] * infor.vel_n[2] - infor.cnb_mat[1][2] * infor.vel_n[0];
			dvlkalman.H_matrix[1][5] = infor.cnb_mat[1][1] * infor.vel_n[0] - infor.cnb_mat[1][0] * infor.vel_n[1];
			dvlkalman.H_matrix[2][3] = infor.cnb_mat[2][2] * infor.vel_n[1] - infor.cnb_mat[2][1] * infor.vel_n[2];
			dvlkalman.H_matrix[2][4] = infor.cnb_mat[2][0] * infor.vel_n[2] - infor.cnb_mat[2][2] * infor.vel_n[0];
			dvlkalman.H_matrix[2][5] = infor.cnb_mat[2][1] * infor.vel_n[0] - infor.cnb_mat[2][0] * infor.vel_n[1];
			dvlkalman.H_matrix[0][15] = -infor.vel_b[0]; dvlkalman.H_matrix[1][15] = -infor.vel_b[1]; dvlkalman.H_matrix[2][15] = -infor.vel_b[2];
			//AKF外信息质量监测与Rk调节
			if (sysc.cnt_s >= dObserver.k_0 + 20)//20s之后开始自适应调节Rk阵
			{
				//统计外信息噪声
				vecmul(6, 19, ek, (double *)dvlkalman.H_matrix, dvlkalman.X_vector);//Hk*Xkk_1_A
				vecsub(6, ek, dvlkalman.Mea_vector, ek);//  ek=(Zk-Hk*Xkk_1_A);
				maturn(6, 1, (double *)ek_transpose, (double *)ek);//ek'
				mamul(6, 19, 19, (double *)tmp2, (double *)dvlkalman.H_matrix, (double *)dvlkalman.P_matrix);//Hk*Pkk_1
				maturn(6, 19, (double *)H_transpose, (double *)dvlkalman.H_matrix);//Hk'
				mamul(6, 19, 6, (double *)tmp8, (double *)tmp2, (double *)H_transpose);//Hk*Pkk_1*Hk'
				amamul(6, 6, (double *)tmp_R, (double *)dObserver.R1_esti, 3);//3*Rt_1_esti
				maadd(6, 6, (double *)tmp3, (double *)tmp8, (double *)tmp_R);//Pzz= Hk*Pkk_1*Hk'+ 3*Rt_1_esti;
				mainv(6, (double *)tmp3);//Pzz^-1
				mamul(1, 6, 6, (double *)tmp7, (double *)ek_transpose, (double *)tmp3);
				mamul(1, 6, 1, (double *)lamda_A, (double *)tmp7, (double *)ek);//lamda_A=ek'*Pzz_1^(-1)*ek;

				if (lamda_A[0][0] <= 100)
				{
					mamul(6, 1, 6, (double *)tmp1, (double *)ek, (double *)ek_transpose);//ek*ek'
					masub(6, 6, (double *)dObserver.R_adaptive, (double *)tmp1, (double *)tmp8);//ek*ek'-Hk*Pkk_1_A*Hk'
				}

				for (i = 0; i < 19; i++)
				for (j = 0; j < 6; j++)
				{
					dObserver.R_A_store[i][j] = dObserver.R_A_store[i + 1][j];
					if (j<=2)
					{
						sum2 = sum2 + dObserver.R_A_store[i][j];
					}
				}
				dObserver.R_A_store[19][0] = dObserver.R_adaptive[0][0]; dObserver.R_A_store[19][1] = dObserver.R_adaptive[1][1]; dObserver.R_A_store[19][2] = dObserver.R_adaptive[2][2];
				dObserver.R_A_store[19][3] = dObserver.R_adaptive[3][3]; dObserver.R_A_store[19][4] = dObserver.R_adaptive[4][4]; dObserver.R_A_store[19][5] = dObserver.R_adaptive[5][5];
				sum2 = sum2 + dObserver.R_A_store[19][0] + dObserver.R_A_store[19][1] + dObserver.R_A_store[19][2];
				sum2 = sum2 / 60;
				//自适应调节R阵
				if (sum2 < 1) //大于，就应该让RKF去处理  令R1_esti不超过1
				{
					if (sqrt(sum2) > sqrt(dObserver.R1_esti[0][0]) + 0.08)
					{
						a = sqrt(dObserver.R1_esti[0][0]); a = a + 0.1; b = a + 0.1; c = a - 0.1;
						dObserver.R1_esti[0][0] = powl(a, 2); dObserver.R1_esti[1][1] = powl(a, 2);	dObserver.R1_esti[2][2] = powl(a, 2);
						dObserver.R1_esti[3][3] = powl(a, 2); dObserver.R1_esti[4][4] = powl(a, 2);	dObserver.R1_esti[5][5] = powl(a, 2);
						dObserver.R2_esti[0][0] = powl(b, 2); dObserver.R2_esti[1][1] = powl(b, 2);	dObserver.R2_esti[2][2] = powl(b, 2);
						dObserver.R2_esti[3][3] = powl(b, 2); dObserver.R2_esti[4][4] = powl(b, 2);	dObserver.R2_esti[5][5] = powl(b, 2);
						dObserver.R3_esti[0][0] = powl(c, 2); dObserver.R3_esti[1][1] = powl(c, 2);	dObserver.R3_esti[2][2] = powl(c, 2);
						dObserver.R3_esti[3][3] = powl(c, 2); dObserver.R3_esti[4][4] = powl(c, 2);	dObserver.R3_esti[5][5] = powl(c, 2);
						u1 = dObserver.u[2]; dObserver.u[2] = dObserver.u[0]; dObserver.u[0] = dObserver.u[1]; dObserver.u[1] = u1;
					}
					else if (sqrt(sum2) < sqrt(dObserver.R1_esti[0][0]) - 0.08)
					{
						a = sqrt(dObserver.R1_esti[0][0]); a = a - 0.1; b = a + 0.1; c = a - 0.1;
						if (c <= 0.05)
						{
							c = 0.05; a = 0.1; b = 0.2;
						}
						else
						{
							u1 = dObserver.u[1]; dObserver.u[1] = dObserver.u[0]; dObserver.u[0] = dObserver.u[2]; dObserver.u[2] = u1;
						}
						dObserver.R1_esti[0][0] = powl(a, 2); dObserver.R1_esti[1][1] = powl(a, 2);	dObserver.R1_esti[2][2] = powl(a, 2);
						dObserver.R1_esti[3][3] = powl(a, 2); dObserver.R1_esti[4][4] = powl(a, 2);	dObserver.R1_esti[5][5] = powl(a, 2);
						dObserver.R2_esti[0][0] = powl(b, 2); dObserver.R2_esti[1][1] = powl(b, 2);	dObserver.R2_esti[2][2] = powl(b, 2);
						dObserver.R2_esti[3][3] = powl(b, 2); dObserver.R2_esti[4][4] = powl(b, 2);	dObserver.R2_esti[5][5] = powl(b, 2);
						dObserver.R3_esti[0][0] = powl(c, 2); dObserver.R3_esti[1][1] = powl(c, 2);	dObserver.R3_esti[2][2] = powl(c, 2);
						dObserver.R3_esti[3][3] = powl(c, 2); dObserver.R3_esti[4][4] = powl(c, 2);	dObserver.R3_esti[5][5] = powl(c, 2);
					}
				}
			}
			//50s之后开始可以用Rkf
			if (sysc.cnt_s >= dObserver.k_0 + 50) flag_Rkf = 1; else flag_Rkf = 0;
			//IMM 滤波
			IMM(dvlkalman, dObserver, flag_Rkf);

			//高度计 默认一直存在
			depkalman.Mea_vector[0] = infor.pos[2] - dObserver.Depth;
			memset(depkalman.H_matrix, 0, sizeof(depkalman.H_matrix));
			depkalman.H_matrix[0][8] = 1;
			Kal_update_15_1(depkalman, 1);

			//磁罗经 判断是否有磁罗经；联邦滤波
			if (dObserver.flag_C == 0)  //有磁罗经
			{
				cmpkalman.Mea_vector[0] = infor.att_angle[2] - dObserver.Cmp;
				memset(cmpkalman.H_matrix, 0, sizeof(cmpkalman.H_matrix));
				cmpkalman.H_matrix[0][5] = -1;
				Kal_update_15_1(cmpkalman, 1);

				//DVL_Depth_Cmp三个子滤波器联邦滤波
				for (i = 0; i < 15; i++)
				for (j = 0; j < 15; j++)
				{
					Pk_transpose[i][j] = dvlkalman.P_matrix[i][j];
				}
				//计算Pg
				mainv(15, (double *)Pk_transpose);
				memcpy(PC_transpose, cmpkalman.P_matrix, sizeof(cmpkalman.P_matrix));
				mainv(15, (double *)PC_transpose);
				memcpy(PD_transpose, depkalman.P_matrix, sizeof(depkalman.P_matrix));
				mainv(15, (double *)PD_transpose);
				maadd(15, 15, (double *)tmp_Pk, (double *)PC_transpose, (double *)PD_transpose);
				maadd(15, 15, (double *)Pg, (double *)Pk_transpose, (double *)tmp_Pk);
				mainv(15, (double *)Pg);
				//计算Xg
				memcpy(Xk, dvlkalman.X_vector, sizeof(Xk));
				mamul(15, 15, 1, Xk, (double *)Pk_transpose, Xk);
				mamul(15, 15, 1, depkalman.X_vector, (double *)PD_transpose, depkalman.X_vector);
				mamul(15, 15, 1, cmpkalman.X_vector, (double *)PC_transpose, cmpkalman.X_vector);
				vecadd(15, Xg, Xk, depkalman.X_vector);
				vecadd(15, Xg, Xg, cmpkalman.X_vector);
				mamul(15, 15, 1, Xg, (double *)Pg, Xg);
			}
			else
			{   //DVL_Depth两个子滤波器联邦滤波
				for (i = 0; i < 15; i++)
				for (j = 0; j < 15; j++)
				{
					Pk_transpose[i][j] = dvlkalman.P_matrix[i][j];
				}
				//计算Pg
				mainv(15, (double *)Pk_transpose);
				memcpy(PD_transpose, depkalman.P_matrix, sizeof(depkalman.P_matrix));
				mainv(15, (double *)PD_transpose);
				maadd(15, 15, (double *)Pg, (double *)Pk_transpose, (double *)PD_transpose);
				mainv(15, (double *)Pg);
				//计算Xg
				memcpy(Xk, dvlkalman.X_vector, sizeof(Xk));
				mamul(15, 15, 1, Xk, (double *)Pk_transpose, Xk);
				mamul(15, 15, 1, depkalman.X_vector, (double *)PD_transpose, depkalman.X_vector);
				vecadd(15, Xg, Xk, depkalman.X_vector);
				mamul(15, 15, 1, Xg, (double *)Pg, Xg);
			}

			//反馈校正
			vecsub(3, infor.vel_n, infor.vel_n, Xg);
			vecsub(3, infor.pos, infor.pos, Xg + 6);
			X2cnn(cnn, Xg + 3);
			mamul(3, 3, 3, (double *)infor.cnb_mat, (double *)infor.cnb_mat, (double *)cnn);
			maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
			cnb2ang(infor.cnb_mat, infor.att_angle);
			cnb2q(infor.cnb_mat, infor.quart);
			for (i = 0; i < 9; i++)
				Xg[i] = 0.0;
			for (i = 0; i < 3; i++)
			{
				infor.gyro_bias_esti[i] = Xg[12 + i] * 180 / 3.14 * 3600;
				infor.acce_bias_esti[i] = Xg[9 + i] * 1000000 / 9.78;
			}
			memcpy(dvlkalman.X_vector, Xg, sizeof(Xg));
			memcpy(cmpkalman.X_vector, Xg, sizeof(Xg));
			memcpy(depkalman.X_vector, Xg, sizeof(Xg));

			//估计虚拟转台系与车体之间的夹角【一开始车子要尽量匀速直线行驶】
			if (dObserver.kk_ZUPT <= 99)
			{
				dObserver.Vbx_store[dObserver.kk_ZUPT] = infor.vel_b[0]; dObserver.Vby_store[dObserver.kk_ZUPT] = infor.vel_b[1];
				dObserver.kk_ZUPT = dObserver.kk_ZUPT + 1;
				if (dObserver.kk_ZUPT == 100)
				{
					maturn(100, 1, (double *)tmp_Z1, (double *)dObserver.Vby_store);//Vb_store(:,2)'
					vecmul(1, 100, (double *)tmp_Z2, (double *)tmp_Z1, (double *)dObserver.Vby_store);//Vb_store(:,2)'*Vb_store(:,2)
					vecmul(1, 100, (double *)tmp_Z3, (double *)tmp_Z1, (double *)dObserver.Vbx_store);//Vb_store(:,2)'*Vb_store(:,1)
					dObserver.ZUPT_angle = atan(tmp_Z3[0][0] / tmp_Z2[0][0]);
				}
			}
	        break;

/**/    case 3:
			if (dObserver.flag_mode_1 != 3)
			{
				if (dObserver.flag_pureins == 1)//工作在ZUPT模式
				{
					memcpy(zupkalman.X_vector, dvlkalman.X_vector, sizeof(zupkalman.X_vector));
					memset(zupkalman.R_measure, 0, sizeof(zupkalman.R_measure));
					zupkalman.R_measure[0][0] = powl(0.05, 2); zupkalman.R_measure[1][1] = powl(0.05, 2); zupkalman.R_measure[2][2] = powl(10, 2);
					for (i = 0; i < 15; i++)
					for (j = 0; j < 15; j++)
					{
						zupkalman.Q_state[i][j] = dvlkalman.Q_state[i][j];
						zupkalman.P_matrix[i][j] = dvlkalman.P_matrix[i][j];
					}
					for (i = 0; i < 20; i++)
					for (j = 0; j < 6; j++)
					{
						dObserver.wbfb_store[i][j] = 1;
					}
				}
				dObserver.flag_pureinsLong = 0; dObserver.k_0 = sysc.cnt_s;
				dObserver.deltaK = dvlkalman.X_vector[15];//存储进入模式3之前的deltaK
			}
			dObserver.flag_mode_1 = 3; dObserver.flag_current = 1; //水速无法估计
			if (sysc.cnt_s - dObserver.k_0 == 300) dObserver.flag_pureinsLong = 1;

			//判断当前纯捷联模式
			switch (dObserver.flag_pureins)
			{
			case 0:  

				break;
			case 1:
				vecmul(3, 3, infor.vel_b, (double*)infor.cnb_mat, infor.vel_n);
				Zk_ZUPT1[0] = 0; Zk_ZUPT1[1] = 0; Zk_ZUPT1[2] = infor.pos[2] - dObserver.Depth;
				Zk_ZUPT2[0] = infor.vel_b[0] * cos(dObserver.ZUPT_angle) - infor.vel_b[1] * sin(dObserver.ZUPT_angle); Zk_ZUPT2[1] = 0; Zk_ZUPT2[2] = infor.pos[2] - dObserver.Depth;
				Zk_ZUPT3[0] = infor.vel_b[0]; Zk_ZUPT3[1] = infor.vel_b[1]; Zk_ZUPT3[2] = infor.pos[2] - dObserver.Depth;

				memset(zupkalman.H_matrix, 0, sizeof(zupkalman.H_matrix));
				zupkalman.H_matrix[0][0] = infor.cnb_mat[0][0]; zupkalman.H_matrix[0][1] = infor.cnb_mat[0][1]; zupkalman.H_matrix[0][2] = infor.cnb_mat[0][2];
				zupkalman.H_matrix[1][0] = infor.cnb_mat[1][0]; zupkalman.H_matrix[1][1] = infor.cnb_mat[1][1]; zupkalman.H_matrix[1][2] = infor.cnb_mat[1][2];
				zupkalman.H_matrix[0][3] = infor.cnb_mat[0][2] * infor.vel_n[1] - infor.cnb_mat[0][1] * infor.vel_n[2];
				zupkalman.H_matrix[0][4] = infor.cnb_mat[0][0] * infor.vel_n[2] - infor.cnb_mat[0][2] * infor.vel_n[0];
				zupkalman.H_matrix[0][5] = infor.cnb_mat[0][1] * infor.vel_n[0] - infor.cnb_mat[0][0] * infor.vel_n[1];
				zupkalman.H_matrix[1][3] = infor.cnb_mat[1][2] * infor.vel_n[1] - infor.cnb_mat[1][1] * infor.vel_n[2];
				zupkalman.H_matrix[1][4] = infor.cnb_mat[1][0] * infor.vel_n[2] - infor.cnb_mat[1][2] * infor.vel_n[0];
				zupkalman.H_matrix[1][5] = infor.cnb_mat[1][1] * infor.vel_n[0] - infor.cnb_mat[1][0] * infor.vel_n[1];
				zupkalman.H_matrix[2][8] = 1;

				//考察陀螺加表输出值
				for (i = 0; i < 19; i++)
				{
					for (j = 0; j < 6; j++)
					{
						dObserver.wbfb_store[i][j] = dObserver.wbfb_store[i + 1][j];
					}
					sum3 = sum3 + sqrt(powl(dObserver.wbfb_store[i][0], 2) + powl(dObserver.wbfb_store[i][1], 2)+powl(dObserver.wbfb_store[i][2], 2));
					sum4 = sum4 + dObserver.wbfb_store[i][2];
				}
				dObserver.wbfb_store[19][0] = infor.gyro_wib_b[0]; dObserver.wbfb_store[19][1] = infor.gyro_wib_b[1]; dObserver.wbfb_store[19][2] = infor.gyro_wib_b[2];
				dObserver.wbfb_store[19][3] = infor.acce_b[0];     dObserver.wbfb_store[19][4] = infor.acce_b[1];     dObserver.wbfb_store[19][5] = infor.acce_b[2];
				sum3 = sum3 + sqrt(powl(dObserver.wbfb_store[19][0], 2) + powl(dObserver.wbfb_store[19][1], 2) + powl(dObserver.wbfb_store[19][2], 2));
				sum4 = sum4 + dObserver.wbfb_store[19][2];
				sum3 = sum3 / 20; sum4 = sum4 / 20;
				//选择是否ZUPT
				if (sum3 < 0.1*D2R) { memcpy(zupkalman.Mea_vector, Zk_ZUPT3, sizeof(Zk_ZUPT3)); }
				else
				{
					if (sum4 < 0.005) 
					{
						memcpy(zupkalman.Mea_vector, Zk_ZUPT2, sizeof(Zk_ZUPT2));
						for (i = 0; i < 15; i++){ zupkalman.H_matrix[1][i] = 0; }						
					}
					else 
					{
						memcpy(zupkalman.Mea_vector, Zk_ZUPT1, sizeof(Zk_ZUPT1));
						for (i = 0; i < 15; i++){ zupkalman.H_matrix[0][i] = 0; zupkalman.H_matrix[1][i] = 0; }
					}
				}
				//滤波
				Kal_update_15_3(zupkalman, 1);
				//反馈校正 
				vecsub(3, infor.vel_n, infor.vel_n, zupkalman.X_vector);
				vecsub(3, infor.pos, infor.pos, zupkalman.X_vector + 6);
				X2cnn(cnn, zupkalman.X_vector + 3);
				mamul(3, 3, 3, (double *)infor.cnb_mat, (double *)infor.cnb_mat, (double *)cnn);
				maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
				cnb2ang(infor.cnb_mat, infor.att_angle);
				cnb2q(infor.cnb_mat, infor.quart);
				for (i = 0; i < 9; i++)
					zupkalman.X_vector[i] = 0.0;
				for (i = 0; i < 3; i++)
				{
					infor.gyro_bias_esti[i] = zupkalman.X_vector[12 + i] * 180 / 3.14 * 3600;
					infor.acce_bias_esti[i] = zupkalman.X_vector[9 + i] * 1000000 / 9.78;
				}

				break;

			default:break;
			}

			break;

/**/    case 4:     //AI辅助惯性导航方法
			if (dObserver.flag_mode_1 != 4)
			{
				initial_mode0_2(dObserver);//用来初始化模式0和2的函数
				dObserver.flag_pureinsLong = 0; dObserver.k_0 = sysc.cnt_s;
				dObserver.deltaK = dvlkalman.X_vector[15];//存储进入模式3之前的deltaK
			}
			dObserver.flag_mode_1 = 4; dObserver.flag_current = 1; //水速无法估计
			if (sysc.cnt_s - dObserver.k_0 == 400) dObserver.flag_pureinsLong = 1;

			//INS/DVL滤波器
			//AI预测
			INPUT_AI[0] = dObserver.AI_fn_k_1[0] * 10; INPUT_AI[1] = dObserver.AI_fn_k_1[1] * 10;	INPUT_AI[2] = dObserver.AI_fn_k_1[2] * 10;
			INPUT_AI[3] = infor.acce_n[0]*10;          INPUT_AI[4] = infor.acce_n[1] * 10;	        INPUT_AI[5] = infor.acce_n[2] * 10;
			INPUT_AI[6] = dObserver.AI_vns_k_1[0];	   INPUT_AI[7] = dObserver.AI_vns_k_1[1];	    INPUT_AI[8] = dObserver.AI_vns_k_1[2];
			INPUT_AI[9] = infor.vel_n[0];	           INPUT_AI[10] = infor.vel_n[1];			    INPUT_AI[11] = infor.vel_n[2];
			//记录历史输入信息（作为下一时刻的上一时刻数据）
			memcpy(dObserver.AI_fn_k_1, infor.acce_n, sizeof(infor.acce_n));
			memcpy(dObserver.AI_vns_k_1, infor.vel_n, sizeof(infor.vel_n));
			//计算kernal
			for (i = 0; i < 1899; i++) //列号  一共1899组训练数据
			{
				Input_tmp[0] = dObserver.AI_Input[0][i]; Input_tmp[1] = dObserver.AI_Input[1][i]; Input_tmp[2] = dObserver.AI_Input[2][i]; Input_tmp[3] = dObserver.AI_Input[3][i];
				Input_tmp[4] = dObserver.AI_Input[4][i]; Input_tmp[5] = dObserver.AI_Input[5][i]; Input_tmp[6] = dObserver.AI_Input[6][i]; Input_tmp[7] = dObserver.AI_Input[7][i];
				Input_tmp[8] = dObserver.AI_Input[8][i]; Input_tmp[9] = dObserver.AI_Input[9][i]; Input_tmp[10] = dObserver.AI_Input[10][i]; Input_tmp[11] = dObserver.AI_Input[11][i];
				vecsub(12, tmp6, INPUT_AI, Input_tmp);
				s1 = vectormo(tmp6, 12); s2 = -powl(s1, 2) / 2 / dObserver.AI_sig2;
				kernal_AI[0][i] = powl(Exp, s2);
			}
			mamul_3000(1, 1899, 1, (double *)T11, (double *)kernal_AI, (double *)dObserver.AI_a1); //还没加b
			mamul_3000(1, 1899, 1, (double *)T22, (double *)kernal_AI, (double *)dObserver.AI_a2);
			mamul_3000(1, 1899, 1, (double *)T33, (double *)kernal_AI, (double *)dObserver.AI_a3);
			tmp4[0] = T11[0][0] + dObserver.AI_b1[0][0]; tmp4[1] = T22[0][0] + dObserver.AI_b2[0][0]; tmp4[2] = T33[0][0] + dObserver.AI_b3[0][0];
			vecmul(3, 3, tmp5, (double*)infor.cnb_mat, tmp4);//预测值

			vecmul(3, 3, infor.vel_b, (double*)infor.cnb_mat, infor.vel_n);
			dvlkalman.Mea_vector[0] = tmp5[0] / 10; dvlkalman.Mea_vector[1] = tmp5[1] / 10; dvlkalman.Mea_vector[2] = tmp5[2] / 10;
			dvlkalman.Mea_vector[3] = 0; dvlkalman.Mea_vector[4] = 0; dvlkalman.Mea_vector[5] = 0;
			memset(dvlkalman.H_matrix, 0, sizeof(dvlkalman.H_matrix));
			dvlkalman.H_matrix[0][0] = infor.cnb_mat[0][0]; dvlkalman.H_matrix[0][1] = infor.cnb_mat[0][1]; dvlkalman.H_matrix[0][2] = infor.cnb_mat[0][2];
			dvlkalman.H_matrix[1][0] = infor.cnb_mat[1][0]; dvlkalman.H_matrix[1][1] = infor.cnb_mat[1][1]; dvlkalman.H_matrix[1][2] = infor.cnb_mat[1][2];
			dvlkalman.H_matrix[2][0] = infor.cnb_mat[2][0]; dvlkalman.H_matrix[2][1] = infor.cnb_mat[2][1]; dvlkalman.H_matrix[2][2] = infor.cnb_mat[2][2];
			dvlkalman.H_matrix[0][3] = infor.cnb_mat[0][2] * infor.vel_n[1] - infor.cnb_mat[0][1] * infor.vel_n[2];
			dvlkalman.H_matrix[0][4] = infor.cnb_mat[0][0] * infor.vel_n[2] - infor.cnb_mat[0][2] * infor.vel_n[0];
			dvlkalman.H_matrix[0][5] = infor.cnb_mat[0][1] * infor.vel_n[0] - infor.cnb_mat[0][0] * infor.vel_n[1];
			dvlkalman.H_matrix[1][3] = infor.cnb_mat[1][2] * infor.vel_n[1] - infor.cnb_mat[1][1] * infor.vel_n[2];
			dvlkalman.H_matrix[1][4] = infor.cnb_mat[1][0] * infor.vel_n[2] - infor.cnb_mat[1][2] * infor.vel_n[0];
			dvlkalman.H_matrix[1][5] = infor.cnb_mat[1][1] * infor.vel_n[0] - infor.cnb_mat[1][0] * infor.vel_n[1];
			dvlkalman.H_matrix[2][3] = infor.cnb_mat[2][2] * infor.vel_n[1] - infor.cnb_mat[2][1] * infor.vel_n[2];
			dvlkalman.H_matrix[2][4] = infor.cnb_mat[2][0] * infor.vel_n[2] - infor.cnb_mat[2][2] * infor.vel_n[0];
			dvlkalman.H_matrix[2][5] = infor.cnb_mat[2][1] * infor.vel_n[0] - infor.cnb_mat[2][0] * infor.vel_n[1];
			dvlkalman.H_matrix[0][15] = -infor.vel_b[0]; dvlkalman.H_matrix[1][15] = -infor.vel_b[1]; dvlkalman.H_matrix[2][15] = -infor.vel_b[2];
			
			//50s之后开始可以用Rkf
			if (sysc.cnt_s >= dObserver.k_0 + 50) flag_Rkf = 1; else flag_Rkf = 0;
			//IMM 滤波
			IMM(dvlkalman, dObserver, flag_Rkf);

			//高度计 默认一直存在
			depkalman.Mea_vector[0] = infor.pos[2] - dObserver.Depth;
			memset(depkalman.H_matrix, 0, sizeof(depkalman.H_matrix));
			depkalman.H_matrix[0][8] = 1;
			Kal_update_15_1(depkalman, 1);

			//磁罗经 判断是否有磁罗经；联邦滤波
			if (dObserver.flag_C == 0)  //有磁罗经
			{
				cmpkalman.Mea_vector[0] = infor.att_angle[2] - dObserver.Cmp;
				memset(cmpkalman.H_matrix, 0, sizeof(cmpkalman.H_matrix));
				cmpkalman.H_matrix[0][5] = -1;
				Kal_update_15_1(cmpkalman, 1);

				//DVL_Depth_Cmp三个子滤波器联邦滤波
				for (i = 0; i < 15; i++)
				for (j = 0; j < 15; j++)
				{
					Pk_transpose[i][j] = dvlkalman.P_matrix[i][j];
				}
				//计算Pg
				mainv(15, (double *)Pk_transpose);
				memcpy(PC_transpose, cmpkalman.P_matrix, sizeof(cmpkalman.P_matrix));
				mainv(15, (double *)PC_transpose);
				memcpy(PD_transpose, depkalman.P_matrix, sizeof(depkalman.P_matrix));
				mainv(15, (double *)PD_transpose);
				maadd(15, 15, (double *)tmp_Pk, (double *)PC_transpose, (double *)PD_transpose);
				maadd(15, 15, (double *)Pg, (double *)Pk_transpose, (double *)tmp_Pk);
				mainv(15, (double *)Pg);
				//计算Xg
				memcpy(Xk, dvlkalman.X_vector, sizeof(Xk));
				mamul(15, 15, 1, Xk, (double *)Pk_transpose, Xk);
				mamul(15, 15, 1, depkalman.X_vector, (double *)PD_transpose, depkalman.X_vector);
				mamul(15, 15, 1, cmpkalman.X_vector, (double *)PC_transpose, cmpkalman.X_vector);
				vecadd(15, Xg, Xk, depkalman.X_vector);
				vecadd(15, Xg, Xg, cmpkalman.X_vector);
				mamul(15, 15, 1, Xg, (double *)Pg, Xg);
			}
			else
			{   //DVL_Depth两个子滤波器联邦滤波
				for (i = 0; i < 15; i++)
				for (j = 0; j < 15; j++)
				{
					Pk_transpose[i][j] = dvlkalman.P_matrix[i][j];
				}
				//计算Pg
				mainv(15, (double *)Pk_transpose);
				memcpy(PD_transpose, depkalman.P_matrix, sizeof(depkalman.P_matrix));
				mainv(15, (double *)PD_transpose);
				maadd(15, 15, (double *)Pg, (double *)Pk_transpose, (double *)PD_transpose);
				mainv(15, (double *)Pg);
				//计算Xg
				memcpy(Xk, dvlkalman.X_vector, sizeof(Xk));
				mamul(15, 15, 1, Xk, (double *)Pk_transpose, Xk);
				mamul(15, 15, 1, depkalman.X_vector, (double *)PD_transpose, depkalman.X_vector);
				vecadd(15, Xg, Xk, depkalman.X_vector);
				mamul(15, 15, 1, Xg, (double *)Pg, Xg);
			}

			//反馈校正
			vecsub(3, infor.vel_n, infor.vel_n, Xg);
			vecsub(3, infor.pos, infor.pos, Xg + 6);
			X2cnn(cnn, Xg + 3);
			mamul(3, 3, 3, (double *)infor.cnb_mat, (double *)infor.cnb_mat, (double *)cnn);
			maturn(3, 3, (double*)infor.cbn_mat, (double*)infor.cnb_mat);
			cnb2ang(infor.cnb_mat, infor.att_angle);
			cnb2q(infor.cnb_mat, infor.quart);
			for (i = 0; i < 9; i++)
				Xg[i] = 0.0;
			for (i = 0; i < 3; i++)
			{
				infor.gyro_bias_esti[i] = Xg[12 + i] * 180 / 3.14 * 3600;
				infor.acce_bias_esti[i] = Xg[9 + i] * 1000000 / 9.78;
			}
			memcpy(dvlkalman.X_vector, Xg, sizeof(Xg));
			memcpy(cmpkalman.X_vector, Xg, sizeof(Xg));
			memcpy(depkalman.X_vector, Xg, sizeof(Xg));

			break;

		default:break;
		}//switch (dObserver.flag_mode)


	}////每1s进行组合  if (0 == sysc.data_cnt % (sysc.Fs / sysc.Kal_fr))

}
#pragma endregion DP yucia_Kalman