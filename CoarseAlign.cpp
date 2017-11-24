#include "stdafx.h"
#include "CoarseAlign.h"

/**
* @brief  coarse_ng 凝固解析粗对准
* @param  gyro      浮点型指针 上一次陀螺输出值
* @retval None
*/
void coarse_ng()
{
	//-----------------------------------------------
	// 凝固解析粗对准的计算时刻点选为对准中间时刻点
	// ib0系：初始时刻t0与载体坐标系b重合的惯性坐标系
	// i0系：初始时刻t0与地球坐标系e重合的惯性坐标系
	// ib0系：初始时刻t0与载体坐标系b重合的惯性坐标系
	// Cbn = Cen * Ci0e(t) * Cib0_i0 * Cb_ib0(t)
	//-----------------------------------------------

	int i;
	double deltsita[3];
	double g = -SINSpara.gn[2];                      // 取导航坐标系下绝对g值
	sysc.coarse_cnt++;

	//==========================利用毕卡算法更新Cb_ib0矩阵====================================//
	for(i = 0; i < 3; i++)
	{
		infor.gyro_wib_b[i] = (infor.gyro_wib_b[i] + infor.gyro_old[i]) * 0.5;  // 该处梯形算法需要统一；
		deltsita[i] = infor.gyro_wib_b[i] * sysc.Ts;
	}
	qcal(deltsita, c_infor.quart_coarse);
	optq(c_infor.quart_coarse);
	q2cnb(c_infor.Cib0_b,c_infor.quart_coarse);
	maturn(3,3,(double *)c_infor.Cb_ib0, (double *)c_infor.Cib0_b);	
	// 注意此时的加表数据映射到ib0上，用来积分得到Vib0
	mamul(3,3,1,(double *)infor.acce_b,(double *)c_infor.Cb_ib0,(double *)infor.acce_b);
	
	for(i = 0; i < 3; i++)
	{
		c_infor.v_sum[i] = c_infor.v_sum[i] + infor.acce_b[i] * sysc.Ts;  // 加表输出值在ib0系投影的积分值
	}

	// k1时刻为中间时刻
	if(sysc.coarse_time * sysc.Fs/2 == sysc.coarse_cnt)//*200/2
	{
		avecmul(3,c_infor.f_ib0_k1,c_infor.v_sum,1);  // t0到tk1时间段加表输出值在ib0系投影的积分值

		c_infor.g_i0_k1[0] = g * cos(infor.pos[0]) * sin(WIE * sysc.coarse_cnt * sysc.Ts) * 13713.61291673734;  // 此处直接除以WIE会出现计算错误
		c_infor.g_i0_k1[1] = g * cos(infor.pos[0]) * (1-cos(WIE * sysc.coarse_cnt * sysc.Ts)) * 13713.61291673734;  // 此处直接除以WIE会出现计算错误
		c_infor.g_i0_k1[2] = g * sin(infor.pos[0]) * sysc.coarse_cnt * sysc.Ts;  // t0到tk1时间段重力加速度在i0系投影的积分值
	}

	// tk2为对准结束时刻
	if(sysc.coarse_time* sysc.Fs == sysc.coarse_cnt)
	{
		double g_tmp[3], g_tmp1[3], f_tmp[3], f_tmp1[3];
		double v_i0_mat[3][3], v_ib0_mat[3][3], tmp_mat[3][3];

		avecmul(3,c_infor.f_ib0_k2,c_infor.v_sum,1);  // t0到tk2时间段加表输出值在ib0系投影的积分值

		c_infor.g_i0_k2[0] = g * cos(infor.pos[0]) * sin(WIE * sysc.coarse_cnt * sysc.Ts) * 13713.61291673734;  // 此处直接除以WIE会出现计算错误
		c_infor.g_i0_k2[1] = g * cos(infor.pos[0]) * (1-cos(WIE * sysc.coarse_cnt * sysc.Ts)) * 13713.61291673734;  // 此处直接除以WIE会出现计算错误
		c_infor.g_i0_k2[2] = g * sin(infor.pos[0]) * sysc.coarse_cnt * sysc.Ts;  // t0到tk2时间段重力加速度在i0系投影的积分值

		cvecmul(g_tmp,c_infor.g_i0_k1,c_infor.g_i0_k2);
		cvecmul(f_tmp,c_infor.f_ib0_k1,c_infor.f_ib0_k2);

		cvecmul(g_tmp1,g_tmp,c_infor.g_i0_k1);
		cvecmul(f_tmp1,f_tmp,c_infor.f_ib0_k1);

		
		c_infor.Ci0_e[0][0] = cos(WIE * sysc.coarse_cnt * sysc.Ts);
		c_infor.Ci0_e[0][1] = sin(WIE * sysc.coarse_cnt * sysc.Ts);
		c_infor.Ci0_e[1][0] = -sin(WIE * sysc.coarse_cnt * sysc.Ts);
		c_infor.Ci0_e[1][1] = cos(WIE * sysc.coarse_cnt * sysc.Ts);
		
		for(i = 0; i < 3; i++)
		{
			v_i0_mat[0][i] = c_infor.g_i0_k1[i];  //         [         (g_i0_k1)'       ]
			v_i0_mat[1][i] = g_tmp[i];        //v_i0_mat = [     (g_i0_k1×g_i0_k2)'   ]
			v_i0_mat[2][i] = g_tmp1[i];  //                [(g_i0_k1×g_i0_k2×g_i0_k1)']

			v_ib0_mat[0][i] = c_infor.f_ib0_k1[i];  //        [         (f_ib0_k1)'         ]
			v_ib0_mat[1][i] = f_tmp[i];       //v_ib0_mat = [     (f_ib0_k1×f_ib0_k2)'    ]
			v_ib0_mat[2][i] = f_tmp1[i];  //                [(f_ib0_k1×f_ib0_k2×f_ib0_k1)']
		}

		mainv(3,(double *) v_i0_mat);  // Cib0_i0 = inv(v_i0_mat) * v_ib0_mat
		mamul(3,3,3,(double *)c_infor.Cib0_i0,(double *)v_i0_mat,(double *)v_ib0_mat);
	//===============================================================================//


		//================================计算初始姿态矩阵===================================//
//		mamul(3,3,3,(double *)tmp_mat,(double *)infor.Cib0_i0,(double *)infor.Cb_ib0_old);
		mamul(3,3,3,(double *)tmp_mat,(double *)c_infor.Cib0_i0,(double *)c_infor.Cb_ib0);
		mamul(3,3,3,(double *)tmp_mat,(double *)c_infor.Ci0_e,(double *)tmp_mat);
		mamul(3,3,3,(double *)infor.cbn_mat,(double *)c_infor.Ce_n,(double *)tmp_mat);

		maturn(3,3,(double *)infor.cnb_mat,(double *)infor.cbn_mat);
		cnb2ang(infor.cnb_mat ,infor.att_angle);
		//===================================================================================//

		sysc.coarse_cnt = 0;  // 对准结束，计数清零

		//===============================初始化捷联结算====================================//
		infor.quart[0]=cos(infor.att_angle[2]/2)*cos(infor.att_angle[0]/2)*cos(infor.att_angle[1]/2)-sin(infor.att_angle[2]/2)*sin(infor.att_angle[0]/2)*sin(infor.att_angle[1]/2);
		infor.quart[1]=cos(infor.att_angle[2]/2)*sin(infor.att_angle[0]/2)*cos(infor.att_angle[1]/2)-sin(infor.att_angle[2]/2)*cos(infor.att_angle[0]/2)*sin(infor.att_angle[1]/2);
		infor.quart[2]=cos(infor.att_angle[2]/2)*cos(infor.att_angle[0]/2)*sin(infor.att_angle[1]/2)+sin(infor.att_angle[2]/2)*sin(infor.att_angle[0]/2)*cos(infor.att_angle[1]/2);
		infor.quart[3]=sin(infor.att_angle[2]/2)*cos(infor.att_angle[0]/2)*cos(infor.att_angle[1]/2)+cos(infor.att_angle[2]/2)*sin(infor.att_angle[0]/2)*sin(infor.att_angle[1]/2);
		optq(infor.quart);

		q2cnb(infor.cnb_mat,infor.quart);

		maturn(3,3,(double *)infor.cbn_mat,(double *)infor.cnb_mat);
		sysc.f_coarse_over = 1;
		//=================================================================================//
	}
}  // end of function coarse_ng