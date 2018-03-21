#include "stdafx.h"
#include "Init_Navigation.h"

//�ṹ�嶨��
PHINS phins;                                         // PHINS �ο���Ϣ
SYS_ELEMENTtransverse inforS;                      //20171123��������
SYS_ELEMENT infor;
OUTIMU IMUout;
COARSE_ALGI c_infor;                                 //�ֶ�׼����
COMPALIGN   cmp;
ADRC_S adrc;
NAVPARA SINSpara;
GPS gps;
CALIPMT calipara;//�궨����
SKALMAN_15_3 fkalman;                     //yyq����׼kalman����    //20171108 
SKALMAN_15_3 nkalman;                     //�������Ƕ�׼�׶Σ���ʼ��kalman���� //20171128 
SKALMAN_16_3 kalman_dvl;                     //�������Ƕ�׼�׶Σ���ʼ��16άkalman���� //20171128 
SKALMAN_16_3 kalman_dvl_transverse;			 //�������Ƕ�׼�׶Σ���ʼ�ĺ�������ϵ��16άkalman���� //20180319
SKALMAN_19_6 dvlkalman;                     //INS/DVL���˲���
SKALMAN_15_1 cmpkalman;                     //INS/Cmp���˲���
SKALMAN_15_1 depkalman;                     //INS/Cmp���˲���
SKALMAN_15_3 zupkalman;                     //ZUPT ���˲���
SDVLCmpDepth dObserver;                     //ˮ����ϵ�������Ϣ�ṹ����� //20180116 
ZTPARA ZT;
INSCAL INScal;
FOSN fosn;
SYSTEMCTRL sysc;
FILTER kal;//�豸�����õ�kalman�˲��ṹ�����
//��������������ʼ��
void init_basicnavi(void)//infor��out_nav��SINSpara��IMUout
{	
	memset(IMUout.gyro_b,0,sizeof(IMUout.gyro_b));
	memset(IMUout.acce_b,0,sizeof(IMUout.acce_b));
	memset(infor.att_angle, 0, sizeof(infor.att_angle));
	memset(infor.vel_n, 0, sizeof(infor.vel_n));
	memset(infor.dvel_n, 0, sizeof(infor.dvel_n));
	memset(infor.old_v, 0, sizeof(infor.old_v));
	SINSpara.gn[0] = 0.0;
	SINSpara.gn[1] = 0.0;
	SINSpara.gn[2] = -latitog(infor.pos[0]);			 // ����ϵ�µ��������ٶ�
	
	memset(infor.gyro_wib_b,0,sizeof(infor.gyro_wib_b));
	memset(infor.gyro_old,0,sizeof(infor.gyro_old));
	memset(infor.acce_b,0,sizeof(infor.acce_b));
	memset(infor.acce_n,0,sizeof(infor.acce_n));
	memset(infor.cnb_mat,0,sizeof(infor.cnb_mat));
	memset(infor.cbn_mat,0,sizeof(infor.cbn_mat));
	memset(infor.gyro_bias_esti, 0, sizeof(infor.gyro_bias_esti));
	memset(infor.acce_bias_esti, 0, sizeof(infor.acce_bias_esti));
	memset(infor.rvib, 0, sizeof(infor.rvib));

	memset(infor.quart, 0, sizeof(infor.quart));
	infor.quart[0] = 1.;

	//�豸�й�
	infor.acce_n_t = 100;
	infor.flagFire = 0;
	infor.device_type = 5;
	infor.flagComps = 0;
	//��ʼ����������
	memset(inforS.att_angle_S, 0, sizeof(inforS.att_angle_S));  
	memset(inforS.vel_S, 0, sizeof(inforS.vel_S));
	memset(inforS.att_angle, 0, sizeof(inforS.att_angle));
	memset(inforS.vel_n, 0, sizeof(inforS.vel_n));
	inforS.lati_S = 0;    
	inforS.longi_S = 0;
	inforS.high_S = 0;
	inforS.lati = 0;
	inforS.longi = 0;
	inforS.high= 0;		
}
//�ֶ�׼��ʼ��
void init_coarsealign(void)	// c_infor
{
	int i,j;
	double sl ;
	double cl;
	double tl ;
	double wie_n[3];
	double g_n[3];
	double cg_wie[3], cg_wie_g[3];

	/* ��׼����������ʼ�� */
	sl = sin(infor.pos[0]);
	cl = cos(infor.pos[0]);
	tl = tan(infor.pos[0]);
	

	for(i = 0; i < 3; i++)
	{
		c_infor.g_sum[i] = 0.;  // �������ٶ���bϵͶӰ���ۼӺ�  
		c_infor.quart_coarse[i+1] = 0.;
	}
	c_infor.quart_coarse[0] = 1;	// ����ϵ����Cbi�����̷���Cb_ib0����ʱʹ�õ���Ԫ��

	//========================�����ֶ�׼������ʼ��=============================//
	wie_n[0] = 0.;
	wie_n[1] = WIE*cl;
	wie_n[2] = WIE*sl;	// �����������ٶ���nϵ��ͶӰ

	
	g_n[1] = g_n[0] = 0.;
	g_n[2] = SINSpara.gn[2]; // �������ٶ���nϵ��ͶӰ

	
	cvecmul(cg_wie,g_n,wie_n);
	cvecmul(cg_wie_g,cg_wie,g_n);

	for(i = 0; i < 3; i++)
	{
		c_infor.cg_wie_mat[0][i] = g_n[i];			
		c_infor.cg_wie_mat[1][i] = cg_wie[i];    
		c_infor.cg_wie_mat[2][i] = cg_wie_g[i];			   
		c_infor.gyro_sum[i] = 0.;  // ��������ۼ�
	}
	mainv(3,(double *)c_infor.cg_wie_mat);


	/*========================����ϵ�ֶ�׼������ʼ��============================*/
	for(i = 0; i < 3; i++)
	{
		c_infor.g_i_k1[i] = g_n[i];					 /* tk1ʱ���������ٶ���iϵ�е�ͶӰ	(Ӧ���ó�ʼʱ�̵ļӱ�����)*/
		c_infor.g_i_k2[i] = 0.;						 /* tk2ʱ���������ٶ���iϵ�е�ͶӰ */
		c_infor.g_i_k3[i] = 0.;						 /* tk3ʱ���������ٶ���iϵ�е�ͶӰ */
		
		
		for(j = 0; j < 3; j++)
		{
			c_infor.Cin[i][j] = 0.;					 /* iϵ��nϵ��ת�ƾ��� */
			c_infor.Cbi_old[i][j] = 0.;				 /* �������ϵ��׼����ʱ�̵��Cbi���� */

			if(i == j)
			{
				c_infor.Cbi[i][j] = 1;				 /* bϵ��iϵ��ת�ƾ��� */
				c_infor.Cib[i][j] = 1;				 /* Cbi��ת�� */
			}
			else
			{
				c_infor.Cbi[i][j] = 0.;
				c_infor.Cib[i][j] = 0.;
			}
		}  /* ��������ϵi���ʼʱ��t0��������ϵ���غϣ�Cbi(t0) = Cib(t0) = I*/
	}
	
	

	/*========================���̽����ֶ�׼������ʼ��==========================*/
	for(i = 0; i < 3; i++)
	{
		c_infor.f_sum[i] = 0.;  /* �ӱ�����ۼ�ֵ*/
		c_infor.v_sum[i] = 0.;  /* �ӱ����ֵ��ib0ϵͶӰ�Ļ���ֵ*/
		c_infor.f_ib0_k1[i] = 0.;  /* t0��tk1ʱ��μӱ����ֵ��ib0ϵͶӰ�Ļ���ֵ*/
		c_infor.f_ib0_k2[i] = 0.;  /* t0��tk2ʱ��μӱ����ֵ��ib0ϵͶӰ�Ļ���ֵ*/
		c_infor.g_i0_k1[i] = 0.;	/* t0��tk1ʱ����������ٶ���i0ϵͶӰ�Ļ���ֵ*/
		c_infor.g_i0_k2[i] = 0.;	/* t0��tk2ʱ����������ٶ���i0ϵͶӰ�Ļ���ֵ*/

		c_infor.avg_att[3] = 0.;;  /*�������ʱ��εĳ�ʼ��̬��ֵ */

		for(j = 0; j < 3; j++)
		{
			c_infor.Cib0_i0[i][j] = 0.;  /* ib0ϵ��i0ϵת�ƾ���*/
			c_infor.Cb_ib0_old[i][j] = 0.; /* �������̽����ֶ�׼����ʱ�̵�Cb_ib0����*/
			
			if(i == j)
			{
				c_infor.Cb_ib0[i][j] = 1;  /* bϵ��ib0ϵת�ƾ���*/
				c_infor.Cib0_b[i][j] = 1;  /* Cb_ib0ת��*/
				c_infor.Ci0_e[i][j] = 1;	/* i0ϵ��eϵת�ƾ���*/
			}
			else
			{
				c_infor.Cb_ib0[i][j] = 0.;
				c_infor.Cib0_b[i][j] = 0.;
				c_infor.Ci0_e[i][j] = 0.;
			}  /*��������ϵi���ʼʱ��t0��������ϵ���غϣ�Cb_ib0(t0) = Cib0_b(t0) = I*/
		}
	}
	

	c_infor.Ce_n[0][0] = c_infor.Ce_n[0][2] = c_infor.Ce_n[1][1] = c_infor.Ce_n[2][1] = 0.;
	c_infor.Ce_n[0][1] = 1;
	c_infor.Ce_n[1][0] = -sl;
	c_infor.Ce_n[1][2] = cl;
	c_infor.Ce_n[2][0] = cl;
	c_infor.Ce_n[2][2] = sl;	/* eϵ��nϵת�ƾ���*/
}	
//�޾�����׼������ʼ��
void init_cmp(void)//cmp
{
    int i;
	for(i=0;i<3;i++)
	{
	   cmp.wc_n[i] = 0.0;
	   cmp.wc_b[i] = 0.0;
	   cmp.fc_n[i] = 0.0;
	   cmp.fc_b[i] = 0.0; 
	}
	//ˮƽ��׼����
 
    cmp.k1[0] = 0.18;//0.06;
	cmp.k1[1] = 4680;//1280;//2340;
	cmp.k1[2] = 0.74646;

	//��λ��׼����
	cmp.k2[0] = 0.12;
	cmp.k2[1] = 2560;
	cmp.k2[2] = 6.25e-8;
	cmp.k2[3] = 1.3;
	cmp.k2[4] = -5.15e-6;
}
//�Կ��Ŷ�׼������ʼ��
void init_adrc(void)
{
	adrc.b0=1;
	adrc.h=sysc.Ts;
	adrc.h2=2*adrc.h;
	adrc.delta=5*adrc.h;
	adrc.b01=150;
	adrc.b02=900;
	adrc.b03=2500;
	adrc.b11=280;
	adrc.b12=1700;
	adrc.b13=2400;
	adrc.b14=0.1;
	adrc.p1=2.5;
	adrc.p2=25;
	adrc.pf=1.5;
	adrc.p11=0.7;
	adrc.p12=40;
	adrc.p13=3;
	adrc.y1e=0;
	adrc.y1n=0;
	adrc.z1e=0;
	adrc.z2e=0;
	adrc.z3e=0;
	adrc.z1n=0;
	adrc.z2n=0;
	adrc.z3n=0;
	adrc.z4n=0;
	adrc.uu=0;
	adrc.ue=0;
	adrc.un=0;
	adrc.ufe=0;
	adrc.ufn=0;
	adrc.azi1=2560;
	adrc.azi2=10e-5;
	adrc.azi3=16;
	adrc.azi4=0.03;
	adrc.azi5=0.4;
}
//������˫λ�þ���׼��ʼ��(15άλ��ƥ��)��ģʽ����ʲô���//0~2�ٶ���3~5��̬��9~11�Ǽӱ�ֵ��12~14�����ݳ�ֵ

void Kal_Init_P_15(SKALMAN_15_3& temp_kal,char mode)              //20171108
{
	memset(temp_kal.P_matrix, 0, sizeof(temp_kal.P_matrix));

	temp_kal.P_matrix[0][0] = powl(1, 2);
	temp_kal.P_matrix[1][1] = temp_kal.P_matrix[0][0];
	temp_kal.P_matrix[2][2] = temp_kal.P_matrix[0][0];

	temp_kal.P_matrix[3][3] = powl(1 * D2R, 2);
	temp_kal.P_matrix[4][4] = temp_kal.P_matrix[3][3];
	temp_kal.P_matrix[5][5] = powl(10 * D2R, 2);

	temp_kal.P_matrix[6][6] = powl(10.0 / RE, 2);
	temp_kal.P_matrix[7][7] = temp_kal.P_matrix[6][6];
	temp_kal.P_matrix[8][8] = powl(5, 2);

	temp_kal.P_matrix[9][9] = powl(1000 * ug, 2);
	temp_kal.P_matrix[10][10] = temp_kal.P_matrix[9][9];
	temp_kal.P_matrix[11][11] = temp_kal.P_matrix[9][9];

	temp_kal.P_matrix[12][12] = powl(0.02*dph, 2);
	temp_kal.P_matrix[13][13] = temp_kal.P_matrix[12][12];
	temp_kal.P_matrix[14][14] = temp_kal.P_matrix[12][12];

	memset(temp_kal.Q_state, 0, sizeof(temp_kal.Q_state));

	temp_kal.Q_state[0][0] = powl(50 * ug, 2);         //20171213 ����ӦΪ50ug!!!!!�����Ұ��죡
	temp_kal.Q_state[1][1] = temp_kal.Q_state[0][0];
	temp_kal.Q_state[2][2] = temp_kal.Q_state[0][0];

	temp_kal.Q_state[3][3] = powl(0.01*dph, 2);
	temp_kal.Q_state[4][4] = temp_kal.Q_state[3][3];
	temp_kal.Q_state[5][5] = temp_kal.Q_state[3][3];

	memset(temp_kal.R_measure, 0, sizeof(temp_kal.R_measure));
	memset(temp_kal.H_matrix, 0, sizeof(temp_kal.H_matrix));
	if (mode == YA_POS)
	{
		temp_kal.H_matrix[0][6] = 1;
		temp_kal.H_matrix[1][7] = 1;
		temp_kal.H_matrix[2][8] = 1;
		temp_kal.R_measure[0][0] = powl(10 / RE, 2);
		temp_kal.R_measure[1][1] = powl(10 / RE, 2);
		temp_kal.R_measure[2][2] = powl(10, 2);

	}
	if (mode == YA_VEL)
	{
		temp_kal.H_matrix[0][0] = 1;
		temp_kal.H_matrix[1][1] = 1;
		temp_kal.H_matrix[2][2] = 1;

		temp_kal.R_measure[0][0] = powl(0.1, 2);
		temp_kal.R_measure[1][1] = powl(0.1, 2);
		temp_kal.R_measure[2][2] = powl(0.1, 2);

	}
	if (mode == YA_VELANDAZ)
	{
		temp_kal.H_matrix[0][0] = 1;
		temp_kal.H_matrix[1][1] = 1;
		temp_kal.H_matrix[2][5] = -1;

		temp_kal.R_measure[0][0] = powl(0.1, 2);
		temp_kal.R_measure[1][1] = powl(0.1, 2);
		temp_kal.R_measure[2][2] = powl(0.5*D2R, 2);
	}
	memset(temp_kal.X_vector, 0, sizeof(temp_kal.X_vector));
}

//INS/DVL��ϳ�ʼ������
void Kal_Init_P_16(SKALMAN_16_3& temp_kal)              //   20171128
{
	memset(temp_kal.P_matrix, 0, sizeof(temp_kal.P_matrix));

	temp_kal.P_matrix[0][0] = powl(0.1, 2);
	temp_kal.P_matrix[1][1] = temp_kal.P_matrix[0][0];
	temp_kal.P_matrix[2][2] = temp_kal.P_matrix[0][0];

	temp_kal.P_matrix[3][3] = powl(0.01 * D2R, 2);
	temp_kal.P_matrix[4][4] = temp_kal.P_matrix[3][3];
	temp_kal.P_matrix[5][5] = powl(0.1 * D2R, 2);

	temp_kal.P_matrix[6][6] = powl(10.0 / RE, 2);
	temp_kal.P_matrix[7][7] = temp_kal.P_matrix[6][6];
	temp_kal.P_matrix[8][8] = powl(5, 2);

	temp_kal.P_matrix[9][9] = powl(100 * ug, 2);
	temp_kal.P_matrix[10][10] = temp_kal.P_matrix[9][9];
	temp_kal.P_matrix[11][11] = temp_kal.P_matrix[9][9];

	temp_kal.P_matrix[12][12] = powl(0.02*dph, 2);
	temp_kal.P_matrix[13][13] = temp_kal.P_matrix[12][12];
	temp_kal.P_matrix[14][14] = temp_kal.P_matrix[12][12];

	temp_kal.P_matrix[15][15] = powl(0.4, 2);   //DVL �̶����Ӳ���
	 
	memset(temp_kal.Q_state, 0, sizeof(temp_kal.Q_state));

	temp_kal.Q_state[0][0] = powl(50 * ug, 2);
	temp_kal.Q_state[1][1] = temp_kal.Q_state[0][0];
	temp_kal.Q_state[2][2] = temp_kal.Q_state[0][0];

	temp_kal.Q_state[3][3] = powl(0.01*dph, 2);
	temp_kal.Q_state[4][4] = temp_kal.Q_state[3][3];
	temp_kal.Q_state[5][5] = temp_kal.Q_state[3][3];

	memset(temp_kal.R_measure, 0, sizeof(temp_kal.R_measure));

	temp_kal.R_measure[0][0] = powl(0.1, 2);
	temp_kal.R_measure[1][1] = powl(0.1, 2);
	temp_kal.R_measure[2][2] = powl(0.1, 2);

	memset(temp_kal.H_matrix, 0, sizeof(temp_kal.H_matrix));
	memset(temp_kal.X_vector, 0, sizeof(temp_kal.X_vector));

}
//INS/DVL_19ά��ʼ������
void Kal_Init_DVL_19 (SKALMAN_19_6& temp_kal)              //   20180116
{
	memset(temp_kal.P_matrix, 0, sizeof(temp_kal.P_matrix));

	temp_kal.P_matrix[0][0] = powl(0.1, 2);
	temp_kal.P_matrix[1][1] = temp_kal.P_matrix[0][0];
	temp_kal.P_matrix[2][2] = temp_kal.P_matrix[0][0];

	temp_kal.P_matrix[3][3] = powl(0.01 * D2R, 2);
	temp_kal.P_matrix[4][4] = temp_kal.P_matrix[3][3];
	temp_kal.P_matrix[5][5] = powl(0.1 * D2R, 2);

	temp_kal.P_matrix[6][6] = powl(10.0 / RE, 2);
	temp_kal.P_matrix[7][7] = temp_kal.P_matrix[6][6];
	temp_kal.P_matrix[8][8] = powl(10, 2);

	temp_kal.P_matrix[9][9] = powl(100 * ug, 2);
	temp_kal.P_matrix[10][10] = temp_kal.P_matrix[9][9];
	temp_kal.P_matrix[11][11] = temp_kal.P_matrix[9][9];

	temp_kal.P_matrix[12][12] = powl(0.02*dph, 2);
	temp_kal.P_matrix[13][13] = temp_kal.P_matrix[12][12];
	temp_kal.P_matrix[14][14] = temp_kal.P_matrix[12][12];

	temp_kal.P_matrix[15][15] = powl(0., 2);   //DVL �̶����Ӳ���

	temp_kal.P_matrix[16][16] = powl(5, 2);   //ˮ���ٶ�
	temp_kal.P_matrix[17][17] = powl(5, 2);   //ˮ���ٶ�
	temp_kal.P_matrix[18][18] = powl(5, 2);   //ˮ���ٶ�


	memset(temp_kal.Q_state, 0, sizeof(temp_kal.Q_state));

	temp_kal.Q_state[0][0] = powl(50 * ug, 2);
	temp_kal.Q_state[1][1] = temp_kal.Q_state[0][0];
	temp_kal.Q_state[2][2] = temp_kal.Q_state[0][0];

	temp_kal.Q_state[3][3] = powl(0.01*dph, 2);
	temp_kal.Q_state[4][4] = temp_kal.Q_state[3][3];
	temp_kal.Q_state[5][5] = temp_kal.Q_state[3][3];

	temp_kal.Q_state[16][16] = powl(0.0001, 2);
	temp_kal.Q_state[17][17] = temp_kal.Q_state[16][16];
	temp_kal.Q_state[18][18] = temp_kal.Q_state[16][16];

	memset(temp_kal.R_measure, 0, sizeof(temp_kal.R_measure));

	temp_kal.R_measure[0][0] = powl(0.1, 2);  //IMM ʱ��R��ֵû�á�
	temp_kal.R_measure[1][1] = powl(0.1, 2);
	temp_kal.R_measure[2][2] = powl(0.1, 2);
	temp_kal.R_measure[3][3] = powl(0.1, 2);  
	temp_kal.R_measure[4][4] = powl(0.1, 2);
	temp_kal.R_measure[5][5] = powl(0.1, 2);

	memset(temp_kal.H_matrix, 0, sizeof(temp_kal.H_matrix));
	memset(temp_kal.X_vector, 0, sizeof(temp_kal.X_vector));

}
//INS/Cmp 15ά��ʼ������
void Kal_Init_Cmp_15(SKALMAN_15_1& temp_kal)              //20180116
{
	memset(temp_kal.P_matrix, 0, sizeof(temp_kal.P_matrix));

	temp_kal.P_matrix[0][0] = powl(0.1, 2);
	temp_kal.P_matrix[1][1] = temp_kal.P_matrix[0][0];
	temp_kal.P_matrix[2][2] = temp_kal.P_matrix[0][0];

	temp_kal.P_matrix[3][3] = powl(0.01 * D2R, 2);
	temp_kal.P_matrix[4][4] = temp_kal.P_matrix[3][3];
	temp_kal.P_matrix[5][5] = powl(0.1 * D2R, 2);

	temp_kal.P_matrix[6][6] = powl(10.0 / RE, 2);
	temp_kal.P_matrix[7][7] = temp_kal.P_matrix[6][6];
	temp_kal.P_matrix[8][8] = powl(10, 2);

	temp_kal.P_matrix[9][9] = powl(100 * ug, 2);
	temp_kal.P_matrix[10][10] = temp_kal.P_matrix[9][9];
	temp_kal.P_matrix[11][11] = temp_kal.P_matrix[9][9];

	temp_kal.P_matrix[12][12] = powl(0.02*dph, 2);
	temp_kal.P_matrix[13][13] = temp_kal.P_matrix[12][12];
	temp_kal.P_matrix[14][14] = temp_kal.P_matrix[12][12];

	memset(temp_kal.Q_state, 0, sizeof(temp_kal.Q_state));

	temp_kal.Q_state[0][0] = powl(50 * ug, 2);        
	temp_kal.Q_state[1][1] = temp_kal.Q_state[0][0];
	temp_kal.Q_state[2][2] = temp_kal.Q_state[0][0];

	temp_kal.Q_state[3][3] = powl(0.01*dph, 2);
	temp_kal.Q_state[4][4] = temp_kal.Q_state[3][3];
	temp_kal.Q_state[5][5] = temp_kal.Q_state[3][3];

	memset(temp_kal.R_measure, 0, sizeof(temp_kal.R_measure));
	temp_kal.R_measure[0][0] = powl(0.01, 2);  
	memset(temp_kal.H_matrix, 0, sizeof(temp_kal.H_matrix));
	memset(temp_kal.X_vector, 0, sizeof(temp_kal.X_vector));
}
//INS/Depth 15ά��ʼ������
void Kal_Init_Depth_15(SKALMAN_15_1& temp_kal)              //20180116
{
	memset(temp_kal.P_matrix, 0, sizeof(temp_kal.P_matrix));

	temp_kal.P_matrix[0][0] = powl(0.1, 2);
	temp_kal.P_matrix[1][1] = temp_kal.P_matrix[0][0];
	temp_kal.P_matrix[2][2] = temp_kal.P_matrix[0][0];

	temp_kal.P_matrix[3][3] = powl(0.01 * D2R, 2);
	temp_kal.P_matrix[4][4] = temp_kal.P_matrix[3][3];
	temp_kal.P_matrix[5][5] = powl(0.1 * D2R, 2);

	temp_kal.P_matrix[6][6] = powl(10.0 / RE, 2);
	temp_kal.P_matrix[7][7] = temp_kal.P_matrix[6][6];
	temp_kal.P_matrix[8][8] = powl(10, 2);

	temp_kal.P_matrix[9][9] = powl(100 * ug, 2);
	temp_kal.P_matrix[10][10] = temp_kal.P_matrix[9][9];
	temp_kal.P_matrix[11][11] = temp_kal.P_matrix[9][9];

	temp_kal.P_matrix[12][12] = powl(0.02*dph, 2);
	temp_kal.P_matrix[13][13] = temp_kal.P_matrix[12][12];
	temp_kal.P_matrix[14][14] = temp_kal.P_matrix[12][12];

	memset(temp_kal.Q_state, 0, sizeof(temp_kal.Q_state));

	temp_kal.Q_state[0][0] = powl(50 * ug, 2);         
	temp_kal.Q_state[1][1] = temp_kal.Q_state[0][0];
	temp_kal.Q_state[2][2] = temp_kal.Q_state[0][0];

	temp_kal.Q_state[3][3] = powl(0.01*dph, 2);
	temp_kal.Q_state[4][4] = temp_kal.Q_state[3][3];
	temp_kal.Q_state[5][5] = temp_kal.Q_state[3][3];

	memset(temp_kal.R_measure, 0, sizeof(temp_kal.R_measure));
	temp_kal.R_measure[0][0] = powl(0.1, 2);
	memset(temp_kal.H_matrix, 0, sizeof(temp_kal.H_matrix));
	memset(temp_kal.X_vector, 0, sizeof(temp_kal.X_vector));
}
//INS/ZUPT 15ά��ʼ������
void Kal_Init_ZUPT_15(SKALMAN_15_3& temp_kal)              //20180116
{
	memset(temp_kal.P_matrix, 0, sizeof(temp_kal.P_matrix));

	temp_kal.P_matrix[0][0] = powl(0.1, 2);
	temp_kal.P_matrix[1][1] = temp_kal.P_matrix[0][0];
	temp_kal.P_matrix[2][2] = temp_kal.P_matrix[0][0];

	temp_kal.P_matrix[3][3] = powl(0.01 * D2R, 2);
	temp_kal.P_matrix[4][4] = temp_kal.P_matrix[3][3];
	temp_kal.P_matrix[5][5] = powl(0.1 * D2R, 2);

	temp_kal.P_matrix[6][6] = powl(10.0 / RE, 2);
	temp_kal.P_matrix[7][7] = temp_kal.P_matrix[6][6];
	temp_kal.P_matrix[8][8] = powl(10, 2);

	temp_kal.P_matrix[9][9] = powl(100 * ug, 2);
	temp_kal.P_matrix[10][10] = temp_kal.P_matrix[9][9];
	temp_kal.P_matrix[11][11] = temp_kal.P_matrix[9][9];

	temp_kal.P_matrix[12][12] = powl(0.02*dph, 2);
	temp_kal.P_matrix[13][13] = temp_kal.P_matrix[12][12];
	temp_kal.P_matrix[14][14] = temp_kal.P_matrix[12][12];

	memset(temp_kal.Q_state, 0, sizeof(temp_kal.Q_state));

	temp_kal.Q_state[0][0] = powl(50 * ug, 2);
	temp_kal.Q_state[1][1] = temp_kal.Q_state[0][0];
	temp_kal.Q_state[2][2] = temp_kal.Q_state[0][0];

	temp_kal.Q_state[3][3] = powl(0.01*dph, 2);
	temp_kal.Q_state[4][4] = temp_kal.Q_state[3][3];
	temp_kal.Q_state[5][5] = temp_kal.Q_state[3][3];

	memset(temp_kal.R_measure, 0, sizeof(temp_kal.R_measure));
	temp_kal.R_measure[0][0] = powl(0.05, 2);  
	temp_kal.R_measure[1][1] = powl(0.05, 2);
	temp_kal.R_measure[2][2] = powl(10, 2);
	memset(temp_kal.H_matrix, 0, sizeof(temp_kal.H_matrix));
	memset(temp_kal.X_vector, 0, sizeof(temp_kal.X_vector));
}
//�豸�����õ�kalman�˲���ʼ��
void kalinitial()
{
	int	i, j;

	for (i = 0; i < sta_num; i++)
		for (j = 0; j < sta_num; j++)
			kal.P_matrix[i][j] = 0.0;

	kal.P_matrix[0][0] = pow(0.1, 2);
	kal.P_matrix[1][1] = kal.P_matrix[0][0];
	kal.P_matrix[2][2] = pow(1.5*D2R, 2);
	kal.P_matrix[3][3] = pow(15 * D2R, 2);//kal.P_matrix[2][2];
	kal.P_matrix[4][4] = kal.P_matrix[2][2];
	kal.P_matrix[5][5] = pow(0.00254*D2R, 2);
	kal.P_matrix[6][6] = pow(0.00446*D2R, 2);
	kal.P_matrix[7][7] = pow(100 * GRAVITY*0.000001, 2);
	kal.P_matrix[8][8] = kal.P_matrix[7][7];
	kal.P_matrix[9][9] = pow(8 * D2R / 60 / 60, 2);
	kal.P_matrix[10][10] = kal.P_matrix[9][9];
	kal.P_matrix[11][11] = pow(10 * D2R / 60 / 60, 2);//kal.P_matrix[9][9];

	for (i = 0; i < sta_num; i++)
		for (j = 0; j < sta_num; j++)
			kal.Q_state[i][j] = 0.0;

	kal.Q_state[0][0] = pow(50 * GRAVITY*0.000001, 2);
	kal.Q_state[1][1] = kal.Q_state[0][0];
	kal.Q_state[2][2] = pow(10 * D2R / 3600, 2);
	kal.Q_state[3][3] = kal.Q_state[2][2];
	kal.Q_state[4][4] = kal.Q_state[2][2];
	kal.Q_state[5][5] = kal.Q_state[0][0];
	kal.Q_state[6][6] = kal.Q_state[0][0];

	for (i = 0; i < mea_num; i++)
		for (j = 0; j < mea_num; j++)
			kal.R_measure[i][j] = 0.0;

	kal.R_measure[0][0] = pow(1.5, 2);	//pow(0.5,2);// zhangtao huludao 7.9
	kal.R_measure[1][1] = pow(1.5, 2);	 //pow(0.5,2);//kal.R_measure[0][0]; huludao 7.9
										 //	kal.R_measure[2][2] = pow(1.5*D2R,2);
										 //	kal.R_measure[2][2] = pow(0.5*D2R,2);
	kal.R_measure[2][2] = pow(3 * D2R, 2);	//Xu and Zhang and LIU 20130415 chongqing

	for (i = 0; i < mea_num; i++)
		for (j = 0; j < sta_num; j++)
			kal.H_matrix[i][j] = 0.0;

	kal.H_matrix[0][0] = 1;
	kal.H_matrix[1][1] = 1;
	kal.H_matrix[2][4] = -1;//ʧ׼��=��̬���ǣ�����

	for (i = 0; i < sta_num; i++)
		for (j = 0; j < sta_num; j++)
			kal.F_sum[i][j] = 0.0;

	for (i = 0; i < sta_num; i++)
		for (j = 0; j < sta_num; j++)
			kal.F_kal[i][j] = 0.0;

	for (i = 0; i < sta_num; i++)
		kal.X_vector[i] = 0.0;

	kal.gyro_comps[0] = 0.0 * D2R / 3600.0;
	kal.gyro_comps[1] = 0.0 * D2R / 3600.0;
	kal.gyro_comps[2] = 0.0 * D2R / 3600.0;
}