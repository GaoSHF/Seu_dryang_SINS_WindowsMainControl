#include <cstring>
#pragma once 

#define  PAI             3.1415926535
#define  D2R             0.017453292519943           // �ǶȻ�Ϊ����
#define  R2D             57.295779513082323          // ���Ȼ��Ƕ�
#define  WIE             7.2921151467E-5             // 15.04088*D2R/3600.0
#define  RE              6.378137e6L                 // ����뾶
#define  AEE             1.0/298.257223563           // ����
#define  E1              0.08181922                  //��һƫ����
#define  E2              0.00669438                  //��һƫ����
#define  EI              1e-6
#define  Exp             2.71828182845904523536      //��Ȼָ��   20180117

#define  GRAVITY         9.7803267714                //g
#define  ug              GRAVITY*0.000001            //ug
#define	 LIGHTSPEED      2.99792458e8                //����
#define  dph             D2R/3600                    //��/h->deg/s

#define  LOOSE           1                           // GPS �������� �����
#define  TIGHT           2                           // GPS �������� �����

#define    NUM_FOSN        71                          // FOSN ���ĸ��� 
#define    NUM_GPS         120
#define    RX_BUF_SIZE     1024                        // GPS �������ݳ���

/* �ֶ�׼ */
#define    COARSE_JX    1                            // ������
#define    COARSE_GX    2                            // ����ϵ��
#define    COARSE_NG    3                            // ���̷�
#define    COARSE_QM    4                            // ��Ԫ����

/* ����׼ģʽ */
#define    FINE_CMPS    1                            // �޾���������
#define    FINE_0su     2                            //��������
#define    FINE_Yucia   3                            // Ҧ����������׼
                           
#define    FINE_ADRC    5                            // ADRC

#define    YA_POS       1							//����Yucia�㷨��15/3 ���ϵͳλ�����
#define    YA_VEL       2							//����Yucia�㷨��15/3 ���ϵͳ�ٶ����
#define    YA_VELANDAZ  3							//����Yucia�㷨��15/3 ���ϵͳˮƽ�ٶ�+�������
//#define    YA_FOSN      1							//����Yucia�㷨��15/3 ���ϵͳFOSN����P Q R��ʼ��    //û�õ���������
//#define    YA_SIMU      2							//����Yucia�㷨��15/3 ���ϵͳ��������P Q R��ʼ��

// ����ģʽѡ��
#define    NAVI_SINS_UNDUMP   0                      // ������������
#define    NAVI_VELANDAZ      1                      // �ٶ�+�������nϵ
#define    NAVI_UNAV          2                      // λ��+�ٶ�+������ϣ�ˮ�µ�����ϣ�
#define    NAVI_SG            3                      // GPS λ�����
#define    NAVI_VEL           4                      // GPS�ٶ����
#define    NAVI_HAISHI_BASIC  5                      // �豸����
#define    NAVI_HAISHI_JZ     6                      // �豸����
#define    NAVI_PHINS_POS     7                      // PHINSλ�����
#define    NAVI_PHINS_VEL1    8                      // PHINS�ٶ����nϵ
#define    NAVI_PHINS_VEL2    9                      // PHINS�ٶ����bϵ
#define    NAVI_VELANDAZ2     10                     // �ٶ�+�������bϵ
#define    NAVI_DVL_Cmp_Depth    11                  // INS/DVL/Cmp/Depth
#define    NAVI_VELb_transverse  12				     // ����INS/DVL
//������ģʽѡ��
#define    PURE_SINS_UNDUMP   0                      // ������������
#define    PURE_SINS_DUMP     1                      // �������߶�����
#define    PURE_SINS_RV       2                      // ��������תʸ�������㷨
#define    PURE_SINS_TRANSVERSE 3                    // ���򵼺�
#define    PURE_SINS_HAISHI_P 4                      // ������haishi_P
#define    PURE_SINS_HAISHI_L 5                      // ������haishi_LD
#define    PURE_SINS_HAISHI_0RP 6                    // ������haishi_�޸˱۳���
//������ϵͳ��ر����ṹ��
struct SYS_ELEMENT
{

	double att_angle[3];//��̬��
	double vel_n[3];//�ٶ�
	double old_v[3];//��һ�μ�����ٶ�ֵ
	double vel_b[3];
	double dvel_n[3];//���ٶ�
   	double lati;//γ��
	double longi;//����
	double high;//�߶�
	double pos[3];
	double g;//����
	double dlati;//γ�ȱ仯��
	double dlongi;//���ȱ仯��
	
	double cbn_mat[3][3];//C b(�±꣩ n(�ϱ�)�������
	double cnb_mat[3][3];
	double quart[4];//��̬��Ԫ��
	double acce_n[3];//�ӱ���nϵֵ


	double gyro_wib_b[3];//������bϵֵ
	double gyro_old[3];//��һ�βɼ�������ֵ
	double acce_b[3];//�ӱ���bϵֵ
	double acce_old[3];
	
	double initial_pos[3];
	double user_position[3];

	float gyro_bias_esti[3];//������ƫ���ƣ�kalman�˲�ģʽ����Ч��
	float acce_bias_esti[3];//�ӱ���ƫ���ƣ�kalman�˲�ģʽ����Ч��

	/////�˱�ЧӦ�й�
	double wnb_b[3];
	double wnb_b_arm[3];
	double vel_arm[3];
	double rp[3];
	/////�豸�ٶȸ����й�
	double acce_n_sum[3];
	int acce_n_t;//���ٴ�һ����
	////pao
	int flagFire;
	double att_angle_old[3];
	int device_type;
	int flagComps;

	double rvib[3];//��תʸ��
	
};
struct SYS_ELEMENTtransverse
{
	//���������������ͳ��������ϵ����������
	double att_angle[3];
	double vel_n[3];
	double lati;
	double longi;
	double high;
	//sinstransverse_algo����������������
	double att_angle_S[3];
	double vel_S[3];
	double dvel_S[3];
	double lati_S;
	double longi_S;
	double high_S;
	double cbs_mat[3][3];
	double csb_mat[3][3];
	double quart_S[4];
	double acce_S[3];
	//������������IMU�����򷴽������������IMU����
	//	double gyro_wib_b[3]; ����Ҫ��ֱ����ȫ�ֱ���gyro����wib_b
	//	double gyro_old[3];
	double acce_b[3];
	double P;//0~2PAI, ��������ϵ˳ʱ��ת����������ϵ�ĽǶ�
	int isstart;    //�����ж��Ƿ��Ѿ���ȥtransverseģʽ

	double wie_s[3];  //20180319
	double wis_s[3];
	double t_1;
	double Rx_1;
	double Ry_1;
	double vel_b[3];
};
//�ֶ�׼��ر����ṹ��
struct COARSE_ALGI
{
	/*===========���ֶ�׼��������===========*/
	double g_sum[3]; // �������ٶ���bϵͶӰ���ۼӺ�
	double quart_coarse[4];  // ����ϵ����Cbi�����̷���Cb_ib0����ʱʹ�õ���Ԫ��


	/*===========�����ֶ�׼����============*/
	double gyro_sum[3];  // ��������ۼӺ�
	double cg_wie_mat[3][3];


	/*===========����ϵ�ֶ�׼����===========*/
	double Cin[3][3];  // iϵ��nϵ��ת�ƾ���
	double Cbi[3][3];  // bϵ��iϵ��ת�ƾ���
	double Cbi_old[3][3];  //�������ϵ��׼����ʱ�̵��Cbi����
	double Cib[3][3];	// Cbi��ת��

	double g_i_k1[3];	// tk1ʱ���������ٶ���iϵ�е�ͶӰ
	double g_i_k2[3];	// tk2ʱ���������ٶ���iϵ�е�ͶӰ
	double g_i_k3[3];	// tk3ʱ���������ٶ���iϵ�е�ͶӰ

	double avg_att[3];	// �������ʱ��εĳ�ʼ��̬��ֵ 


	/*==========���̽����ֶ�׼����===========*/
	double Cb_ib0[3][3];  // bϵ��ib0ϵת�ƾ���
	double Cib0_b[3][3];  //Cb_ib0ת��
	double Cb_ib0_old[3][3]; //�������̽����ֶ�׼����ʱ�̵�Cb_ib0����
	double Cib0_i0[3][3];  // ib0ϵ��i0ϵת�ƾ���
	double Ci0_e[3][3];  // i0ϵ��eϵת�ƾ���
	double Ce_n[3][3];	// eϵ��nϵת�ƾ���

	/*-------------������Ԫ������̬ȷ���㷨����------------*/
	double qbi[4];	// iϵ��bϵ��ת����Ԫ��
	double qni[4];	// iϵ��nϵ��ת����Ԫ��
	double qin[4];	// nϵ��iϵ��ת����Ԫ��
	double qbn_0[4];  // ��׼��ʼʱ��nϵ��bϵ��ת����Ԫ��
	double Kk[4][4];  // Wahba����K����
	double delta_Kk[4][4];
	double v_accb[3];
	double v_accb_old[3];
	double vel_n_dvl[3];

	double f_sum[3];  // �ӱ�����ۼ�ֵ
	double v_sum[3];  //�ӱ����ֵ��ib0ϵͶӰ�Ļ���ֵ
	double f_ib0_k1[3];  // t0��tk1ʱ��μӱ����ֵ��ib0ϵͶӰ�Ļ���ֵ
	double f_ib0_k2[3];  // t0��tk2ʱ��μӱ����ֵ��ib0ϵͶӰ�Ļ���ֵ	
	double g_i0_k1[3];	// t0��tk1ʱ����������ٶ���i0ϵͶӰ�Ļ���ֵ
	double g_i0_k2[3];	// t0��tk2ʱ����������ٶ���i0ϵͶӰ�Ļ���ֵ
};
//IMU���
struct OUTIMU
{
	double gyro_b[3];
	double acce_b[3];
};
//������������Ҫ������ʾ
class INSCAL
{
public:
	double ang[3];
	double vel[3];
	double pos[3];
	float err_ang[3];
	float err_pos[3];
	float err_vel[3];
	INSCAL()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		memset(pos, 0, sizeof(ang));
		memset(err_ang, 0, sizeof(err_ang));
		memset(err_pos, 0, sizeof(err_pos));
		memset(err_vel, 0, sizeof(err_vel));
	}
};
//ת̨��̬
class ZTPARA
{
public:
	double ang[3];
	int Frame;
	int cnt;
	ZTPARA()
	{
		Frame = 0;
		cnt = 0;
		memset(ang, 0, sizeof(ang));
	}
};
//������������δ��ϸ����
struct NAVPARA
{
	double  gn[3];
};
class EARTH
{
	double g[3];
};
//�޾�����׼������
struct  COMPALIGN
{
	double  wc_n[3];
	double  wc_b[3];
	double  fc_n[3];
	double  fc_b[3];
	double  k1[3];
	double	k2[5]; 
};
//GPS���
class GPS
{
public:
	double pos[3];
	double time;
	double vel[3];
	int cnt;
	int flag;
	double vv,hv,att;//Vertical speed,Horizontal speed,Actual direction with respect to True North
	
	GPS()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		
		time = 0.0;
		cnt = 0;
		flag = 0;
	}
	void reset()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		
		time = 0.0;
		cnt = 0;
		flag = 0;
	}
};
//FOSN���
class FOSN
{
public:
	double vel[3];
	double pos[3];
	double ang[3];
	double time;
	int recnum;
	int s, ms;
	FOSN()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		memset(ang, 0, sizeof(ang));
		time = 0;
		recnum = 0;
		ms = 0;
		s = 0;
	}
	void reset()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		memset(ang, 0, sizeof(ang));
		time = 0;
		recnum = 0;
		ms = 0;
		s = 0;
	}
};
class PHINS
{
public:
	double vel[3];
	double pos[3];
	double ang[3];
	double vel_b[3];
	int cnt;
	double utc;
	PHINS() 
	{
		memset(vel,0,sizeof(vel));
		memset(pos, 0, sizeof(pos));
		memset(ang, 0, sizeof(ang));
		memset(vel_b, 0, sizeof(vel_b));
		cnt = 0;
		utc = 0;
	}
	void reset()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		memset(ang, 0, sizeof(ang));
		memset(vel_b, 0, sizeof(vel_b));
		cnt = 0;
		utc = 0;
	}
};
//calibration parameters
class CALIPMT
{
public:
	double bias_gyro[3], bias_acce[3];
	double bias_gyro_random[3], bias_acce_random[3];
	double fix_err[3];//IMU��ת̨��װ����
	double fix_mat[3][3];
	double Eg_ang[6];//Egxy,Egyx,Egyz,Egzy,Egxz,Egzx
	double Ea_ang[6];//Eaxy,Eayx,Eayz,Eazy,Eaxz,Eazx
	double Cg[3], Ca[3];//calibration factors
	double Eg_mat[3][3] , Ea_mat[3][3];
	double Eg_mat_inv[3][3], Ea_mat_inv[3][3];
	CALIPMT();
	void Eang2mat();
	void fixup(double fixmat[3], double Cnb0[3][3]);
	void IMUcalibrate(SYS_ELEMENT &infor);
};

// ״̬��15ά�۲���3ά��Kalman�ṹ��, ����kalman����׼�������INS_GPS,INS_DVL��ϵ���
struct SKALMAN_15_3                   //20171108
{
	double	X_vector[15];         //״̬��XΪ15ά
	double  X_forecast[15];       //һ��Ԥ���Xֵ
	double	state_dis[15][15];    // ״̬ת�ƾ���
	double	P_matrix[15][15];     //������
	double  P_forecast[15][15];    //һ��Ԥ���Pֵ
	double	Q_state[15][15];      //ϵͳ����������
	double	R_measure[3][3];   //�۲�����������
	double	H_matrix[3][15];    //								
	double	Mea_vector[3];              //�۲���	
};
// ״̬��15ά�۲���3ά��Kalman�ṹ��, ����kalman����׼�������INS_GPS,INS_DVL��ϵ���
struct SKALMAN_16_3                   //20171108
{
	double	X_vector[16];         //״̬��XΪ15ά
	double  X_forecast[16];       //һ��Ԥ���Xֵ
	double	state_dis[16][16];    // ״̬ת�ƾ���
	double	P_matrix[16][16];     //������
	double  P_forecast[16][16];    //һ��Ԥ���Pֵ
	double	Q_state[16][16];      //ϵͳ����������
	double	R_measure[3][3];   //�۲�����������
	double	H_matrix[3][16];    //								
	double	Mea_vector[3];              //�۲���	
};
// ״̬��19ά�۲���3ά��Kalman�ṹ��, ����ˮ�ٹ��Ƶ�INS_DVL��ϵ���
struct SKALMAN_19_6                   //20171108
{
	double	X_vector[19];         //״̬��XΪ15ά
	double  X_forecast[19];       //һ��Ԥ���Xֵ
	double	state_dis[19][19];    // ״̬ת�ƾ���
	double	P_matrix[19][19];     //������
	double  P_forecast[19][19];    //һ��Ԥ���Pֵ
	double	Q_state[19][19];      //ϵͳ����������
	double	R_measure[6][6];   //�۲�����������
	double	H_matrix[6][19];    //								
	double	Mea_vector[6];              //�۲���
	double  f;              //����
};
// ״̬��15ά�۲���1ά��Kalman�ṹ��, ����INS_Cmp,INS_Depth��ϵ���
struct SKALMAN_15_1                   //20180116
{
	double	X_vector[15];         //״̬��XΪ15ά
	double  X_forecast[15];       //һ��Ԥ���Xֵ
	double	state_dis[15][15];    // ״̬ת�ƾ���
	double	P_matrix[15][15];     //������
	double  P_forecast[15][15];    //һ��Ԥ���Pֵ
	double	Q_state[15][15];      //ϵͳ����������
	double	R_measure[1][1];   //�۲�����������
	double	H_matrix[1][15];    //								
	double	Mea_vector[1];              //�۲���	
};
// ͨ����kalman�˲�����Ƴ���,��ʱ���21*7ά��21��Ŀǰ��֪����ô�ã�7ά����3�ٶ�3λ��1�����
// ͨ���ͱ�����д���о����������㣬_(:�١���)_������ʱ����
struct SKALMAN_RE                   
{
	double	X_vector[21];         //״̬��XΪ21ά
	double  X_forecast[21];       //һ��Ԥ���Xֵ
	double	state_dis[21][21];    // ״̬ת�ƾ���
	double	P_matrix[21][21];     //������
	double  P_forecast[21][21];    //һ��Ԥ���Pֵ
	double	Q_state[21][21];      //ϵͳ����������
	double	R_measure[7][7];   //�۲�����������
	double	H_matrix[7][21];    //								
	double	Mea_vector[7];      //�۲���	
};
//ADRC�����ṹ��
struct ADRC_S
{
	double z1e,z2e,z3e,z1n,z2n,z3n,z4n;//״̬��
	double y1e,y1n;//״̬��
	int b01,b02,b03;//�������
	int b11,b12,b13,b14;//�������
	float p1,p2,pf;//2���Ʋ���
	float p11,p12,p13;//3�׿��Ʋ���
	float h,delta,h2;//
	float b0;//����ϵ��
	double ue,un,uu,ufe,ufn;
	float azi1,azi2,azi3,azi4,azi5;//���޾������׼����
	double v1,v2;//TD���
};
// ����ˮ����ϵ������������е�DVL��Cmp��Depth����ⲿ��Ϣ��
// �Լ�IMM����������Ϣ�����Ĵ洢�ռ䡢Rkf���޵�
class SDVLCmpDepth                   //20171108
{
public:
	double	DVL_d[3];         //DVL�Եز���
	double  DVL_c[3];         //DVL�Եز���
	double  K;                //�̶�����
	double	V_c_n[3];    // ����ϵˮ��
	double  L;           //ˮ�ٿռ����ϵ��
	double	Cmp;     //���޾�
	double  Depth;    //�߶ȼ�
	int flag_DVL; //0-DVL��ˮ�Եؾ���Ч��1-DVL��ˮ��Ч��2-DVL�Ե���Ч��3-DVL��Ч
	int flag_C;   //0-���޾���Ч��1-��Ч
	int flag_D;   //0-��ȼ���Ч��1-��Ч
	int flag_current;   //ˮ���ٶȳɹ����Ʊ�־��1-δ���ƣ�0-ˮ���ѹ���
	int flag_mode_1;      //��һʱ�̹���ģʽ��־λ�������ж��Ƿ������ģʽ��������ʼ��,100Ϊ�ս������׼�ĳ�ʼģʽ
	int flag_mode;       // Ӧѡ��Ĺ���ģʽ,����ǰ����ģʽ����DVL״̬��ˮ������״̬λ�����ж�
	int flag_pureins;    //ģʽ3(������)��  0 - ������(������); 1 - ZUPT;  2 - AI(������)
	int flag_pureinsLong; // �жϴ�����ʱ�䳤�̵ı�־λ��1 - ����0 - �̣���800sΪ�硣
	double DVL_ERR; //DVL ������3������һ������һ��ָʾ��
	double R1_esti[6][6];  //IMM KF1��ӦR
	double R2_esti[6][6];  //IMM KF2��ӦR
	double R3_esti[6][6];  //IMM KF3��ӦR
	double R_adaptive[6][6];  //IMM ����ӦRk��������������Ϣ��Rk
	double u[3];           //ģ�͸���
	double P[3][3];        //ģ�͸���ת�ƾ���
	double R_A_store[20][6]; //����������Ϣ���ݴ洢,for ����Ӧ
	double wbfb_store[20][6]; //�����������ݼӱ����ݴ洢��for ZUPT
	int k_0;   //��¼�����ģʽ��ʱ��
	double deltaK; //��¼����ģʽ3ʱ��deltaK���������뿪ģʽ3ʱ�ô˸�Xk[15]��ֵ
	double ZUPT_angle;//bϵ������ϵ�ļн�
	double lamda;//������¼lamda
	double Vbx_store[100];//�����洢Vb,����ȡ����ת̨ϵ�복��ϵ�ĺ���Ǽн�
	double Vby_store[100];//�����洢Vb,����ȡ����ת̨ϵ�복��ϵ�ĺ���Ǽн�
	int kk_ZUPT;//������¼Vb_store�洢������

	int flag_AI; // ���������Ƿ����AI�����Ĵ���������Ϊ���ơ�1-����AI��0-������AI
	int flag_AI_train; //AIѵ����־λ������ѵ���̣߳�1 - ����AIѵ����0 - ������AIѵ������ʼ��1
	int flag_AI_store; //AI�洢��־λ��1-���Ŵ洢AIѵ�����ݣ�0-ѵ�����洢�������ٴ洢����ʼ��1
	int flag_AI_Ready; //0-��ûѵ���ã�1-ѵ����ɣ�����Ԥ��
	int kk_AI; //�ƴΣ�ѵ�����ݣ�
	int sum_k_AI;//ѵ��������
	double AI_vn_store[3][1900]; //�洢vns   1900��sum_k_AIҪ��Ӧ����������
	double AI_fn_store[3][1900]; // �洢fn   1900��sum_k_AIҪ��Ӧ����������
	double AI_deltaVn_store[3][1900]; //�洢delta_vn  1900��sum_k_AIҪ��Ӧ����������
	double AI_fn_k_1[3];  //��¼��ʷ������Ϣ
	double AI_vns_k_1[3]; //��¼��ʷ������Ϣ
	double AI_Input[12][1899];//��sum_k_AI��1
	double AI_sig2;
	double AI_gamma;
	double AI_b1[1][1];
	double AI_b2[1][1];
	double AI_b3[1][1];
	double AI_a1[1899][1];   //��sum_k_AI��1
	double AI_a2[1899][1];
	double AI_a3[1899][1];

	SDVLCmpDepth()
	{
		V_c_n[0] = 0.3; V_c_n[1] = 0.3; V_c_n[2] = 0.1;
		L = 10000;
		K = 1.00;
		flag_DVL = 0;
		flag_C = 0;
		flag_D = 0;
		flag_current = 1; //1-δ����
		flag_mode_1 = 100; //��㸳һ��ֵ
		flag_mode = 100;
		flag_pureins = 1;
		flag_pureinsLong = 0;
		DVL_ERR = 0.0;
		lamda = 0;
		kk_ZUPT = 0;
		u[0] = 0.05; u[1] = 0; u[2] = 0.95;
		P[0][0] = 0.8; P[0][1] = 0.1; P[0][2] = 0.1;
		P[1][0] = 0.1; P[1][1] = 0.8; P[1][2] = 0.1;
		P[2][0] = 0.1; P[2][1] = 0.1; P[2][2] = 0.8;
		memset(R_A_store, 0, sizeof(R_A_store));
		memset(wbfb_store, 0, sizeof(wbfb_store));//�ڳ���ģʽ3ʱ����
		memset(Vbx_store, 0, sizeof(Vbx_store));
		memset(Vby_store, 0, sizeof(Vby_store));
		deltaK = 0;
		ZUPT_angle = 0;// 0.8185*D2R;
		flag_AI = 0;   // ���������Ƿ����AI�����Ĵ���������Ϊ���ơ�1 - ����AI��0 - ������AI
		flag_AI_Ready = 0;
		flag_AI_store = 1;
		flag_AI_train = 0;
		kk_AI = 0;//�ƴΣ�ѵ�����ݣ�
		sum_k_AI = 1900;//ѵ��������
		memset(AI_vn_store, 0, sizeof(AI_vn_store));
		memset(AI_fn_store, 0, sizeof(AI_fn_store));
		memset(AI_deltaVn_store, 0, sizeof(AI_deltaVn_store));
		AI_sig2 = 10000;
		AI_gamma = 10000;
	}
};
//ϵͳ���Ʋ���
class SYSTEMCTRL
{
public:
	//�����������
	int cnt_s;                                //���㿪ʼϵͳʱ��
	long int data_cnt;                        //���㿪ʼ����IMU����ͳ��
	int Fs;                                   // ��������Ƶ��
	float Ts;                                 // ������������
	int Kal_fr;                               // Kalman����Ƶ��(��ʱû����С��1�����)
	bool f_coarse_over;                       //�ֶ�׼��ɱ�־
	bool f_fine_over;                         //����׼��ɱ�־
	bool f_navi_over;
	CString state;                            //ϵͳ����״̬
	int coarse_cnt;                           //�ֶ�׼�������ͳ��
	int kal_cnt;                              //Kalman�˲����д���

	
	//��׼ʱ��
	int coarse_time;                          //�ֶ�׼ʱ��
	int algn_time;                            //��׼��ʱ��
	int fine_level;
	int fine_azimuth;

	SYSTEMCTRL()
	{
		data_cnt = 0;
		cnt_s = 0;									// ��������ʱ�䣬 ��λ��
		state = _T("��û��ʼ");
		f_fine_over = 1;							// ����׼������־
		coarse_cnt = 0;								// �ֶ�׼����
		f_coarse_over = 1;							// �ֶ�׼������־
		f_navi_over = 1;
		kal_cnt = 0;
		Kal_fr = 1;
	}
	void reset()
	{
		data_cnt = 0;
		cnt_s = 0;									// ��������ʱ�䣬 ��λ��
		state = _T("��û��ʼ");
		f_fine_over = 1;							// ����׼������־
		coarse_cnt = 0;								// �ֶ�׼����
		f_coarse_over = 1;							// �ֶ�׼������־
		f_navi_over = 1;
		kal_cnt = 0;
		Kal_fr = 1;
	}
};
//������׼ģʽһЩ���Ʊ�����
class READSIMULATION
{
public:
	bool RS_mode;//�Ƿ��Ƕ�����׼ģʽ
	int delay5ms, ReadInitPos, ReadInitAtt;//3��checkbox
	bool isstart;//��ʼ��־
	bool canCal;//��������
	int file_mode;//�ļ���ʽ
	int skiptime;//������ʼ��һ������
	CString RdataFile;
	FILE *RdataFilefid;
	READSIMULATION() 
	{
		delay5ms = 0;
		skiptime = 0;
		ReadInitPos = 0;
		ReadInitAtt = 0;
		RS_mode = false;
		isstart = false;
		canCal = false;
		RdataFilefid = NULL;
	}
	void reset()
	{
		delay5ms = 0;
		skiptime = 0;
		ReadInitPos = 0;
		ReadInitAtt = 0;
		RS_mode = false;
		isstart = false;
		canCal = false;
		if (RdataFilefid != NULL) fclose(RdataFilefid);
		RdataFilefid = NULL;
	}


};

//�豸�����õ�kalman�˲��ṹ��
#define	        sta_num         12
#define	 		mea_num         3
struct FILTER
{
	double	X_vector[sta_num];
	double	P_matrix[sta_num][sta_num];
	double	Q_state[sta_num][sta_num];
	double	R_measure[mea_num][mea_num];
	double	H_matrix[mea_num][sta_num];
	double	F_sum[sta_num][sta_num];
	double	F_kal[sta_num][sta_num];
	double	Mea_vector[mea_num];
	double 	state_matrix[sta_num][sta_num];
	double 	gyro_comps[3];
};
