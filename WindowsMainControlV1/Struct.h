#include <cstring>
#pragma once 

#define  PAI             3.1415926535
#define  D2R             0.017453292519943           // 角度化为弧度
#define  R2D             57.295779513082323          // 弧度化角度
#define  WIE             7.2921151467E-5             // 15.04088*D2R/3600.0
#define  RE              6.378137e6L                 // 地球半径
#define  AEE             1.0/298.257223563           // 扁率
#define  E1              0.08181922                  //第一偏心率
#define  E2              0.00669438                  //第一偏心率
#define  EI              1e-6

#define  GRAVITY         9.7803267714                //g
#define  ug              GRAVITY*0.000001            //ug
#define	 LIGHTSPEED      2.99792458e8                //光速
#define  dph             D2R/3600                    //°/h->deg/s

#define  LOOSE           1                           // GPS 接收数据 松组合
#define  TIGHT           2                           // GPS 接收数据 紧组合

#define    NUM_FOSN        71                          // FOSN 报文个数 
#define    NUM_GPS         120
#define    RX_BUF_SIZE     1024                        // GPS 接收数据长度

/* 粗对准 */
#define    COARSE_JX    1                            // 解析法
#define    COARSE_GX    2                            // 惯性系法
#define    COARSE_NG    3                            // 凝固法
#define    COARSE_QM    4                            // 四元数法

/* 精对准模式 */
#define    FINE_CMPS    1                            // 罗经法静基座
#define    FINE_0su     2                            //零速修正
#define    FINE_Yucia   3                            // 姚逸卿完美对准
                           
#define    FINE_ADRC    5                            // ADRC

#define    YA_POS       1							//基于Yucia算法的15/3 组合系统位置组合
#define    YA_VEL       2							//基于Yucia算法的15/3 组合系统速度组合
#define    YA_VELANDAZ  3							//基于Yucia算法的15/3 组合系统水平速度+航向组合
//#define    YA_FOSN      1							//基于Yucia算法的15/3 组合系统FOSN数据P Q R初始化    //没用到，作甚？
//#define    YA_SIMU      2							//基于Yucia算法的15/3 组合系统仿真数据P Q R初始化

// 导航模式选择
#define    NAVI_SINS_UNDUMP   0                      // 纯捷联无阻尼
#define    NAVI_VELANDAZ      1                      // 速度+航向组合n系
#define    NAVI_UNAV          2                      // 位置+速度+航向组合（水下导航组合）
#define    NAVI_SG            3                      // GPS 位置组合
#define    NAVI_VEL           4                      // GPS速度组合
#define    NAVI_HAISHI_BASIC  5                      // 设备基础
#define    NAVI_HAISHI_JZ     6                      // 设备降噪
#define    NAVI_PHINS_POS     7                      // PHINS位置组合
#define    NAVI_PHINS_VEL1    8                      // PHINS速度组合n系
#define    NAVI_PHINS_VEL2    9                      // PHINS速度组合b系
#define    NAVI_VELANDAZ2     10                     // 速度+航向组合b系
//纯惯性模式选择
#define    PURE_SINS_UNDUMP   0                      // 纯捷联无阻尼
#define    PURE_SINS_DUMP     1                      // 纯捷联高度阻尼
#define    PURE_SINS_RV       2                      // 纯捷联旋转矢量更新算法
#define    PURE_SINS_TRANSVERSE 3                    // 横向导航
#define    PURE_SINS_HAISHI_P 4                      // 纯捷联haishi_P
#define    PURE_SINS_HAISHI_L 5                      // 纯捷联haishi_LD
#define    PURE_SINS_HAISHI_0RP 6                    // 纯捷联haishi_无杆臂长度
//主导航系统相关变量结构体
struct SYS_ELEMENT
{

	double att_angle[3];//姿态角
	double vel_n[3];//速度
	double old_v[3];//上一次计算的速度值
	double vel_b[3];
	double dvel_n[3];//加速度
   	double lati;//纬度
	double longi;//经度
	double high;//高度
	double pos[3];
	double g;//重力
	double dlati;//纬度变化率
	double dlongi;//经度变化率
	
	double cbn_mat[3][3];//C b(下标） n(上标)这个矩阵
	double cnb_mat[3][3];
	double quart[4];//姿态四元数
	double acce_n[3];//加表在n系值


	double gyro_wib_b[3];//陀螺在b系值
	double gyro_old[3];//上一次采集的陀螺值
	double acce_b[3];//加表在b系值
	double acce_old[3];
	/*==========初始位置===========*/
	double initial_pos[3];
	double user_position[3];

	float gyro_bias_esti[3];//陀螺零偏估计，kalman滤波模式下有效果
	float acce_bias_esti[3];//加表零偏估计，kalman滤波模式下有效果

	/////杆臂效应有关
	double wnb_b[3];
	double wnb_b_arm[3];
	double vel_arm[3];
	double rp[3];
	/////设备速度更新有关
	double acce_n_sum[3];
	int acce_n_t;//多少次一计算
	////pao
	int flagFire;
	double att_angle_old[3];
	int device_type;
	int flagComps;

	double rvib[3];//旋转矢量
	
};
struct SYS_ELEMENTtransverse
{
	//将解算结果输出到传统地理坐标系进行误差计算
	double att_angle[3];
	double vel_n[3];
	double lati;
	double longi;
	double high;
	//sinstransverse_algo解算参数，输出参数
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
	//捷联解算所需IMU参数或反捷联解算产生的IMU参数
	//	double gyro_wib_b[3]; 不需要，直接有全局变量gyro输入wib_b
	//	double gyro_old[3];
	double acce_b[3];
	double P;//0~2PAI, 地理坐标系顺时针转到横向坐标系的角度
	int isstart;    //用来判断是否已经进去transverse模式
};
//粗对准相关变量结构体
struct COARSE_ALGI
{
	/*===========各粗对准公共参数===========*/
	double g_sum[3]; // 重力加速度在b系投影的累加和
	double quart_coarse[4];  // 惯性系法中Cbi、凝固法中Cb_ib0更新时使用的四元数


	/*===========解析粗对准参数============*/
	double gyro_sum[3];  // 陀螺输出累加和
	double cg_wie_mat[3][3];


	/*===========惯性系粗对准参数===========*/
	double Cin[3][3];  // i系至n系的转移矩阵
	double Cbi[3][3];  // b系至i系的转移矩阵
	double Cbi_old[3][3];  //保存惯性系对准计算时刻点的Cbi矩阵
	double Cib[3][3];	// Cbi的转置

	double g_i_k1[3];	// tk1时刻重力加速度在i系中的投影
	double g_i_k2[3];	// tk2时刻重力加速度在i系中的投影
	double g_i_k3[3];	// tk3时刻重力加速度在i系中的投影

	double avg_att[3];	// 求各个分时间段的初始姿态均值 


	/*==========凝固解析粗对准参数===========*/
	double Cb_ib0[3][3];  // b系至ib0系转移矩阵
	double Cib0_b[3][3];  //Cb_ib0转置
	double Cb_ib0_old[3][3]; //保存凝固解析粗对准计算时刻的Cb_ib0矩阵
	double Cib0_i0[3][3];  // ib0系至i0系转移矩阵
	double Ci0_e[3][3];  // i0系至e系转移矩阵
	double Ce_n[3][3];	// e系至n系转移矩阵

	/*-------------基于四元数的姿态确定算法参数------------*/
	double qbi[4];	// i系至b系的转换四元数
	double qni[4];	// i系至n系的转换四元数
	double qin[4];	// n系至i系的转换四元数
	double qbn_0[4];  // 对准开始时刻n系至b系的转换四元数
	double Kk[4][4];  // Wahba问题K矩阵
	double delta_Kk[4][4];
	double v_accb[3];
	double v_accb_old[3];
	double vel_n_dvl[3];

	double f_sum[3];  // 加表输出累加值
	double v_sum[3];  //加表输出值在ib0系投影的积分值
	double f_ib0_k1[3];  // t0到tk1时间段加表输出值在ib0系投影的积分值
	double f_ib0_k2[3];  // t0到tk2时间段加表输出值在ib0系投影的积分值	
	double g_i0_k1[3];	// t0到tk1时间段重力加速度在i0系投影的积分值
	double g_i0_k2[3];	// t0到tk2时间段重力加速度在i0系投影的积分值
};
//IMU输出
struct OUTIMU
{
	double gyro_b[3];
	double acce_b[3];
};
//解算的输出，主要用来显示
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
//转台姿态
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
//导航参数，还未仔细考究
struct NAVPARA
{
	double  gn[3];
};
class EARTH
{
	double g[3];
};
//罗经法对准控制量
struct  COMPALIGN
{
	double  wc_n[3];
	double  wc_b[3];
	double  fc_n[3];
	double  fc_b[3];
	double  k1[3];
	double	k2[5]; 
};
//GPS相关
class GPS
{
public:
	double pos[3];
	double time;
	double vel[3];
	int cnt;
	int flag;
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
//FOSN相关
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
	double fix_err[3];//IMU与转台安装误差角
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

// 状态量15维观测量3维的Kalman结构体, 用于kalman精对准、最基本INS_GPS,INS_DVL组合导航
struct SKALMAN_15_3                   //20171108
{
	double	X_vector[15];         //状态量X为15维
	double  X_forecast[15];       //一步预测的X值
	double	state_dis[15][15];    // 状态转移矩阵；
	double	P_matrix[15][15];     //误差方差阵；
	double  P_forecast[15][15];    //一步预测的P值
	double	Q_state[15][15];      //系统噪声方差阵
	double	R_measure[3][3];   //观测噪声方差阵
	double	H_matrix[3][15];    //								
	double	Mea_vector[3];              //观测量	
};
// 状态量15维观测量3维的Kalman结构体, 用于kalman精对准、最基本INS_GPS,INS_DVL组合导航
struct SKALMAN_16_3                   //20171108
{
	double	X_vector[16];         //状态量X为15维
	double  X_forecast[16];       //一步预测的X值
	double	state_dis[16][16];    // 状态转移矩阵；
	double	P_matrix[16][16];     //误差方差阵；
	double  P_forecast[16][16];    //一步预测的P值
	double	Q_state[16][16];      //系统噪声方差阵
	double	R_measure[3][3];   //观测噪声方差阵
	double	H_matrix[3][16];    //								
	double	Mea_vector[3];              //观测量	
};
// 通用型kalman滤波器设计尝试,暂时设计21*7维。21的目前不知道怎么用，7维考虑3速度3位置1航向等
// 通用型必须重写所有矩阵、向量运算，_(:з」∠)_算了暂时放弃
struct SKALMAN_RE                   
{
	double	X_vector[21];         //状态量X为21维
	double  X_forecast[21];       //一步预测的X值
	double	state_dis[21][21];    // 状态转移矩阵；
	double	P_matrix[21][21];     //误差方差阵；
	double  P_forecast[21][21];    //一步预测的P值
	double	Q_state[21][21];      //系统噪声方差阵
	double	R_measure[7][7];   //观测噪声方差阵
	double	H_matrix[7][21];    //								
	double	Mea_vector[7];      //观测量	
};
//ADRC参数结构体
struct ADRC_S
{
	double z1e,z2e,z3e,z1n,z2n,z3n,z4n;//状态量
	double y1e,y1n;//状态量
	int b01,b02,b03;//东向参数
	int b11,b12,b13,b14;//北向参数
	float p1,p2,pf;//2控制参数
	float p11,p12,p13;//3阶控制参数
	float h,delta,h2;//
	float b0;//反馈系数
	double ue,un,uu,ufe,ufn;
	float azi1,azi2,azi3,azi4,azi5;//仿罗经航向对准参数
	double v1,v2;//TD输出
};
//系统控制参数
class SYSTEMCTRL
{
public:
	//捷联计算参数
	int cnt_s;                                //解算开始系统时间
	long int data_cnt;                        //解算开始接收IMU数据统计
	int Fs;                                   // 捷联解算频率
	float Ts;                                 // 捷联解算周期
	int Kal_fr;                               // Kalman解算频率(暂时没考虑小于1的情况)
	bool f_coarse_over;                       //粗对准完成标志
	bool f_fine_over;                         //精对准完成标志
	bool f_navi_over;
	CString state;                            //系统运行状态
	int coarse_cnt;                           //粗对准计算次数统计
	int kal_cnt;                              //Kalman滤波运行次数

	
	//对准时间
	int coarse_time;                          //粗对准时间
	int algn_time;                            //对准总时间
	int fine_level;
	int fine_azimuth;

	SYSTEMCTRL()
	{
		data_cnt = 0;
		cnt_s = 0;									// 导航解算时间， 单位秒
		state = _T("还没开始");
		f_fine_over = 1;							// 精对准结束标志
		coarse_cnt = 0;								// 粗对准计数
		f_coarse_over = 1;							// 粗对准结束标志
		f_navi_over = 1;
		kal_cnt = 0;
		Kal_fr = 1;
	}
	void reset()
	{
		data_cnt = 0;
		cnt_s = 0;									// 导航解算时间， 单位秒
		state = _T("还没开始");
		f_fine_over = 1;							// 精对准结束标志
		coarse_cnt = 0;								// 粗对准计数
		f_coarse_over = 1;							// 粗对准结束标志
		f_navi_over = 1;
		kal_cnt = 0;
		Kal_fr = 1;
	}
};
//读数对准模式一些控制变量类
class READSIMULATION
{
public:
	bool RS_mode;//是否是读数对准模式
	int delay5ms, ReadInitPos, ReadInitAtt;//3个checkbox
	bool isstart;//开始标志
	bool canCal;//解算允许
	int file_mode;//文件格式
	CString RdataFile;
	FILE *RdataFilefid;
	READSIMULATION() 
	{
		delay5ms = 0;
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
		ReadInitPos = 0;
		ReadInitAtt = 0;
		RS_mode = false;
		isstart = false;
		canCal = false;
		if (RdataFilefid != NULL) fclose(RdataFilefid);
		RdataFilefid = NULL;
	}


};

//设备程序用的kalman滤波结构体
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