#include "Struct.h"
#pragma once

union INT4CH
{
	unsigned char Ch[4];
    int Int;
};
union INT20CH
{
	unsigned char Ch[20];
	int Int[5];
};
union INT12CH
{
	unsigned char Ch[12];
    long int Int[3];
};

union FL6CH
{
	char Ch[24];
	float  FLo[6];
};
union FL9CH
{
	char Ch[36];
	float  FLo[9];
};
union DB9CH
{
	char ch[72];
	double db[9];
};
union DB3CH
{
	char ch[24];
	double db[3];
};
union  VA2CH
{
    char Ch[2];
	short int Int;
};


//IMU输出
struct OUTIMU
{
	double gyro_b[3];
	double acce_b[3];
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

//GPS相关
class GPS
{
public:
	double pos[3];
	double time;
	double vel[3];
	int cnt;
	int flag;
	double vv, hv, att;//Vertical speed,Horizontal speed,Actual direction with respect to True North
	unsigned char SVs;
	int trackednum;
	int Conv(BYTE gps_buf[180]);
	GPS()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		SVs = 0;
		time = 0.0;
		cnt = 0;
		flag = 0;
		trackednum = 0;
	}
	void reset()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		SVs = 0;
		time = 0.0;
		cnt = 0;
		flag = 0;
		trackednum = 0;
	}
};
//FOSN相关
class FOSN
{
public:
	double vel[3];
	double pos[3];
	double ang[3];
	double fb[3];
	double wb[3];
	double time;
	int recnum;
	int s, ms;

	//新FOSN相关变量
	float input_p[3];//初始位置set
	float input_v[3];//初始速度set
	char  WorkingMode, IntegratedMode;
	int GPSmode;
	CString WorkingModeS, IntegratedModeS;
	float IPos[3], IVel[3], IAtt[3];//组合的位置、速度、姿态
	float GPos[3], GVel[3];//GPS的位置、速度

	FOSN()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		memset(ang, 0, sizeof(ang));
		memset(ang, 0, sizeof(input_p));
		memset(ang, 0, sizeof(input_v));
		memset(ang, 0, sizeof(IPos));
		memset(ang, 0, sizeof(IVel));
		memset(ang, 0, sizeof(IAtt));
		memset(ang, 0, sizeof(GPos));
		memset(ang, 0, sizeof(GVel));
		time = 0;
		recnum = 0;
		ms = 0;
		s = 0;
		GPSmode = 0;
		WorkingMode = 0x05;
		WorkingModeS = "没用";
		IntegratedMode = 0;
		IntegratedModeS = "没用";
	}
	void reset()
	{
		memset(vel, 0, sizeof(vel));
		memset(pos, 0, sizeof(pos));
		memset(ang, 0, sizeof(ang));
		memset(ang, 0, sizeof(input_p));
		memset(ang, 0, sizeof(input_v));
		memset(ang, 0, sizeof(IPos));
		memset(ang, 0, sizeof(IVel));
		memset(ang, 0, sizeof(IAtt));
		memset(ang, 0, sizeof(GPos));
		memset(ang, 0, sizeof(GVel));
		time = 0;
		recnum = 0;
		ms = 0;
		s = 0;
		GPSmode = 0;
		WorkingMode = 0x05;
		WorkingModeS = "没用";
		IntegratedMode = 0;
		IntegratedModeS = "没用";
	}
	void Conv(BYTE fosn_buf[70]);
	void Conv2(BYTE fosn_buf[180],int mode);
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
	void Conv(char phins_buf[42]);
	double getutc(char phins_buf[42]);
	PHINS()
	{
		memset(vel, 0, sizeof(vel));
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






