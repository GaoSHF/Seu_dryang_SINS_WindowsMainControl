#include "stdafx.h"
#include "Init_Navigation.h"
#include "DataConvert.h"


void FOSN::Conv(BYTE fosn_buf[70])
{
	INT4CH int4ch;
	INT12CH int12ch;
	FL6CH fl6ch;
	int i;
	//数据解析
	int4ch.Ch[0] = fosn_buf[3];
	int4ch.Ch[1] = fosn_buf[4];
	int4ch.Ch[2] = fosn_buf[5];
	int4ch.Ch[3] = fosn_buf[6];
	time = int4ch.Int / 1000.0;
	//msecond=(unsigned int)IN4CH.Int%1000/1000.0;
	s = (int)time;
	ms = ((int)time - s) * 1000;

	/* 载体坐标系为前上右 */
	for (i = 0; i < 12; i++)
		int12ch.Ch[i] = fosn_buf[i + 7];

	/***************XYZ轴角速率****************/
	wb[1] = int12ch.Int[0] / 3.6*1e-6;
	wb[2] = int12ch.Int[1] / 3.6*1e-6;
	wb[0] = int12ch.Int[2] / 3.6*1e-6;

	for (i = 0; i < 12; i++)
		int12ch.Ch[i] = fosn_buf[i + 19];

	/***************XYZ轴加速度****************/
	fb[1] = int12ch.Int[0] * 1e-6;
	fb[2] = int12ch.Int[1] * 1e-6;
	fb[0] = int12ch.Int[2] * 1e-6;

	for (i = 0; i < 24; i++)
		fl6ch.Ch[i] = fosn_buf[i + 33];

	/********横滚角、航向角、俯仰角************/
	ang[1] = fl6ch.FLo[0];
	ang[2] = fl6ch.FLo[1];
	ang[0] = fl6ch.FLo[2];

	/*************北、天、东向速度************/
	vel[1] = fl6ch.FLo[3];
	vel[2] = fl6ch.FLo[4];
	vel[0] = fl6ch.FLo[5];

	for (i = 0; i < 12; i++)
		fl6ch.Ch[i] = fosn_buf[i + 57];
	/**************纬度、经度、高度***********/
	pos[0] = fl6ch.FLo[0] * 57.29578;
	pos[1] = fl6ch.FLo[1] * 57.29578;
	pos[2] = fl6ch.FLo[2];
}
void FOSN::Conv2(BYTE fosn_buf[180],int mode)
{
	if (mode != 0)
	{
		//输出串口1/2
		time = (*(int *)(fosn_buf + 3)) / 1000.0;
		s = (int)time;
		ms = (int)((time - s) * 1000);
		wb[1] = (*(int *)(fosn_buf + 7)) *0.0005 / 3600;
		wb[2] = (*(int *)(fosn_buf + 11)) *0.0005 / 3600;
		wb[0] = (*(int *)(fosn_buf + 15)) *0.0005 / 3600;
		fb[1] = (*(int *)(fosn_buf + 19)) / 100000.0;
		fb[2] = (*(int *)(fosn_buf + 23)) / 100000.0;
		fb[0] = (*(int *)(fosn_buf + 27)) / 100000.0;
	}
	else
	{
		//监控口0
		time = (*(int *)(fosn_buf + 72)) / 1000.0;
		s = (int)time;
		ms = (int)((time - s) * 1000);
		wb[1] = (*(int *)(fosn_buf + 51)) *0.0005 / 3600.0;
		wb[2] = (*(int *)(fosn_buf + 55)) *0.0005 / 3600.0;
		wb[0] = (*(int *)(fosn_buf + 59)) *0.0005 / 3600.0;
		fb[1] = (*(int *)(fosn_buf + 21)) / 100000.0;
		fb[2] = (*(int *)(fosn_buf + 25)) / 100000.0;
		fb[0] = (*(int *)(fosn_buf + 29)) / 100000.0;
		ang[1] = (*(int *)(fosn_buf + 127)) / 10000.0;
		ang[2] = (*(int *)(fosn_buf + 131)) / 10000.0;
		ang[0] = (*(int *)(fosn_buf + 135)) / 10000.0;
		vel[1] = (*(short *)(fosn_buf + 139)) / 100.0;
		vel[2] = (*(short *)(fosn_buf + 141)) / 100.0;
		vel[0] = (*(short *)(fosn_buf + 143)) / 100.0;
		pos[0] = (*(int *)(fosn_buf + 145)) / 1000000.0;
		pos[2] = (*(int *)(fosn_buf + 149)) / 10.0;
		pos[1] = (*(int *)(fosn_buf + 153)) / 1000000.0;
		/*	IAtt[1] = (*(int *)(fosn_buf + 76)) / 10000.0;
		IAtt[2] = (*(int *)(fosn_buf + 80)) / 10000.0;
		IAtt[0] = (*(int *)(fosn_buf + 84)) / 10000.0;
		IVel[1] = (*(short *)(fosn_buf + 88)) / 100.0;
		IVel[2] = (*(short *)(fosn_buf + 90)) / 100.0;
		IVel[0] = (*(short *)(fosn_buf + 92)) / 100.0;
		IPos[0] = (*(int *)(fosn_buf + 94)) / 1000000.0;
		IPos[2] = (*(int *)(fosn_buf + 98)) / 10.0;
		IPos[1] = (*(int *)(fosn_buf + 102)) / 1000000.0;
		GPos[1] = (*(int *)(fosn_buf + 106)) / 100000.0;
		GPos[2] = (*(int *)(fosn_buf + 110)) / 100000.0;
		GPos[0] = (*(int *)(fosn_buf + 114)) / 100000.0;
		GVel[1] = (*(short *)(fosn_buf + 118)) / 100.0;
		GVel[2] = (*(short *)(fosn_buf + 120)) / 100.0;
		GVel[0] = (*(short *)(fosn_buf + 122)) / 100.0;*/
		GPSmode = (int)(*(fosn_buf + 124));
		WorkingMode = *(fosn_buf + 125);
		IntegratedMode = *(fosn_buf + 126);
		switch (WorkingMode)
		{
		case 0x00: WorkingModeS = "自检"; break;
		case 0x01: WorkingModeS = "待机"; break;
		case 0x02: WorkingModeS = "粗对准"; break;
		case 0x03: WorkingModeS = "精对准"; break;
		case 0x04: WorkingModeS = "导航"; break;
		case 0x05: WorkingModeS = "没用"; break;
		default:break;
		}

		switch (IntegratedMode)
		{
		case 0x00: IntegratedModeS = "没用"; break;
		case 0x01: IntegratedModeS = "自动零速修正"; break;
		case 0x02: IntegratedModeS = "惯性/卫星"; break;
		case 0x03: IntegratedModeS = "惯性/里程计"; break;
		case 0x04: IntegratedModeS = "惯性/电磁计程仪"; break;
		case 0x05: IntegratedModeS = "惯性/DVL"; break;
		default:break;
		}
	}
}
int GPS::Conv(BYTE gps_buf[180])
{
	int i;
	INT4CH int4ch;
	DB3CH gps_pv;
	if (gps_buf[0] == 0xAA && gps_buf[1] == 0x44 && gps_buf[2] == 0x12)
		switch (gps_buf[4])//pos 
		{
		case 0x2A:
#pragma region POS_VEL
			for (i = 0; i < 4; i++)
				int4ch.Ch[i] = gps_buf[i + 16];
			gps.time = int4ch.Int*0.001;

			for (i = 0; i < 24; i++)
				gps_pv.ch[i] = gps_buf[i + 36];
			gps.pos[0] = gps_pv.db[0];//*D2R;
			gps.pos[1] = gps_pv.db[1];//*D2R;
			gps.pos[2] = gps_pv.db[2];
			gps.SVs = gps_buf[i + 28 + 64];
			for (i = 0; i < 24; i++)
				gps_pv.ch[i] = gps_buf[i + 104 + 28 + 16];
			gps.hv = gps_pv.db[0];
			gps.att = gps_pv.db[1];
			gps.vv = gps_pv.db[2];
			

			gps.vel[0] = gps.hv*sin(gps.att*D2R);
			gps.vel[1] = gps.hv*cos(gps.att*D2R);
			gps.vel[2] = gps.vv;
			if (gps_buf[28] == 0)//solution computed
				return 1;
			else
				return 0;
			break;
#pragma endregion POS_VEL
		case 0x63:
#pragma region VEL_POS
			for (i = 0; i < 4; i++)
				int4ch.Ch[i] = gps_buf[i + 16];
			gps.time = int4ch.Int*0.001;

			for (i = 0; i < 24; i++)
				gps_pv.ch[i] = gps_buf[i + 28 + 16];
			gps.hv = gps_pv.db[0];
			gps.att = gps_pv.db[1];
			gps.vv = gps_pv.db[2];

			for (i = 0; i < 24; i++)
				gps_pv.ch[i] = gps_buf[i + 76 + 28 + 8];
			gps.pos[0] = gps_pv.db[0];
			gps.pos[1] = gps_pv.db[1];
			gps.pos[2] = gps_pv.db[2];
		//	if (is_startCal) gps.cnt++;
			gps.SVs = gps_buf[i + 76 + 28 + 64];
			gps.vel[0] = gps.hv*sin(gps.att*D2R);
			gps.vel[1] = gps.hv*cos(gps.att*D2R);
			gps.vel[2] = gps.vv;
			if (gps_buf[28] == 0)//solution computed
				return 1;
			else
				return 0;
			break;
#pragma endregion VEL_POS
		default:break;
		}
	else 
		return 0;
}
void PHINS::Conv(char phins_buf[42])
{
	INT4CH data;
	VA2CH Vel2CH, Ang2CH;
	double binary_value_31_bite = 180.0 / powl(2, 31);
	
	phins.cnt++;
	data.Ch[3] = phins_buf[1];
	data.Ch[2] = phins_buf[2];
	data.Ch[1] = phins_buf[3];
	data.Ch[0] = phins_buf[4];
	phins.utc = data.Int + ((UINT)phins_buf[5])*0.01;		//UTC time

	data.Ch[3] = phins_buf[6];
	data.Ch[2] = phins_buf[7];
	data.Ch[1] = phins_buf[8];
	data.Ch[0] = phins_buf[9];
	phins.pos[0] = data.Int * binary_value_31_bite;		//纬度

	data.Ch[3] = phins_buf[10];
	data.Ch[2] = phins_buf[11];
	data.Ch[1] = phins_buf[12];
	data.Ch[0] = phins_buf[13];
	phins.pos[1] = data.Int * binary_value_31_bite;      //经度


	data.Ch[3] = phins_buf[14];
	data.Ch[2] = phins_buf[15];
	data.Ch[1] = phins_buf[16];
	data.Ch[0] = phins_buf[17];
	phins.pos[2] = data.Int * 0.01;                       //高度


	Vel2CH.Ch[0] = phins_buf[23];              //东向速度
	Vel2CH.Ch[1] = phins_buf[22];
	phins.vel[0] = Vel2CH.Int*0.01;

	Vel2CH.Ch[0] = phins_buf[21];             //北向速度
	Vel2CH.Ch[1] = phins_buf[20];
	phins.vel[1] = Vel2CH.Int*0.01;

	Vel2CH.Ch[0] = phins_buf[25];              //天向速度
	Vel2CH.Ch[1] = phins_buf[24];
	phins.vel[2] = -Vel2CH.Int*0.01;

	Ang2CH.Ch[1] = phins_buf[28];
	Ang2CH.Ch[0] = phins_buf[29];
	phins.ang[0] = Ang2CH.Int * 180.0 / 32768.0;	//纵摇

	Ang2CH.Ch[1] = phins_buf[26];
	Ang2CH.Ch[0] = phins_buf[27];
	phins.ang[1] = Ang2CH.Int * 180.0 / 32768.0;	//横摇

	Ang2CH.Ch[1] = phins_buf[30];
	Ang2CH.Ch[0] = phins_buf[31];
	phins.ang[2] = -Ang2CH.Int * 180.0 / 32768.0;	//航向
}
double PHINS::getutc(char phins_buf[42])
{
	INT4CH data;	
	double utc;
	double binary_value_31_bite = 180.0 / powl(2, 31);	
	data.Ch[3] = phins_buf[1];
	data.Ch[2] = phins_buf[2];
	data.Ch[1] = phins_buf[3];
	data.Ch[0] = phins_buf[4];
	utc=data.Int + ((UINT)phins_buf[5])*0.01;		//UTC time
	return utc;
}