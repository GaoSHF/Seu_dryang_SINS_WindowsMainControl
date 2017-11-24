#pragma once

union INT4CH
{
	unsigned char Ch[4];
    int Int;
}int4ch;

union INT12CH
{
	unsigned char Ch[12];
    long int Int[3];
}int12ch;

union FL6CH
{
	char Ch[24];
	float  FLo[6];
}fl6ch;
union FL3CH
{
	char Ch[12];
	float  FLo[3];
};


union DB8CH
{
	char Ch[8];
	double Db;
};
union INT2CH
{
	unsigned char ch[2];
	short int Int;
};
union GPSLOOSE
{
	char ch[24];
	double db[3];
}gps_pv;

union  VA2CH
{
    char Ch[2];
	short int Int;
}Vel2CH,Ang2CH;

union
{
unsigned char ch_tem[4];
int int_tem;
}data;







