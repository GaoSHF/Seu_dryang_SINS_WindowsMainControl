#include <string.h>
#include <iostream>
#include "BasicNaviCal.h"

#pragma once 

#define GYROMIN  0.00000008161
#define ACCEMIN  0.00387
class Route
{
public:
	char state;//0b abcd efgh;  ab=00,stationary base;ab=01,dynamic base;
	//double vel_t;//velocity in trajectory  frame
	//swing motion   0b0100 0001
	double fsw[3], Asw[3], phi0[3];//frequency,amplitude,initial phase
	double Csw[3];//only used on stationary base 0b0000 0001
	double tsw[3];//time for swing	

	//circular motion(horizontal)   0b0100 0010
	//azimuth change/accelerated rectilinear motion/rectilinear motion 0b0100 0100
	//roll change  0b0100 1000
	//up direction velocity change  0b0101 0000

	double wt[3];// angular rate in trajectory  frame
	double at[3];// acceleration in trajectory  frame. 
	double vt_target;
	double timev[8];//tsw3,tw1,tw2.tw3,ta2
	double time_delay[8];//delay to execute
	double time_sort[8];//sort of the above 7 time variables
	bool turnwithroll;

	int var1;//user-defined;
	
public:
	//function	
	//degree to radius,default input is degree
	Route();//constructor
	//void create_cle(double Theta, double ommiga, bool withroll);//circular motion. angular rate
	void create_sw1(double f[3], double A[3], double p[3], double t[3]);//swing motion dynamic base
	void create_sw0(double f[3], double A[3], double p[3], double C[3], double t[3]);//swing motion stationary base
	void create_a(double t, double a = 0);//accelerate
	void create_v(double t, double v );//accelerate
	void create_r(double Theta, double ommiga);//roll change
	void create_p(double Theta, double ommiga);//pitch change
	float get_maxtime();//get the biggest time variable of Route
	void create_turn(double Theta, double ommiga, bool withroll=0);//turn left or right with azimuth change;
	void reset_route(double t);//t>timev[i], reset the corresponding variables
	void reset();
};

class IMUOUT
{
	
public:
	double att_angle[3];
	double att_dangle[3];
	double vel_n[3];
	double vel_b;// speed
	double vel_t;// 
	double dvel_n[3];
	double lati;
	double longi;
	double high;
	double g;
	double dlati;
	double dlongi;
	double f_b[3], wib_b[3];
	double cbn_mat[3][3];
	double cnb_mat[3][3];

	double gyro_wib_b[3];
	double acce_b[3];
	CALIPMT err_parameter;

	IMUOUT();
	double generate_data(Route route, double t, double delta_t, FILE *fide);//t, current time;quart_del;delta_t, generate period
	void printf_data(double t, FILE *fide, Route route);
	//void setbias(double w_bias_c[3], double w_bias_r[3], double a_bias_c[3], double a_bias_r[3]);
};



