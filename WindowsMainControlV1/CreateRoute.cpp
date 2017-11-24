#include "stdafx.h"
#include "CreateRoute.h"

Route::Route()
{
	memset(fsw, 0, sizeof(fsw));
	memset(Asw, 0, sizeof(Asw));
	memset(phi0, 0, sizeof(phi0));
	memset(Csw, 0, sizeof(Csw));
	memset(tsw, 0, sizeof(tsw));
	memset(wt, 0, sizeof(wt));
	memset(at, 0, sizeof(at));
	memset(timev, 0, sizeof(timev));
	memset(time_sort, 0, sizeof(time_sort));
	memset(time_delay, 0, sizeof(time_delay));
	state = 0;
	turnwithroll = 0;
	var1 = 0;
}
void Route::create_turn(double Theta, double ommiga, bool withroll)
{
	timev[5] = abs(Theta / ommiga);
	wt[2] = ommiga * D2R;
	state |= 0x42;
	turnwithroll = withroll;
}
void Route::create_sw1(double f[3], double A[3], double p[3], double t[3])
{
	int i;
	for (i = 0; i < 3; i++)
	{
		fsw[i] = f[i];
		Asw[i] = A[i] * D2R;
		phi0[i] = p[i] * D2R;
		tsw[i] = t[i];
	}
	state |= 0x41;
}
void Route::create_sw0(double f[3], double A[3], double p[3], double C[3], double t[3])
{
	int i;
	for (i = 0; i < 3; i++)
	{
		fsw[i] = f[i];
		Asw[i] = A[i] * D2R;
		phi0[i] = p[i] * D2R;
		tsw[i] = t[i];
		Csw[i] = C[i] * D2R;
	}
	state |= 0x01;
}
void Route::create_a(double t, double a)
{
	at[1] = a;
	timev[6] = t;
	state |= 0x44;
}
void Route::create_v(double t, double v)
{
	vt_target = v;
	timev[7] = t;
	//state |= 0x44;
	//at[1] = 0;

}
void Route::create_r(double Theta, double ommiga)
{
	timev[4] = abs(Theta / ommiga);
	wt[1] = ommiga*D2R;
	state |= 0x48;

}
void Route::create_p(double Theta, double ommiga)
{
	timev[3] = abs(Theta / ommiga);
	wt[0] = ommiga*D2R;
	state |= 0x50;

}
float Route::get_maxtime()
{
	int num = sizeof(timev) / sizeof(timev[0]);
	memcpy(timev, tsw, sizeof(tsw));
	memcpy(time_sort, timev, sizeof(timev));
	bubblesort(time_sort, num);
	return time_sort[num - 1];
}
void Route::reset()
{
	memset(fsw, 0, sizeof(fsw));
	memset(Asw, 0, sizeof(Asw));
	memset(phi0, 0, sizeof(phi0));
	memset(Csw, 0, sizeof(Csw));
	memset(tsw, 0, sizeof(tsw));
	memset(wt, 0, sizeof(wt));
	memset(at, 0, sizeof(at));
	memset(timev, 0, sizeof(timev));
	memset(time_sort, 0, sizeof(time_sort));
	memset(time_delay, 0, sizeof(time_delay));
	state = 0;
	turnwithroll = 0;
	var1 = 0;
}
void Route::reset_route(double t)
{
	int i;
	int num = sizeof(timev) / sizeof(timev[0]);//the  array length of timev; which is 7 now;
	var1 = 0;
	for (i = 0; i < num; i++)
	{
		if (t >= timev[i])
		{
			switch (i)
			{
			case 0: case 1: case 2:fsw[i] = 0; Asw[i] = 0; phi0[i] = 0; Csw[i] = 0; tsw[i] = 0; break;
			case 3: wt[0] = 0;  break;
			case 4: wt[1] = 0;  break;
			case 5: wt[2] = 0;  break;
			case 6: at[1] = 0;  break;
			case 7: vt_target = 0;  break;
			default:break;
			}
			timev[i] = 0;
			time_delay[i] = 0; 
		}
	}
}
IMUOUT::IMUOUT()
{
	memset(att_angle, 0, sizeof(att_angle));
	memset(att_dangle, 0, sizeof(att_dangle));
	memset(vel_n, 0, sizeof(vel_n));
	memset(dvel_n, 0, sizeof(dvel_n));
	vel_b = 0;
	vel_t = 0;
	lati = 32.057305 * D2R;
	longi = 118.786362 * D2R;
	high = 0;
	double gi = sqrt(1 - 0.00669437999013*sin(lati)*sin(lati));
	g = 9.7803267714*(1 + 0.00193185138639*sin(lati)*sin(lati)) / gi;

	dlati = 0;
	dlongi = 0;

	memset(cbn_mat, 0, sizeof(cbn_mat));
	memset(cnb_mat, 0, sizeof(cnb_mat));
	memset(gyro_wib_b, 0, sizeof(gyro_wib_b));
	memset(acce_b, 0, sizeof(acce_b));
	int i;
	for (i = 0; i < 3; i++)
	{
		err_parameter.bias_acce[i] = 50 * 9.78*0.000001;
		err_parameter.bias_acce_random[i] = 20 * 9.78*0.000001;
		err_parameter.bias_gyro[i] = 0.02* D2R / 3600;
		err_parameter.bias_gyro_random[i] = 0.006* D2R / 3600;
	}
}

double IMUOUT::generate_data(Route route, double t, double delta_t, FILE *fide)
{
	double tempt = t;
	//double temp_azimuth,temp_roll;
	//double temp_ve, temp_vn,temp_vu;
	double tmax;
	double Re, Rn;
	double an[3], wn[3], old_an[3], old_wn[3];//acceleration and angular rate in navigation  frame
	double win_n[3], win_b[3], wnb_b[3], temp_wa[3];
	double old_veln[3] = { 0 };
	double fw[3], f_n[3];
	double err_gyro[3], err_acce[3];
	double last_t = 0;
	int i;
	double Ctn[3][3], Cnt[3][3];
	double angnt[3];
	long int data_num = 0;
	double delta_v = 0;
	
	err_parameter.Eang2mat();
	tmax = route.get_maxtime();
	vel_t = vectormo(vel_n, 3);
	if (vel_t != 0)
		route.state == route.state | 0x40;//dynamic base
	if (route.timev[7] != 0)
	{
		delta_v = route.vt_target - vel_t;
		route.at[1] = delta_v / 2 / route.timev[7] * PAI*sin(PAI / route.timev[7] * last_t);
	}
	route.at[0] = -vel_t*route.wt[2];
	route.at[2] = vel_t*route.wt[0];

	//first time;
	angnt[0] = att_angle[0];
	angnt[1] = 0;
	angnt[2] = att_angle[2];
	ang2cnb(Cnt, angnt);
	maturn(3, 3, (double *)Ctn, (double *)Cnt);
	vecmul(3, 3, an, (double *)Ctn, route.at);
	vecmul(3, 3, wn, (double *)Ctn, route.wt);
	while (1)
	{

		Rn = RE*(1.0 - 2.0*AEE + 3.0*AEE*sin(lati)*sin(lati));
		Re = RE*(1.0 + AEE*sin(lati)*sin(lati));
		ang2cnb(cnb_mat, att_angle);

		last_t = t - tempt;
		route.reset_route(last_t);
		tmax = route.get_maxtime();
		if (last_t > tmax)
			break;

		//////Ctn//////
		angnt[0] = att_angle[0];
		angnt[1] = 0;
		angnt[2] = att_angle[2];
		ang2cnb(Cnt, angnt);
		maturn(3, 3, (double *)Ctn, (double *)Cnt);
		avecmul(3, old_an, an, 1);
		avecmul(3, old_wn, wn, 1);
		if (route.timev[7] != 0)
		{
			route.at[1] = delta_v / 2 / route.timev[7] * PAI*sin(PAI / route.timev[7] * last_t);
		}
		vel_t = vectormo(vel_n, 3);
		vecmul(3, 3, an, (double *)Ctn, route.at);
		vecmul(3, 3, wn, (double *)Ctn, route.wt);

		if (route.state & 0x01 == 0x01)////swing
		{
			for (i = 0; i < 3; i++)
			{
				att_dangle[i] = 2 * PAI * route.Asw[i] * route.fsw[i] * cos(2 * PAI * last_t * route.fsw[i] + route.phi0[i]);
				if (route.state == 0x01)//stationary			
					att_angle[i] = route.Asw[i] * sin(2 * PAI * last_t * route.fsw[i] + route.phi0[i]) + route.Csw[i];
				else
					att_angle[i] += att_dangle[i] * delta_t;
			}
		}
		else
		{
			for (i = 0; i < 3; i++)
			{
				att_dangle[i] = wn[i];
				att_angle[i] += (wn[i] + old_wn[i]) / 2 * delta_t;
				old_veln[i] = vel_n[i];
				vel_n[i] += (an[i] + old_an[i]) / 2 * delta_t;

				if (att_angle[i] > PAI)
					att_angle[i] = att_angle[i] - 2 * PAI;
				else if (att_angle[i] < -PAI)
					att_angle[i] = att_angle[i] + 2 * PAI;
			}
		}


		dlati = (old_veln[1] + vel_n[1]) / 2 / RE;
		dlongi = (old_veln[0] + vel_n[0]) / 2 / (RE * cos(lati));

		win_n[0] = -dlati;
		win_n[1] = WIE * cos(lati) + dlongi * cos(lati);
		win_n[2] = WIE * sin(lati) + dlongi * sin(lati);

		wnb_b[0] = att_dangle[0] * cos(att_angle[1]) - att_dangle[2] * sin(att_angle[1]) * cos(att_angle[0]);
		wnb_b[1] = att_dangle[1] + att_dangle[2] * sin(att_angle[0]);
		wnb_b[2] = att_dangle[0] * sin(att_angle[1]) + att_dangle[2] * cos(att_angle[1]) * cos(att_angle[0]);

		vecmul(3, 3, win_b, (double*)cnb_mat, (double*)win_n);
		vecadd(3, wib_b, win_b, wnb_b);
		vecmul(3, 3, temp_wa, (double *)err_parameter.Eg_mat, wib_b);
		for (i = 0; i < 3; i++)
		{
			err_gyro[i] = err_parameter.bias_gyro[i] + white() * err_parameter.bias_gyro_random[i];
			gyro_wib_b[i] = err_parameter.Cg[i] * (temp_wa[i] + err_gyro[i]);
			//gyro_normal = (int)(gyro_wib_b[i] / GYROMIN);
			//gyro_wib_b[i] = GYROMIN*gyro_normal;
		}

		/****************** simulate accelerator out *****************/


		fw[0] = -dlati;
		fw[1] = 2 * WIE * cos(lati) + dlongi * cos(lati);
		fw[2] = 2 * WIE * sin(lati) + dlongi * sin(lati);

		f_n[0] = an[0] - fw[2] * vel_n[1] + fw[1] * vel_n[2];
		f_n[1] = an[1] + fw[2] * vel_n[0] - fw[0] * vel_n[2];
		f_n[2] = an[2] - fw[1] * vel_n[0] + fw[0] * vel_n[1] + GRAVITY;

		vecmul(3, 3, f_b, (double*)cnb_mat, f_n);
		vecmul(3, 3, temp_wa, (double *)err_parameter.Ea_mat, f_b);
		for (i = 0; i < 3; i++)
		{
			err_acce[i] = err_parameter.bias_acce[i] + white() * err_parameter.bias_acce_random[i];
			acce_b[i] = err_parameter.Ca[i] * (temp_wa[i] + err_acce[i]);
			//acce_normal = (int)(acce_b[i] / ACCEMIN);
			//acce_b[i] = ACCEMIN*acce_normal;
		}

		/******** latitude & longitude **********/
		lati += dlati * delta_t;
		longi += dlongi * delta_t;
		high += vel_n[2] * delta_t;

		t += delta_t;
		static int issave = 0;
		if (issave)
		{
			printf_data(t, fide, route);			
		}
		issave = 1 - issave;

	}
	return t;
}
void IMUOUT::printf_data(double t, FILE *fide, Route route)
{
	//                  |                    |                    |                    |                    |           |       |
	fprintf_s(fide, "%lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%lf,%lf,%lf,%lf,%lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
		t,
		gyro_wib_b[0], gyro_wib_b[1], gyro_wib_b[2],
		acce_b[0], acce_b[1], acce_b[2],
		att_angle[0], att_angle[1], att_angle[2],
		vel_n[0], vel_n[1], vel_n[2],
		lati, longi, high,
		vel_t, route.at[1],
		wib_b[0], wib_b[1], wib_b[2],
		f_b[0], f_b[1], f_b[2]);
	fflush(fide);
}
