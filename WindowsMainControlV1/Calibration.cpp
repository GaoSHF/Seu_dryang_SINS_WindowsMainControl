// Calibration.cpp : 实现文件
//

#include "stdafx.h"
#include "WindowsMainControlV1.h"
#include "Calibration.h"
#include "afxdialogex.h"


// Calibration 对话框

IMPLEMENT_DYNAMIC(Calibration, CDialogEx)

Calibration::Calibration(CWnd* pParent, bool iserr_input/*=NULL, = false*/)
	: CDialogEx(Calibration::IDD, pParent)
{
	int i;
	if (iserr_input == true)
		for (i = 0; i < 3; i++)
		{
			calipmt.bias_acce[i] = 50;
			calipmt.bias_acce_random[i] = 20;
			calipmt.bias_gyro[i] = 0.02;
			calipmt.bias_gyro_random[i] = 0.006;
		}
}

Calibration::~Calibration()
{
}

void Calibration::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_ACCC_BIAS_x, calipmt.bias_acce[0]);
	DDX_Text(pDX, IDC_ACCC_BIAS_y, calipmt.bias_acce[1]);
	DDX_Text(pDX, IDC_ACCC_BIAS_z, calipmt.bias_acce[2]);
	DDX_Text(pDX, IDC_GYRO_BIAS_x, calipmt.bias_gyro[0]);
	DDX_Text(pDX, IDC_GYRO_BIAS_y, calipmt.bias_gyro[1]);
	DDX_Text(pDX, IDC_GYRO_BIAS_z, calipmt.bias_gyro[2]);
	DDX_Text(pDX, IDC_FIX_ERR0, calipmt.fix_err[0]);
	DDX_Text(pDX, IDC_FIX_ERR1, calipmt.fix_err[1]);
	DDX_Text(pDX, IDC_FIX_ERR2, calipmt.fix_err[2]);
	DDX_Text(pDX, IDC_ACCC_Ca_x, calipmt.Ca[0]);
	DDX_Text(pDX, IDC_ACCC_Ca_y, calipmt.Ca[1]);
	DDX_Text(pDX, IDC_ACCC_Ca_z, calipmt.Ca[2]);
	DDX_Text(pDX, IDC_GYRO_Cg_x, calipmt.Cg[0]);
	DDX_Text(pDX, IDC_GYRO_Cg_y, calipmt.Cg[1]);
	DDX_Text(pDX, IDC_GYRO_Cg_z, calipmt.Cg[2]);
	DDX_Text(pDX, IDC_ACCC_Ea_xy, calipmt.Ea_ang[0]);
	DDX_Text(pDX, IDC_ACCC_Ea_yx, calipmt.Ea_ang[1]);
	DDX_Text(pDX, IDC_ACCC_Ea_yz, calipmt.Ea_ang[2]);
	DDX_Text(pDX, IDC_ACCC_Ea_zy, calipmt.Ea_ang[3]);
	DDX_Text(pDX, IDC_ACCC_Ea_xz, calipmt.Ea_ang[4]);
	DDX_Text(pDX, IDC_ACCC_Ea_zx, calipmt.Ea_ang[5]);
	DDX_Text(pDX, IDC_ACCC_Eg_xy, calipmt.Eg_ang[0]);
	DDX_Text(pDX, IDC_ACCC_Eg_yx, calipmt.Eg_ang[1]);
	DDX_Text(pDX, IDC_ACCC_Eg_yz, calipmt.Eg_ang[2]);
	DDX_Text(pDX, IDC_ACCC_Eg_zy, calipmt.Eg_ang[3]);
	DDX_Text(pDX, IDC_ACCC_Eg_xz, calipmt.Eg_ang[4]);
	DDX_Text(pDX, IDC_ACCC_Eg_zx, calipmt.Eg_ang[5]);
	DDX_Text(pDX, IDC_ACCC_BIAS_R_x, calipmt.bias_acce_random[0]);
	DDX_Text(pDX, IDC_ACCC_BIAS_R_y, calipmt.bias_acce_random[1]);
	DDX_Text(pDX, IDC_ACCC_BIAS_R_z, calipmt.bias_acce_random[2]);
	DDX_Text(pDX, IDC_GYRO_BIAS_R_x, calipmt.bias_gyro_random[0]);
	DDX_Text(pDX, IDC_GYRO_BIAS_R_y, calipmt.bias_gyro_random[1]);
	DDX_Text(pDX, IDC_GYRO_BIAS_R_z, calipmt.bias_gyro_random[2]);
		
	if (err_input == true)
	{
		GetDlgItem(IDC_STATIC_BIAS_R)->ShowWindow(true);
		GetDlgItem(IDC_STATIC_ACCE_R)->ShowWindow(true);
		GetDlgItem(IDC_STATIC_GYRO_R)->ShowWindow(true);
		GetDlgItem(IDC_ACCC_BIAS_R_x)->ShowWindow(true);
		GetDlgItem(IDC_ACCC_BIAS_R_y)->ShowWindow(true);
		GetDlgItem(IDC_ACCC_BIAS_R_z)->ShowWindow(true);
		GetDlgItem(IDC_GYRO_BIAS_R_x)->ShowWindow(true);
		GetDlgItem(IDC_GYRO_BIAS_R_y)->ShowWindow(true);
		GetDlgItem(IDC_GYRO_BIAS_R_z)->ShowWindow(true);
	
		
	}
	else
	{
		GetDlgItem(IDC_STATIC_BIAS_R)->ShowWindow(false);
		GetDlgItem(IDC_STATIC_ACCE_R)->ShowWindow(false);
		GetDlgItem(IDC_STATIC_GYRO_R)->ShowWindow(false);
		GetDlgItem(IDC_ACCC_BIAS_R_x)->ShowWindow(false);
		GetDlgItem(IDC_ACCC_BIAS_R_y)->ShowWindow(false);
		GetDlgItem(IDC_ACCC_BIAS_R_z)->ShowWindow(false);
		GetDlgItem(IDC_GYRO_BIAS_R_x)->ShowWindow(false);
		GetDlgItem(IDC_GYRO_BIAS_R_y)->ShowWindow(false);
		GetDlgItem(IDC_GYRO_BIAS_R_z)->ShowWindow(false);
	}
}


BEGIN_MESSAGE_MAP(Calibration, CDialogEx)
END_MESSAGE_MAP()


// Calibration 消息处理程序
