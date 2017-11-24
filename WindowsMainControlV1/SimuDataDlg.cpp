// SimuDataDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "WindowsMainControlV1.h"
#include "SimuDataDlg.h"
#include "afxdialogex.h"

#include "Calibration.h"

// SimuDataDlg �Ի���

IMPLEMENT_DYNAMIC(SimuDataDlg, CDialogEx)

SimuDataDlg::SimuDataDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(SimuDataDlg::IDD, pParent)
{
	routenum = 0;
	ordernum = 0;
	justaddswing = false;
	Iputfid = NULL;
	lati = 32.057305 ;
	longi = 118.786362 ;
	high = 0;
	creat_swing = false;
	route_show.phi0[0] = 0;
	is_combomove = false;
}

SimuDataDlg::~SimuDataDlg()
{
}

void SimuDataDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_EDIT_SwingAmplitude1, route_show.Asw[0]);
	DDX_Text(pDX, IDC_EDIT_SwingCenter1, route_show.Csw[0]);
	DDX_Text(pDX, IDC_EDIT_SwingFrequency1, route_show.fsw[0]);
	DDX_Text(pDX, IDC_EDIT_InitPhase1, route_show.phi0[0]);
	DDX_Text(pDX, IDC_EDIT_SwingTime1, route_show.tsw[0]);
	DDX_Text(pDX, IDC_EDIT_AX_DelayTime1, route_show.time_delay[0]);

	DDX_Text(pDX, IDC_EDIT_SwingAmplitude2, route_show.Asw[1]);
	DDX_Text(pDX, IDC_EDIT_SwingCenter2, route_show.Csw[1]);
	DDX_Text(pDX, IDC_EDIT_SwingFrequency2, route_show.fsw[1]);
	DDX_Text(pDX, IDC_EDIT_InitPhase2, route_show.phi0[1]);
	DDX_Text(pDX, IDC_EDIT_SwingTime2, route_show.tsw[1]);
	DDX_Text(pDX, IDC_EDIT_AX_DelayTime2, route_show.time_delay[1]);

	DDX_Text(pDX, IDC_EDIT_SwingAmplitude3, route_show.Asw[2]);
	DDX_Text(pDX, IDC_EDIT_SwingCenter3, route_show.Csw[2]);
	DDX_Text(pDX, IDC_EDIT_SwingFrequency3, route_show.fsw[2]);
	DDX_Text(pDX, IDC_EDIT_InitPhase3, route_show.phi0[2]);
	DDX_Text(pDX, IDC_EDIT_SwingTime3, route_show.tsw[2]);
	DDX_Text(pDX, IDC_EDIT_AX_DelayTime3, route_show.time_delay[2]);

	DDX_Text(pDX, IDC_EDIT_MoveTimeOrAngle, route_show.at[0]);
	DDX_Text(pDX, IDC_EDIT_MoveVOrW, route_show.wt[0]);	
	DDX_Control(pDX, IDC_COMBO_AxisXmode, SD_Combo_AxisXmode);
	DDX_Control(pDX, IDC_COMBO_AxisYmode, SD_Combo_AxisYmode);
	DDX_Control(pDX, IDC_COMBO_AxisZmode, SD_Combo_AxisZmode);
	DDX_Control(pDX, IDC_COMBO_MoveState, SD_Combo_turn);	
	DDX_Control(pDX, IDC_LIST_MoveOrder, SD_list_Moveorder);
	DDX_Text(pDX, IDC_EDIT_SDLati, lati);
	DDX_Text(pDX, IDC_EDIT_SDLongi, longi);
	DDX_Text(pDX, IDC_EDIT_SDHeight, high);


	init_combo();

}



BEGIN_MESSAGE_MAP(SimuDataDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BTN_AddErr, &SimuDataDlg::OnBnClickedBtnAdderr)
	ON_BN_CLICKED(IDC_BTN_SaveSimuFile, &SimuDataDlg::OnBnClickedBtnSavesimufile)
	ON_BN_CLICKED(IDC_BTN_AddSwing, &SimuDataDlg::OnBnClickedBtnAddswing)
	ON_BN_CLICKED(IDC_BTN__DelRoute, &SimuDataDlg::OnBnClickedBtn)
	ON_BN_CLICKED(IDC_BTN_AddMove, &SimuDataDlg::OnBnClickedBtnAddmove)
	ON_CBN_SELCHANGE(IDC_COMBO_MoveState, &SimuDataDlg::OnCbnSelchangeComboMovestate)
	ON_BN_CLICKED(IDC_BTN_CreateSimuData, &SimuDataDlg::OnBnClickedBtnCreatesimudata)
	ON_BN_CLICKED(IDCANCEL, &SimuDataDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDOK, &SimuDataDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDC_BTN_AddRecombMove, &SimuDataDlg::OnBnClickedBtnAddrecombmove)
END_MESSAGE_MAP()


// SimuDataDlg ��Ϣ�������
void SimuDataDlg::init_combo()
{
	int judge_tf;
	int i;
	CString str[] = { _T("ҡ��"),_T("��λ"),_T("����") };
	for (i = 0; i<1; i++)
	{
		SD_Combo_AxisXmode.InsertString(i, str[i]);
		SD_Combo_AxisYmode.InsertString(i, str[i]);
		SD_Combo_AxisZmode.InsertString(i, str[i]);
	}
	SD_Combo_AxisXmode.SetCurSel(0);
	SD_Combo_AxisYmode.SetCurSel(0);
	SD_Combo_AxisZmode.SetCurSel(0);

	CString str2[] = { _T("ֱ�߼�/����"),_T("ֱ������"),_T("��ת") ,_T("��ת"),_T("��̧"),_T("��ѹ"),_T("����"),_T("����") };
	for (i = 0; i<8; i++)
	{
		judge_tf = SD_Combo_turn.InsertString(i, str2[i]);
		if ((judge_tf == CB_ERR) || (judge_tf == CB_ERRSPACE))
		{
			MessageBox(_T("build baud error!"));
		}
	}
	SD_Combo_turn.SetCurSel(0);


}
void SimuDataDlg::OnBnClickedBtnAdderr()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	Calibration cali(NULL,true);
	cali.err_input = true;
	INT_PTR nRes;
	int i;
	nRes = cali.DoModal();
	if (IDOK == nRes)
	{
		IMUSimuout.err_parameter = cali.calipmt;
		amamul(3, 1, IMUSimuout.err_parameter.bias_acce, IMUSimuout.err_parameter.bias_acce, 9.78*0.000001);	//*ug	
		amamul(3, 1, IMUSimuout.err_parameter.bias_acce_random, IMUSimuout.err_parameter.bias_acce_random, 9.78*0.000001);
		amamul(3, 1, IMUSimuout.err_parameter.bias_gyro, IMUSimuout.err_parameter.bias_gyro, D2R / 3600);
		amamul(3, 1, IMUSimuout.err_parameter.bias_gyro_random, IMUSimuout.err_parameter.bias_gyro_random, D2R / 3600);
		IMUSimuout.err_parameter.Eang2mat();
	}
	else//û������͸�һ��Ĭ��ֵ
	{
		for (i = 0; i < 3; i++)
		{
			IMUSimuout.err_parameter.bias_acce[i] = 50* 9.78*0.000001;
			IMUSimuout.err_parameter.bias_acce_random[i] = 20 * 9.78*0.000001;
			IMUSimuout.err_parameter.bias_gyro[i] = 0.02* D2R / 3600;
			IMUSimuout.err_parameter.bias_gyro_random[i] = 0.006* D2R / 3600;
		}
	}
}


void SimuDataDlg::OnBnClickedBtnSavesimufile()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString szFilter;
	if (Iputfid != NULL) fclose(Iputfid);
	szFilter = "*.txt|*.txt|All Files (*.*)|*.*||";
	CFileDialog m_FileDialog0(true, NULL, NULL, 0, szFilter, NULL);
	if (m_FileDialog0.DoModal())
	{
		if ((filename = m_FileDialog0.GetPathName()) == "")
		{			
			return;
		}
		else
		{
			filename.Replace(_T(".txt"), _T(""));
			fopen_s(&Iputfid, filename + ".txt", "w");
			GetDlgItem(IDC_BTN_CreateSimuData)->EnableWindow(true);
		}
	}
}


void SimuDataDlg::OnBnClickedBtnAddswing()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString str1=NULL,tempStr;
	int x, y, z, i;
	x = SD_Combo_AxisXmode.GetCurSel();
	y = SD_Combo_AxisYmode.GetCurSel();
	z = SD_Combo_AxisZmode.GetCurSel();
	CString phi0,fsw,Asw,tsw,Csw,Tdelay;	

//	if (justaddswing == true )
//		routenum--;
	if (false==is_combomove)
	{
		CString str1, tempStr;
		tempStr.Format(_T("%d"), routenum + 1);
		str1 = tempStr;
		str1 += ",ҡ���˶�";
		SD_list_Moveorder.AddString(str1);
		ordernum += 1;
		route[routenum].var1++;//��ʾ������
	}
	GetDlgItemText(IDC_EDIT_SwingCenter1, Csw);
	route_show.Csw[0] = _ttof(Csw);
	GetDlgItemText(IDC_EDIT_SwingFrequency1, fsw);
	route_show.fsw[0] = _ttof(fsw);
	GetDlgItemText(IDC_EDIT_SwingTime1, tsw);
	route_show.tsw[0] = _ttof(tsw);
	GetDlgItemText(IDC_EDIT_SwingAmplitude1, Asw);
	route_show.Asw[0] = _ttof(Asw);
	GetDlgItemText(IDC_EDIT_InitPhase1, phi0);
	route_show.phi0[0] = _ttof(phi0);
	GetDlgItemText(IDC_EDIT_InitPhase1, Tdelay);
	route_show.time_delay[0] = _ttof(Tdelay);

	GetDlgItemText(IDC_EDIT_SwingCenter2, Csw);
	route_show.Csw[1] = _ttof(Csw);
	GetDlgItemText(IDC_EDIT_SwingFrequency2, fsw);
	route_show.fsw[1] = _ttof(fsw);
	GetDlgItemText(IDC_EDIT_SwingTime2, tsw);
	route_show.tsw[1] = _ttof(tsw);
	GetDlgItemText(IDC_EDIT_SwingAmplitude2, Asw);
	route_show.Asw[1] = _ttof(Asw);
	GetDlgItemText(IDC_EDIT_InitPhase2, phi0);
	route_show.phi0[1] = _ttof(phi0);
	GetDlgItemText(IDC_EDIT_InitPhase2, Tdelay);
	route_show.time_delay[1] = _ttof(Tdelay);

	GetDlgItemText(IDC_EDIT_SwingCenter3, Csw);
	route_show.Csw[2] = _ttof(Csw);
	GetDlgItemText(IDC_EDIT_SwingFrequency3, fsw);
	route_show.fsw[2] = _ttof(fsw);
	GetDlgItemText(IDC_EDIT_SwingTime3, tsw);
	route_show.tsw[2] = _ttof(tsw);
	GetDlgItemText(IDC_EDIT_SwingAmplitude3, Asw);
	route_show.Asw[2] = _ttof(Asw);
	GetDlgItemText(IDC_EDIT_InitPhase3, phi0);
	route_show.phi0[2] = _ttof(phi0);
	GetDlgItemText(IDC_EDIT_InitPhase3, Tdelay);
	route_show.time_delay[2] = _ttof(Tdelay);

	double ttt = route_show.tsw[0] + route_show.tsw[1] + route_show.tsw[2];
	if (ttt == 0)
	{
		MessageBox(_T("ʱ���趨Ϊ0�����ʧ��"), _T("error"), MB_OK | MB_SYSTEMMODAL);
		creat_swing = false;
		return;
	}

	for (i = 0; i < 3; i++)
	{
		route[routenum].Asw[i] = route_show.Asw[i] * D2R;
		route[routenum].fsw[i] = route_show.fsw[i];
		route[routenum].phi0[i] = route_show.phi0[i] * D2R;
		route[routenum].tsw[i] = route_show.tsw[i];
		route[routenum].timev[i] = route_show.tsw[i];
		route[routenum].Csw[i] = route_show.Csw[i] * D2R;
		route[routenum].time_delay[i] = route_show.time_delay[i] * D2R;
		route[routenum].state |= 0x01;
		switch (i)
		{
		case 0:str1 = "��ҡ ҡ�ڷ��ȣ�"; break;
		case 1:str1 = "��ҡ ҡ�ڷ��ȣ�"; break;
		case 2:str1 = "���� ҡ�ڷ��ȣ�"; break;
		default: break;
		}
		tempStr.Format(_T("%lf"), route_show.Asw[i]);
		str1 += tempStr;
		str1 += "��,Ƶ�ʣ�";
		tempStr.Format(_T("%lf"), route_show.fsw[i]);
		str1 += tempStr;
		str1 += "Hz";
		str1 += "���ģ�";
		tempStr.Format(_T("%lf"), route_show.Csw[i]);
		str1 += tempStr;
		str1 += "�㡣";
		SD_list_Moveorder.AddString(str1);
		str1 = "���ࣺ";
		tempStr.Format(_T("%lf"), route_show.phi0[i]);
		str1 += tempStr;
		str1 += "��,ʱ�䣺";
		tempStr.Format(_T("%lf"), route_show.tsw[i]);
		str1 += tempStr;
		str1 += "s,��ʱ��";
		tempStr.Format(_T("%lf"), route_show.time_delay[i]);
		str1 += tempStr;
		SD_list_Moveorder.AddString(str1);
		ordernum += 2;
	}

	route[routenum].var1+=6;//��ʾ������
	routenum++;
	justaddswing = true;	
	creat_swing = true;
}


void SimuDataDlg::OnBnClickedBtn()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������	
	bool issw = false;
	int i;
	int var1=0;
	if (routenum > 0)
	{
		routenum--;
		var1 = route[routenum].var1;	
		route[routenum].reset();
	}
	for (i = 0; i < var1; i++)
	{
		if (ordernum <= 0)
			return;
		ordernum--;
		SD_list_Moveorder.DeleteString(ordernum);
		
	}		
}


void SimuDataDlg::OnBnClickedBtnAddmove()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	int i = SD_Combo_turn.GetCurSel();
	CString str1 = NULL, tempStr, str;

	CString at, wt;
	GetDlgItemText(IDC_EDIT_MoveTimeOrAngle, at);
	route_show.at[0] = _ttof(at);
	GetDlgItemText(IDC_EDIT_MoveVOrW, wt);
	route_show.wt[0] = _ttof(wt);
	if (false == is_combomove)
	{
		CString str1, tempStr;
		tempStr.Format(_T("%d"), routenum + 1);
		str1 = tempStr;
		str1 += ",�н��˶�";
		SD_list_Moveorder.AddString(str1);
		ordernum += 1;
		route[routenum].var1++;//��ʾ������
	}
	switch (i)
	{
	case 0:
		route[routenum++].create_v(route_show.at[0], route_show.wt[0]);
		str1 += "�仯��Ŀ���ٶȣ�";
		tempStr.Format(_T("%lf"), route_show.wt[0]);
		str1 += tempStr;
		str1 += "m/s������ʱ�䣺";
		tempStr.Format(_T("%lf"), route_show.at[0]);
		str1 += tempStr;
		str1 += "s��";
		break;
	case 1:
		route[routenum++].create_a(route_show.at[0]);
		str1 += "�����˶�������ʱ�䣺";
		tempStr.Format(_T("%lf"), route_show.at[0]);
		str1 += tempStr;
		str1 += "s��";
		break;
	case 2:
		route[routenum++].create_turn(route_show.at[0], route_show.wt[0]);
		str1 += "��ת��";
		tempStr.Format(_T("%lf"), route_show.at[0]);
		str1 += tempStr;
		str1 += "�㣬ת�٣�";
		tempStr.Format(_T("%lf"), route_show.wt[0]);
		str1 += tempStr;
		str1 += "��/s��";
		break;
	case 3:
		route[routenum++].create_turn(route_show.at[0], -route_show.wt[0]);
		str1 += "��ת��";
		tempStr.Format(_T("%lf"), route_show.at[0]);
		str1 += tempStr;
		str1 += "�㣬ת�٣�";
		tempStr.Format(_T("%lf"), route_show.wt[0]);
		str1 += tempStr;
		str1 += "��/s��";
		break;
	case 4:
		route[routenum++].create_p(route_show.at[0], route_show.wt[0]);
		str1 += "��̧��";
		tempStr.Format(_T("%lf"), route_show.at[0]);
		str1 += tempStr;
		str1 += "�㣬ת�٣�";
		tempStr.Format(_T("%lf"), route_show.wt[0]);
		str1 += tempStr;
		str1 += "��/s��";
		break;
	case 5:
		route[routenum++].create_p(route_show.at[0], -route_show.wt[0]);
		str1 += "��ѹ��";
		tempStr.Format(_T("%lf"), route_show.at[0]);
		str1 += tempStr;
		str1 += "�㣬ת�٣�";
		tempStr.Format(_T("%lf"), route_show.wt[0]);
		str1 += tempStr;
		str1 += "��/s��";
		break;
	case 6:
		route[routenum++].create_r(route_show.at[0], route_show.wt[0]);
		str1 += "���㣺";
		tempStr.Format(_T("%lf"), route_show.at[0]);
		str1 += tempStr;
		str1 += "�㣬ת�٣�";
		tempStr.Format(_T("%lf"), route_show.wt[0]);
		str1 += tempStr;
		str1 += "��/s��";
		break;
	case 7:
		route[routenum++].create_r(route_show.at[0], -route_show.wt[0]);
		str1 += "���㣺";
		tempStr.Format(_T("%lf"), route_show.at[0]);
		str1 += tempStr;
		str1 += "�㣬ת�٣�";
		tempStr.Format(_T("%lf"), route_show.wt[0]);
		str1 += tempStr;
		str1 += "��/s��";
		break;
	default: break;
	}
	justaddswing = false;
	route[routenum-1].var1++;//��ʾ������
	ordernum ++;	
	SD_list_Moveorder.AddString(str1);
}


void SimuDataDlg::OnCbnSelchangeComboMovestate()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	int i = SD_Combo_turn.GetCurSel();
	CString str1 = NULL, tempStr, str;
	switch (i)
	{
	case 0:
		str = _T("ʱ��");
		SetDlgItemText(IDC_STATIC_T, str);
		str = _T("Ŀ���ٶ�");
		SetDlgItemText(IDC_STATIC_VorW, str);
		str = _T("s");
		SetDlgItemText(IDC_STATIC_Tunit, str);
		str = _T("m/s");
		SetDlgItemText(IDC_STATIC_Vunit, str);
		GetDlgItem(IDC_EDIT_MoveVOrW)->EnableWindow(true);
		break;
	case 1:
		str = _T("ʱ��");
		SetDlgItemText(IDC_STATIC_T, str);		
		str = _T("s");
		SetDlgItemText(IDC_STATIC_Tunit, str);
		GetDlgItem(IDC_EDIT_MoveVOrW)->EnableWindow(false);		
		break;
	case 2:
		str = _T("ת���Ƕ�");
		SetDlgItemText(IDC_STATIC_T, str);
		str = _T("ת�����ٶ�");
		SetDlgItemText(IDC_STATIC_VorW, str);
		str = _T("��");
		SetDlgItemText(IDC_STATIC_Tunit, str);
		str = _T("��/s");
		SetDlgItemText(IDC_STATIC_Vunit, str);
		GetDlgItem(IDC_EDIT_MoveVOrW)->EnableWindow(true);
		break;
	case 3:
		str = _T("ת���Ƕ�");
		SetDlgItemText(IDC_STATIC_T, str);
		str = _T("ת�����ٶ�");
		SetDlgItemText(IDC_STATIC_VorW, str);
		str = _T("��");
		SetDlgItemText(IDC_STATIC_Tunit, str);
		str = _T("��/s");
		SetDlgItemText(IDC_STATIC_Vunit, str);
		GetDlgItem(IDC_EDIT_MoveVOrW)->EnableWindow(true);
		break;
	case 4:
		str = _T("ת���Ƕ�");
		SetDlgItemText(IDC_STATIC_T, str);
		str = _T("ת�����ٶ�");
		SetDlgItemText(IDC_STATIC_VorW, str);
		str = _T("��");
		SetDlgItemText(IDC_STATIC_Tunit, str);
		str = _T("��/s");
		SetDlgItemText(IDC_STATIC_Vunit, str);
		GetDlgItem(IDC_EDIT_MoveVOrW)->EnableWindow(true);
		break;
	case 5:
		str = _T("ת���Ƕ�");
		SetDlgItemText(IDC_STATIC_T, str);
		str = _T("ת�����ٶ�");
		SetDlgItemText(IDC_STATIC_VorW, str);
		str = _T("��");
		SetDlgItemText(IDC_STATIC_Tunit, str);
		str = _T("��/s");
		SetDlgItemText(IDC_STATIC_Vunit, str);
		GetDlgItem(IDC_EDIT_MoveVOrW)->EnableWindow(true);
		break;
	case 6:
		str = _T("ת���Ƕ�");
		SetDlgItemText(IDC_STATIC_T, str);
		str = _T("ת�����ٶ�");
		SetDlgItemText(IDC_STATIC_VorW, str);
		str = _T("��");
		SetDlgItemText(IDC_STATIC_Tunit, str);
		str = _T("��/s");
		SetDlgItemText(IDC_STATIC_Vunit, str);
		GetDlgItem(IDC_EDIT_MoveVOrW)->EnableWindow(true);
		break;
	case 7:
		str = _T("ת���Ƕ�");
		SetDlgItemText(IDC_STATIC_T, str);
		str = _T("ת�����ٶ�");
		SetDlgItemText(IDC_STATIC_VorW, str);
		str = _T("��");
		SetDlgItemText(IDC_STATIC_Tunit, str);
		str = _T("��/s");
		SetDlgItemText(IDC_STATIC_Vunit, str);
		GetDlgItem(IDC_EDIT_MoveVOrW)->EnableWindow(true);
		break;
	default: break;
	}
}


void SimuDataDlg::OnBnClickedBtnCreatesimudata()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	double t = 0;
	int i;
	if (Iputfid == NULL)
	{
		MessageBox(_T("δѡ�񱣴��ļ���"), _T("error"), MB_OK | MB_SYSTEMMODAL);
		return;
	}		
	CString temp;
	GetDlgItemText(IDC_EDIT_SDLongi, temp);
	IMUSimuout.longi = _ttof(temp)*D2R;
	GetDlgItemText(IDC_EDIT_SDLati, temp);
	IMUSimuout.lati = _ttof(temp)*D2R;
	GetDlgItemText(IDC_EDIT_SDHeight, temp);
	IMUSimuout.high = _ttof(temp);

	for (i = 0; i < routenum; i++)
	{
		t = IMUSimuout.generate_data(route[i], t, 0.0025, Iputfid);
	}
	if (Iputfid != NULL) fclose(Iputfid);

	GetDlgItem(IDC_BTN_CreateSimuData)->EnableWindow(false);
	MessageBox(_T("���ݷ�����ɣ�"), _T("success"), MB_OK | MB_SYSTEMMODAL);
}


void SimuDataDlg::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if (Iputfid != NULL) fclose(Iputfid);
	CDialogEx::OnCancel();
}


void SimuDataDlg::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	if (Iputfid != NULL) fclose(Iputfid);
	CDialogEx::OnOK();
}

void SimuDataDlg::OnBnClickedBtnAddrecombmove()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	is_combomove = true;
	CString str1, tempStr;
	tempStr.Format(_T("%d"), routenum + 1);
	str1 = tempStr;
	str1 += ",�����˶�";
	SD_list_Moveorder.AddString(str1);
	ordernum += 1;
	route[routenum].var1++;//��ʾ������
	OnBnClickedBtnAddswing();	
	
	if (creat_swing)
	{
		routenum--;
		OnBnClickedBtnAddmove();
	}
	is_combomove = false;
}
