


// WindowsMainControlV1Dlg.cpp : 实现文件
//

#include "stdafx.h"
#include "WindowsMainControlV1.h"
#include "WindowsMainControlV1Dlg.h"
#include "afxdialogex.h"
#include "BoardCard.h"

#include "CoarseAlign.h"
#include "FineAlign.h"
#include "Navigation.h"
#include "Calibration.h"
#include "ReadSimuDlg.h"
#include "SimuDataDlg.h"
#include <WinSock2.h>
#include <MMSystem.h>
#include "afxwin.h"
#pragma comment(lib,"WS2_32.lib")  
#pragma comment(lib, "winmm.lib")
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#pragma region VarDef 
//*******非对话框类变量声明*******//
bool blrxreset;//板卡通道复位许可
//文件保存相关
FILE* fid_FOSN;
FILE* fid_PHINS;
FILE* fid_gps;
FILE* fid_zt;
FILE* fid_Cal;
bool is_SingleFile;//为1表示单文件保存，2表示多文件保存
bool saveStart;
CWinThread* phins_Thread;
//刷新率设置
int refresh_time;
bool is_UpdateData;//刷新关闭和开启
//定时存数
int save_time;//保存定时常数
int m_save_time;//显示定时常数，倒计时
int temp_cnt_s;//用于减去点倒计时已经过去的系统时间
bool is_timeSave;

//初始位置,显示用，单位为°
double initial_latitude, initial_longitude, initial_height;
double real_pos[3];//计算位置误差用
//模式选择
int CoarseModeNum;
int FineModeNum;
int NaviModeNum;
int TestModeNum;
int PureINSModeNum;
bool isStartCalOk;
//标志判断
bool is_cardReceive;
bool is_startCal;
bool is_cardReset;//板卡重开
//bool is_InitNaviVar;//导航参数初始化完成标志
bool is_data_used;//接收数据是否已经被解算使用
bool is_start_phins;
bool mquit;//程序退出，用于线程循环退出控制
bool pc104RecQuit;//pc104接收线程退出
//FOSN数据
BYTE bufFOSN[70];
//gps数据
BYTE BufGPS[180];
////转台数据接收	
BYTE BufZT[50] = { 0 };
int m_cnt_err;//帧差

//解算数据
int m_cnt_s;
//PHINS数据
char m_Recv_PHINS_Buff[256];

//读数仿真模式
READSIMULATION RS_para;
//多媒体定时器
void CALLBACK TimeDalay(UINT uID, UINT uMsg, DWORD dwUsers, DWORD dw1, DWORD dw2);
UINT TimerRes = 5;
UINT TimerID;
bool isCreateTimer = false;//创建定时器事件
int m_PRecNum;//纯录数录取的数据量
int datanavinum = 0;
bool temp_test = true;
double temp_ang[3], temp_pos[3], temp_v[3];
#pragma endregion VarDef 

// 用于应用程序“关于”菜单项的 CAboutDlg 对话框
class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

	// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};
CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}
void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()

// CWindowsMainControlV1Dlg 对话框
CWindowsMainControlV1Dlg::CWindowsMainControlV1Dlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CWindowsMainControlV1Dlg::IDD, pParent)
	, edit_data_f(100)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	initial_latitude = 32.05731;
	initial_longitude = 118.786355;
	initial_height = 31;
	Init_CardVar();

	is_SingleFile = 1;

	CoarseModeNum = 0;
	FineModeNum = 0;
	NaviModeNum = 0;
	PureINSModeNum = 0;
	TestModeNum = 0;

	sysc.coarse_time = 80;
	sysc.fine_level = 120;
	sysc.fine_azimuth = 400;
	sysc.algn_time = 600;

	refresh_time = 1000;
	is_UpdateData = true;
	is_cardReset = 1;
	mquit = 0;
	pc104RecQuit = true;
	init_var();
}
void CWindowsMainControlV1Dlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_CoarseAlignMode, m_CoarseAignMode);
	DDX_Control(pDX, IDC_COMBO_FineAlignMode, m_FineAignMode);
	DDX_Control(pDX, IDC_COMBO_NavigationMode, m_NaviMode);
	DDX_Control(pDX, IDC_COMBO_HeightMode, m_HeightMode);
	DDX_Text(pDX, IDC_COARSE_TIME, sysc.coarse_time);
	DDX_Text(pDX, IDC_FINE_LEVEL, sysc.fine_level);
	DDX_Text(pDX, IDC_FINE_AZIMUTH, sysc.fine_azimuth);
	DDX_Text(pDX, IDC_FINE_TIME, sysc.algn_time);
	DDX_Text(pDX, IDC_FOSN_UTC1, fosn.s);
	DDX_Text(pDX, IDC_FOSN_UTC2, fosn.ms);
	DDX_Text(pDX, IDC_FOSN_GRYO_X, IMUout.gyro_b[0]);
	DDX_Text(pDX, IDC_FOSN_GRYO_Y, IMUout.gyro_b[1]);
	DDX_Text(pDX, IDC_FOSN_GRYO_Z, IMUout.gyro_b[2]);
	DDX_Text(pDX, IDC_FOSN_ACCE_X, IMUout.acce_b[0]);
	DDX_Text(pDX, IDC_FOSN_ACCE_Y, IMUout.acce_b[1]);
	DDX_Text(pDX, IDC_FOSN_ACCE_Z, IMUout.acce_b[2]);
	DDX_Text(pDX, IDC_FOSN_Latitude, fosn.pos[0]);
	DDX_Text(pDX, IDC_FOSN_Longitude, fosn.pos[1]);
	DDX_Text(pDX, IDC_FOSN_Height, fosn.pos[2]);
	DDX_Text(pDX, IDC_FOSN_V_X, fosn.vel[0]);
	DDX_Text(pDX, IDC_FOSN_V_Y, fosn.vel[1]);
	DDX_Text(pDX, IDC_FOSN_V_Z, fosn.vel[2]);
	DDX_Text(pDX, IDC_FOSN_Angle_X, fosn.ang[0]);
	DDX_Text(pDX, IDC_FOSN_Angle_Y, fosn.ang[1]);
	DDX_Text(pDX, IDC_FOSN_Angle_Z, fosn.ang[2]);
	DDX_Text(pDX, IDC_FOSNREC_CNT, fosn.recnum);
	DDX_Text(pDX, IDC_ZT_Frame, ZT.Frame);
	DDX_Text(pDX, IDC_ZT_Count, ZT.cnt);
	DDX_Text(pDX, IDC_ZT_X, ZT.ang[0]);
	DDX_Text(pDX, IDC_ZT_Y, ZT.ang[1]);
	DDX_Text(pDX, IDC_ZT_Z, ZT.ang[2]);
	DDX_Text(pDX, IDC_CNT_ERR, m_cnt_err);
	DDX_Text(pDX, IDC_GPS_Latitude, gps.pos[0]);
	DDX_Text(pDX, IDC_GPS_Longitude, gps.pos[1]);
	DDX_Text(pDX, IDC_GPS_Height, gps.pos[2]);
	DDX_Text(pDX, IDC_GPS_Ve, gps.vel[0]);
	DDX_Text(pDX, IDC_GPS_Vn, gps.vel[1]);
	DDX_Text(pDX, IDC_GPS_Vu, gps.vel[2]);
	DDX_Text(pDX, IDC_GPS_Time, gps.time);
	DDX_Text(pDX, IDC_GPS_FLAG, gps.flag);
	DDX_Text(pDX, IDC_GPS_CNT, gps.cnt);
	DDX_Text(pDX, IDC_Refresh_Time, refresh_time);
	DDX_Text(pDX, IDC_Cal_Angle_X, INScal.ang[0]);
	DDX_Text(pDX, IDC_Cal_Angle_Y, INScal.ang[1]);
	DDX_Text(pDX, IDC_Cal_Angle_Z, INScal.ang[2]);
	DDX_Text(pDX, IDC_Cal_V_X, INScal.vel[0]);
	DDX_Text(pDX, IDC_Cal_V_Y, INScal.vel[1]);
	DDX_Text(pDX, IDC_Cal_V_Z, INScal.vel[2]);
	DDX_Text(pDX, IDC_Cal_Latitude, INScal.pos[0]);
	DDX_Text(pDX, IDC_Cal_Longitude, INScal.pos[1]);
	DDX_Text(pDX, IDC_Cal_HEIGHT, INScal.pos[2]);
	DDX_Text(pDX, IDC_ERR_Angle_X, INScal.err_ang[0]);
	DDX_Text(pDX, IDC_ERR_Angle_Y, INScal.err_ang[1]);
	DDX_Text(pDX, IDC_ERR_Angle_Z, INScal.err_ang[2]);
	DDX_Text(pDX, IDC_ERR_Vel_X, INScal.err_vel[0]);
	DDX_Text(pDX, IDC_ERR_Vel_Y, INScal.err_vel[1]);
	DDX_Text(pDX, IDC_ERR_Vel_Z, INScal.err_vel[2]);
	DDX_Text(pDX, IDC_ERR_LocationX, INScal.err_pos[0]);
	DDX_Text(pDX, IDC_ERR_LocationY, INScal.err_pos[1]);
	DDX_Text(pDX, IDC_ERR_Location, INScal.err_pos[2]);
	DDX_Text(pDX, IDC_LATITUDE0, initial_latitude);
	DDX_Text(pDX, IDC_LONGITUDE0, initial_longitude);
	DDX_Text(pDX, IDC_HEIGHT0, initial_height);
	DDX_Text(pDX, IDC_RUNNING_TIME, m_cnt_s);
	DDX_Text(pDX, IDC_PHINS_Agnle_X, phins.ang[0]);
	DDX_Text(pDX, IDC_PHINS_Agnle_Y, phins.ang[1]);
	DDX_Text(pDX, IDC_PHINS_Agnle_Z, phins.ang[2]);
	DDX_Text(pDX, IDC_PHINS_CNT, phins.cnt);
	DDX_Text(pDX, IDC_PHINS_Height, phins.pos[2]);
	DDX_Text(pDX, IDC_PHINS_Latitude, phins.pos[0]);
	DDX_Text(pDX, IDC_PHINS_Longitude, phins.pos[1]);
	DDX_Text(pDX, IDC_PHINS_UTC, phins.utc);
	DDX_Text(pDX, IDC_PHINS_V_X, phins.vel[0]);
	DDX_Text(pDX, IDC_PHINS_V_Y, phins.vel[1]);
	DDX_Text(pDX, IDC_PHINS_V_Z, phins.vel[2]);
	DDX_Text(pDX, IDC_SAVETIME, m_save_time);
	DDX_Text(pDX, IDC_Kal_gyro_Bias_1, infor.gyro_bias_esti[0]);
	DDX_Text(pDX, IDC_Kal_gyro_Bias_2, infor.gyro_bias_esti[1]);
	DDX_Text(pDX, IDC_Kal_gyro_Bias_3, infor.gyro_bias_esti[2]);
	DDX_Text(pDX, IDC_Kal_acce_Bias_1, infor.acce_bias_esti[0]);
	DDX_Text(pDX, IDC_Kal_acce_Bias_2, infor.acce_bias_esti[1]);
	DDX_Text(pDX, IDC_Kal_acce_Bias_3, infor.acce_bias_esti[2]);
	DDX_Control(pDX, IDC_TEXT_MODE, m_TestMode);
	DDX_Text(pDX, IDC_STATE, sysc.state);
	DDX_Text(pDX, IDC_Data_F, edit_data_f);
}
BEGIN_MESSAGE_MAP(CWindowsMainControlV1Dlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()

	ON_MESSAGE(WM_UPDATEDATA, updateData)
	ON_MESSAGE(WM_TIMESAVE, saveCancel)

	ON_BN_CLICKED(IDC_BTN_StartCard, &CWindowsMainControlV1Dlg::OnBnClickedBtnStartcard)
	ON_BN_CLICKED(IDC_BTN_StartPhins, &CWindowsMainControlV1Dlg::OnBnClickedBtnStartphins)
	ON_BN_CLICKED(IDC_BTN_StartCal, &CWindowsMainControlV1Dlg::OnBnClickedBtnStartcal)
	ON_BN_CLICKED(IDC_BTN_SAVEPATH, &CWindowsMainControlV1Dlg::OnBnClickedBtnSavepath)
	ON_BN_CLICKED(IDC_BTN_SAVEALL, &CWindowsMainControlV1Dlg::OnBnClickedBtnSaveall)
	ON_BN_CLICKED(IDC_BTN_SingleFile, &CWindowsMainControlV1Dlg::OnBnClickedBtnSinglefile)
	ON_BN_CLICKED(IDC_BTN_TimeSet, &CWindowsMainControlV1Dlg::OnBnClickedBtnTimeset)
	ON_BN_CLICKED(IDC_BTN_ReadMe, &CWindowsMainControlV1Dlg::OnBnClickedBtnReadme)
	ON_BN_CLICKED(IDC_BTN_SAVE_HELP, &CWindowsMainControlV1Dlg::OnBnClickedBtnSaveHelp)
	ON_BN_CLICKED(IDCANCEL, &CWindowsMainControlV1Dlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BTN_Reset, &CWindowsMainControlV1Dlg::OnBnClickedBtnReset)

	ON_BN_CLICKED(IDC_BTN_SAVETIME, &CWindowsMainControlV1Dlg::OnBnClickedBtnSavetime)
	ON_BN_CLICKED(IDC_BTN_STOPREFRESH, &CWindowsMainControlV1Dlg::OnBnClickedBtnStoprefresh)
	ON_BN_CLICKED(IDC_BTN_Calibration, &CWindowsMainControlV1Dlg::OnBnClickedBtnCalibration)
	ON_CBN_SELCHANGE(IDC_TEXT_MODE, &CWindowsMainControlV1Dlg::OnCbnSelchangeTextMode)
	ON_CBN_SELCHANGE(IDC_COMBO_FineAlignMode, &CWindowsMainControlV1Dlg::OnCbnSelchangeComboFinealignmode)
END_MESSAGE_MAP()

// CWindowsMainControlV1Dlg 消息处理程序

BOOL CWindowsMainControlV1Dlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码
	ghWnd = this->ghWnd;
	init_Combo();
	init_mainmode();
	Init_Net();
	timeBeginPeriod(TimerRes);
	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}
void CWindowsMainControlV1Dlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}
// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。
void CWindowsMainControlV1Dlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}
//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CWindowsMainControlV1Dlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}

/////↓↓↓↓***********按键响应****************↓↓↓↓//////
#pragma region ClBtn 
void CWindowsMainControlV1Dlg::OnBnClickedBtnStartcard()
{
	// TODO: 在此添加控件通知处理程序代码

	is_cardReceive = !is_cardReceive;
	CWnd *h1;
	h1 = GetDlgItem(IDC_BTN_StartCard);		//指向控件的caption
	CString str;
	if (is_cardReceive)
	{
		NaviModeNum = m_NaviMode.GetCurSel();
		TestModeNum = m_TestMode.GetCurSel();
		switch (TestModeNum)
		{
		case 1:if(!is_start_phins) OnBnClickedBtnStartphins(); break;
		case 2: case 3: sysc.state = _T("纯录数模式"); break;
		default:break;
		}
		Init_Card(ghWnd, NaviModeNum, 0x0f);
		Sio_Rx_ResetFIFO(hCard, 0);//复位板卡
		Sio_Rx_ResetFIFO(hCard, 1);
		Sio_Rx_ResetFIFO(hCard, 2);
		CWinThread* Card_Thread;
		THREADPARAM  *phWndParam = new THREADPARAM;
		phWndParam->hwnd = m_hWnd;
		if (!(Card_Thread = AfxBeginThread(CardRec, (LPVOID)phWndParam)))
			return;

		UpdateData(true);
		sysc.Fs = edit_data_f;
		sysc.Ts = 1.0 / sysc.Fs;
		str = _T("板卡接收停止");
		h1->SetWindowText(str);
		GetDlgItem(IDC_BTN_SAVEPATH)->EnableWindow(TRUE);
		GetDlgItem(IDC_BTN_StartCal)->EnableWindow(TRUE);
		m_TestMode.EnableWindow(FALSE);
	//	m_NaviMode.EnableWindow(FALSE);

	}
	else
	{
		str = _T("板卡接收");
		h1->SetWindowText(str);
		GetDlgItem(IDC_BTN_SAVEPATH)->EnableWindow(FALSE);
		GetDlgItem(IDC_BTN_StartCal)->EnableWindow(FALSE);
		m_TestMode.EnableWindow(TRUE);
	//	m_NaviMode.EnableWindow(TRUE);

	}
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnStartphins()
{
	// TODO: 在此添加控件通知处理程序代码
	is_start_phins = !is_start_phins;
	CWnd *h1;
	h1 = GetDlgItem(IDC_BTN_StartPhins);		//指向控件的caption
	CString str;
	if (is_start_phins)
	{		
		SOCKETPARAM  *pRecvParam = new SOCKETPARAM;
		pRecvParam->hwnd = m_hWnd;
		pRecvParam->sock = m_socketPHINSDataRec;
		if (!(phins_Thread = AfxBeginThread(PHINSThread, (LPVOID)pRecvParam)))
			return;
		str = _T("PHINS接收停止");
		h1->SetWindowText(str);
	}
	else
	{
		str = _T("PHINS接收");
		h1->SetWindowText(str);
		phins_Thread->PostThreadMessageW(WM_QUIT, NULL, NULL); 
	}
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnStartcal()
{
	// TODO: 在此添加控件通知处理程序代码
	TestModeNum = m_TestMode.GetCurSel();
	is_startCal = !is_startCal;
	CWnd *h1;
	h1 = GetDlgItem(IDC_BTN_StartCal);		//指向控件的caption
	CString str;
	if (is_startCal)
	{
		UpdateData(true);
		//根据模式进行初始化
		TestModeNum = m_TestMode.GetCurSel();
		CoarseModeNum = m_CoarseAignMode.GetCurSel();
		FineModeNum = m_FineAignMode.GetCurSel();
		PureINSModeNum = m_HeightMode.GetCurSel();
		NaviModeNum = m_NaviMode.GetCurSel();
		//初始位置参数传递		
		CalVarInit(TestModeNum);
		//捷联初始化
		init_basicnavi();
		sysc.Fs = edit_data_f;
		sysc.Ts = 1.0 / sysc.Fs;

		if (5 == FineModeNum)
			CoarseModeNum=0;		

		if (0 == CoarseModeNum)
		{
			sysc.f_coarse_over = 1;
			sysc.coarse_time = 0;
		}
		else
		{
			init_coarsealign();
			sysc.f_coarse_over = 0;
		}

		if (0 == FineModeNum)
		{
			sysc.f_fine_over = 1;
			sysc.algn_time = sysc.coarse_time;
		}
		else
		{
			sysc.f_fine_over = 0;
			switch (FineModeNum)
			{
			case FINE_CMPS:
				init_cmp(); //初始化罗经参数	
				if (FineModeNum == 1)sysc.algn_time = sysc.coarse_time + sysc.fine_level + sysc.fine_azimuth;//把水平和航向时间加起来得到精对准总时间，再加上粗对准的时间。
				break;	
			case FINE_Yucia: Kal_Init_P_15(fkalman,YA_POS); break;
			case FINE_0su: Kal_Init_P_15(fkalman, YA_VEL); break;
			case FINE_ADRC: init_adrc(); break;
			default: break;
			}
		}

		sysc.f_navi_over = 0;
		switch (NaviModeNum)
		{
		case NAVI_HAISHI_BASIC:case NAVI_HAISHI_JZ:kalinitial(); break;
		case NAVI_SG:case NAVI_PHINS_POS: Kal_Init_P_15(nkalman,YA_POS); break;   //20171128  导航阶段的kalman对象与精对准不用同一个
		case NAVI_VEL:case NAVI_PHINS_VEL1: Kal_Init_P_15(nkalman,YA_VEL); break; //20171128  导航阶段的kalman对象与精对准不用同一个
		case NAVI_PHINS_VEL2: Kal_Init_P_16(kalman_dvl); break;      //20171128 Vb组合的H阵初始值设0，需要实时更新。
		case NAVI_VELANDAZ:Kal_Init_P_15(nkalman,YA_VELANDAZ); break;  //20171128  导航阶段的kalman对象与精对准不用同一个

		default: break;
		}
		if (RS_para.delay5ms == 1)
		{
			isCreateTimer = true;
			TimerID = timeSetEvent(
				5,
				TimerRes,//最小分辨率
				TimeDalay,//回调函数
				(DWORD)m_hWnd,//参数
				TIME_PERIODIC);//周期性触发
		}
		GetDlgItem(IDC_TEXT_MODE)->EnableWindow(FALSE);
		m_CoarseAignMode.EnableWindow(FALSE);
		m_FineAignMode.EnableWindow(FALSE);
		m_HeightMode.EnableWindow(FALSE);
		m_NaviMode.EnableWindow(FALSE);
		str = _T("解算停止");
		h1->SetWindowText(str);
		isStartCalOk = true;

	}
	else
	{
		isStartCalOk = false;
		if (isCreateTimer == true)
		{
			timeKillEvent(TimerID);
			isCreateTimer = false;
		}
		sysc.f_coarse_over = 1;
		sysc.f_fine_over = 1;
		sysc.f_navi_over = 1;
		sysc.state = _T("无解算工作");
		str = _T("解算开始");
		h1->SetWindowText(str);
		GetDlgItem(IDC_TEXT_MODE)->EnableWindow(TRUE);
		m_CoarseAignMode.EnableWindow(TRUE);
		m_FineAignMode.EnableWindow(TRUE);
		m_HeightMode.EnableWindow(TRUE);
		m_NaviMode.EnableWindow(TRUE);
		RS_para.reset();
		if (saveStart)
			OnBnClickedBtnSaveall();
	}
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnSavepath()
{
	// TODO: 在此添加控件通知处理程序代码	
	if (fid_zt != NULL) fclose(fid_zt);
	if (fid_Cal != NULL) fclose(fid_Cal);
	if (fid_FOSN != NULL) fclose(fid_FOSN);
	if (fid_gps != NULL) fclose(fid_gps);
	if (fid_PHINS != NULL) fclose(fid_PHINS);

	CString Filename = GetDirectory();
	if (Filename == _T(""))
		return;
	SYSTEMTIME systime;
	GetLocalTime(&systime);
	CString Str1;
	Str1.Format(_T("%ld_%ld"), systime.wHour, systime.wMinute);//不能用冒号 %ld:%ld 由于文件命名的符号问题，冒号及后面的字符都不会有
	if (!is_SingleFile)
	{
		CString DocName;
		if (TestModeNum != 1)
		{
			DocName = Filename + _T("zt") + Str1;
			fopen_s(&fid_zt, DocName + ".txt", "w+");
		}

		DocName = Filename + _T("cal") + Str1;
		fopen_s(&fid_Cal, DocName + ".txt", "w+");

		DocName = Filename + _T("fosn") + Str1;
		fopen_s(&fid_FOSN, DocName + ".txt", "w+");

		DocName = Filename + _T("gps") + Str1;
		fopen_s(&fid_gps, DocName + ".txt", "w+");

		DocName = Filename + _T("phins") + Str1;
		fopen_s(&fid_PHINS, DocName + ".txt", "w+");
	}
	if (is_SingleFile)
	{
		CString DocName;
		DocName = Filename + _T("data") + Str1;
		fopen_s(&fid_Cal, DocName + ".txt", "w+");
		fid_FOSN = NULL;
		fid_PHINS = NULL;
		fid_gps = NULL;
		fid_zt = NULL;
	}
	GetDlgItem(IDC_BTN_SAVEALL)->EnableWindow(TRUE);
	GetDlgItem(IDC_BTN_SingleFile)->EnableWindow(FALSE);
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnSaveall()
{
	// TODO: 在此添加控件通知处理程序代码
	saveStart = !saveStart;
	CWnd *h1;
	h1 = GetDlgItem(IDC_BTN_SAVEALL);		//指向控件的caption
	CString str;
	if (saveStart)
	{
		GetDlgItem(IDC_BTN_SAVEPATH)->EnableWindow(FALSE);
		str = _T("存数停止");
		h1->SetWindowText(str);
	}
	else
	{
		if (fid_zt != NULL) fclose(fid_zt);
		if (fid_Cal != NULL) fclose(fid_Cal);
		if (fid_FOSN != NULL) fclose(fid_FOSN);
		if (fid_gps != NULL) fclose(fid_gps);
		if (fid_PHINS != NULL) fclose(fid_PHINS);
		if (RS_para.RdataFilefid != NULL) fclose(RS_para.RdataFilefid);

		GetDlgItem(IDC_BTN_SAVEPATH)->EnableWindow(TRUE);
		GetDlgItem(IDC_BTN_SingleFile)->EnableWindow(TRUE);
		GetDlgItem(IDC_BTN_SAVEALL)->EnableWindow(FALSE);
		str = _T("存数开始");
		h1->SetWindowText(str);
	}
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnSinglefile()
{
	// TODO: 在此添加控件通知处理程序代码
	is_SingleFile = !is_SingleFile;
	CWnd *h1;
	h1 = GetDlgItem(IDC_BTN_SingleFile);		//指向控件的caption
	CString str;
	if (is_SingleFile)
	{
		str = _T("当前为单文件存数");
		h1->SetWindowText(str);
	}
	else
	{
		str = _T("当前为多文件存数");
		h1->SetWindowText(str);
	}
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnTimeset()
{
	// TODO: 在此添加控件通知处理程序代码
	refresh_time = GetDlgItemInt(IDC_Refresh_Time);
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnReadme()
{
	// TODO: 在此添加控件通知处理程序代码
	CString c;
	c = "!!!修改各种参数前，请先选择刷新暂停!!!\n复位功能不是太好用，建议直接重启防止出现意外";
	MessageBox(c, _T("★★★★★Windows导航平台监控软件说明★★★★★"), MB_OK);

}
void CWindowsMainControlV1Dlg::OnBnClickedBtnSaveHelp()
{
	// TODO: 在此添加控件通知处理程序代码
	CString c;
	/*多文件情况格式：\n\
		ZT数据为：1 帧号，2 ZT接收计数，3FOSN接收计数，3~6转台三个姿态角。\n\
		cal数据格式为：1 FOSN接收计数，2~4姿态，5~7速度，8~10位置，11~13姿态误差，14~16位置误差。\n\
		fosn数据格式为：1 FOSN接收计数，2 FOSN工作时间，3~5陀螺，6~8加表，9~11姿态，12~14速度，15~17位置\n\
		gps数据格式为：1 FOSN接收计数，2 GPS时间，3~5 经纬高\n\
		phins数据格式为：1 PHINS接收计数，2 FOSN接收计数，3 PHINS时间，4~6姿态，7~9速度，10~12位置\n\n\*/
	c = "·录数前选择单文件模式或者多文件模式，多文件模式目前已不在使用\n\
·选择保存路径为文件夹路径，之后自动根据系统时间自动生成录数文件\n\
·请注意该命名最小单位为分钟，同一分钟连续建立文件会覆盖\n\
·在转台和车载模式下，只有数据解算开始才会录数，所以放心先点录数再点解算\n\n\
·陀螺单位为°/s,姿态单位为°\n\
单文件情况格式：\n\
（纯录数模式（不含PHINS）\n\
1~3   录数统计，转台数据号，fosn时间\n\
4~9   陀螺加表数据\n\
10~12 转台姿态\n\
13~15 FOSN姿态\n\n\
（车载模式）\n\
1~5   录数统计，转台/多功能版帧号(备用)，fosn时间,gps时间，phins时间\n\
6~11  陀螺加表\n\
12~14 解算姿态\n\
15~17 解算速度\n\
18~20 解算位置\n\
21~23 PHINS姿态\n\
24~26 PHINS速度\n\
27~29 PHINS位置\n\
30~35 GPS位置，速度（没有）\n\
36~38 FOSN姿态\n\
39~41 FOSN速度\n\
42~44 FOSN位置 \n\
45~47 ZT姿态\n\nby Dr.Yang";
	MessageBox(c, _T("★★★★★Windows导航平台监控软件录数格式说明★★★★★"), MB_OK);
}
void CWindowsMainControlV1Dlg::OnBnClickedCancel()
{
	// TODO: 在此添加控件通知处理程序代码
	mquit = 1;
	closesocket(m_socketPHINSDataRec);
	closesocket(m_socketPC104DataRec);
	WSACleanup();
	if (is_startCal)OnBnClickedBtnStartcal();
	if (fid_zt != NULL) fclose(fid_zt);
	if (fid_Cal != NULL) fclose(fid_Cal);
	if (fid_FOSN != NULL) fclose(fid_FOSN);
	if (fid_gps != NULL) fclose(fid_gps);
	if (fid_PHINS != NULL) fclose(fid_PHINS);
	if (RS_para.RdataFilefid != NULL) fclose(RS_para.RdataFilefid);
	timeEndPeriod(TimerRes);
	CDialogEx::OnCancel();
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnReset()
{
	// TODO: 在此添加控件通知处理程序代码
	mquit = 1;
	//1.关闭文件系统
	if (saveStart)OnBnClickedBtnSaveall();

	//2.导航解算终止
	if (is_startCal) OnBnClickedBtnStartcal();

	//3.数据接收停止
	if (is_cardReceive) OnBnClickedBtnStartcard();
	if (is_start_phins) OnBnClickedBtnStartphins();

	//4.变量初始化	
	init_var();
	phins.reset();
	fosn.reset();
	gps.reset();
	sysc.reset();

}
void CWindowsMainControlV1Dlg::OnBnClickedBtnSavetime()
{
	// TODO: 在此添加控件通知处理程序代码

	CWnd *h1;
	CString str;
	h1 = GetDlgItem(IDC_BTN_SAVETIME);		//指向控件的caption
	save_time = GetDlgItemInt(IDC_SAVETIME);
	m_save_time = save_time;
	is_timeSave = !is_timeSave;
	temp_cnt_s = m_cnt_s;//记录点击时候已经过去的时间
	if (is_timeSave)
	{
		str = _T("关闭定时录数");
		h1->SetWindowText(str);
		GetDlgItem(IDC_SAVETIME)->EnableWindow(FALSE);
	}
	else
	{
		str = _T("启动定时录数");
		h1->SetWindowText(str);
		GetDlgItem(IDC_SAVETIME)->EnableWindow(TRUE);

	}
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnStoprefresh()
{
	// TODO: 在此添加控件通知处理程序代码
	is_UpdateData = !is_UpdateData;
	CWnd *h1;
	h1 = GetDlgItem(IDC_BTN_STOPREFRESH);		//指向控件的caption
	CString str;
	if (is_UpdateData)
	{
		str = _T("刷新暂停");
		h1->SetWindowText(str);

	}
	else
	{
		str = _T("刷新继续");
		h1->SetWindowText(str);

	}
}
void CWindowsMainControlV1Dlg::OnBnClickedBtnCalibration()
{
	// TODO: 在此添加控件通知处理程序代码
	Calibration cali;
	cali.err_input = false;
	INT_PTR nRes;
	if (is_UpdateData)
		OnBnClickedBtnStoprefresh();
	nRes = cali.DoModal();
	if (11 == nRes)//预设值1，车载，吴梅标定于2017年12月
	{
		calipara.Eg_ang[0] = 2.0317e-4;//xy
		calipara.Eg_ang[1] = -4.0032e-6;//yx
		calipara.Eg_ang[2] = -2.04252e-5;//yz
		calipara.Eg_ang[3] = 8.4158e-5;//zy
		calipara.Eg_ang[4] = 7.412e-4;//xz
		calipara.Eg_ang[5] = -5.80456e-4;//zx
		calipara.Cg[0] = 1.000399;
		calipara.Cg[1] = 1.0004312;
		calipara.Cg[2] = 1.00028645;
		calipara.bias_gyro[0] = -7.076e-8;
		calipara.bias_gyro[1] = -7.373e-8;
		calipara.bias_gyro[2] = -2.4168e-9;
		
		calipara.Ea_ang[0] = 1.3127e-04;//xy
		calipara.Ea_ang[1] = 6.3079e-06;//yx
		calipara.Ea_ang[2] = 4.0829e-05;//yz
		calipara.Ea_ang[3] = 9.1177e-05;//zy
		calipara.Ea_ang[4] = 7.3794e-04;//xz
		calipara.Ea_ang[5] = -4.9893e-04;//zx
		calipara.Ca[0] = 1.000194;
		calipara.Ca[1] = 1.00033;
		calipara.Ca[2] = 1.000253;
		calipara.bias_acce[0] = -0.01108;
		calipara.bias_acce[1] = -0.006;
		calipara.bias_acce[2] = -0.006042;
		calipara.Eang2mat();
		if (!is_UpdateData)
			OnBnClickedBtnStoprefresh();
		return;
	}
	if (IDOK == nRes)
	{
		calipara = cali.calipmt;
		amamul(3, 1, calipara.bias_acce, calipara.bias_acce, 9.78*0.000001);	// *ug	
		amamul(3, 1, calipara.bias_gyro, calipara.bias_gyro, D2R / 3600);         // °/h -> rad/s
		calipara.Eang2mat();
		if (!is_UpdateData)
			OnBnClickedBtnStoprefresh();
	}
	else
	{
		if (!is_UpdateData)
			OnBnClickedBtnStoprefresh();
		return;
	}

}
////实验模式改选事件
void CWindowsMainControlV1Dlg::OnCbnSelchangeTextMode()
{
	// TODO: 在此添加控件通知处理程序代码	
	INT_PTR nRes;

	if (m_TestMode.GetCurSel() == 6)
	{
		RS_para.reset();
		ReadSimuDlg ReadSimuDlg1;
		OnBnClickedBtnReset();
		nRes = ReadSimuDlg1.DoModal();
		if (nRes == 6)
		{
			if (RS_para.RdataFilefid != NULL) fclose(RS_para.RdataFilefid);
			RS_para.delay5ms = ReadSimuDlg1.Delay5ms;
			RS_para.ReadInitAtt = ReadSimuDlg1.ReadInitAtt;
			RS_para.ReadInitPos = ReadSimuDlg1.ReadInitPos;
			RS_para.RdataFile = ReadSimuDlg1.filename;
			RS_para.file_mode = ReadSimuDlg1.combonum;
			RS_para.skiptime = ReadSimuDlg1.Skiptime;
			if (RS_para.skiptime == 0) RS_para.skiptime = -1;
			fopen_s(&RS_para.RdataFilefid, RS_para.RdataFile + "", "r");
			RS_para.canCal = 1;
			RS_para.RS_mode = true;
			GetDlgItem(IDC_BTN_SAVEPATH)->EnableWindow(TRUE);
			GetDlgItem(IDC_BTN_StartCal)->EnableWindow(TRUE);
			GetDlgItem(IDC_BTN_StartCard)->EnableWindow(FALSE);
			GetDlgItem(IDC_BTN_StartPhins)->EnableWindow(FALSE);
			refresh_time = 5000;
			UpdateData(false);
			CWinThread* simu_Thread;
			THREADPARAM  *phWndParam = new THREADPARAM;
			phWndParam->hwnd = m_hWnd;
			if (!(simu_Thread = AfxBeginThread(SimulateThread, (LPVOID)phWndParam)))
				return;
		}
		else
		{
			MessageBox(_T("文件读取失败"));
			m_TestMode.SetCurSel(0);
			RS_para.reset();
			if (isCreateTimer == true)
			{
				timeKillEvent(TimerID);
				isCreateTimer = false;
			}
			GetDlgItem(IDC_BTN_SAVEPATH)->EnableWindow(FALSE);
			GetDlgItem(IDC_BTN_StartCal)->EnableWindow(FALSE);
			GetDlgItem(IDC_BTN_StartCard)->EnableWindow(TRUE);
			GetDlgItem(IDC_BTN_StartPhins)->EnableWindow(TRUE);
		}
	}
	else RS_para.reset();

	if (m_TestMode.GetCurSel() == 7)
	{
		SimuDataDlg simuDlg;
		nRes = simuDlg.DoModal();
	}

	if (m_TestMode.GetCurSel() == 8)
	{
		pc104RecQuit = false;
		GetDlgItem(IDC_BTN_SAVEPATH)->EnableWindow(TRUE);	
		GetDlgItem(IDC_BTN_StartCard)->EnableWindow(FALSE);
		GetDlgItem(IDC_BTN_StartPhins)->EnableWindow(FALSE);	
		CWinThread* pc104_Thread;
		SOCKETPARAM  *phWndParam = new SOCKETPARAM;
		phWndParam->hwnd = m_hWnd;
		phWndParam->sock = m_socketPC104DataRec;
		if (!(pc104_Thread = AfxBeginThread(PC104RecThread, (LPVOID)phWndParam)))
			return;
	}
	else
	{
		pc104RecQuit = true;
		GetDlgItem(IDC_BTN_SAVEPATH)->EnableWindow(FALSE);
		GetDlgItem(IDC_BTN_StartCard)->EnableWindow(TRUE);
		GetDlgItem(IDC_BTN_StartPhins)->EnableWindow(TRUE);
	}
}
//精对准下才启用水平和航向对准时间
void CWindowsMainControlV1Dlg::OnCbnSelchangeComboFinealignmode()
{
	// TODO: 在此添加控件通知处理程序代码
	FineModeNum = m_FineAignMode.GetCurSel();
	if (FineModeNum == 1)
	{
		GetDlgItem(IDC_FINE_LEVEL)->EnableWindow(TRUE);
		GetDlgItem(IDC_FINE_AZIMUTH)->EnableWindow(TRUE);
	}
	else
	{
		GetDlgItem(IDC_FINE_LEVEL)->EnableWindow(FALSE);
		GetDlgItem(IDC_FINE_AZIMUTH)->EnableWindow(FALSE);
	}
}
#pragma endregion ClBtn 

/////↓↓↓↓***********初始化函数****************↓↓↓↓//////
#pragma region InitFunc
//haishi模式还是常规模式的一些值预设，方便连续实验不用不停切换选项卡
void CWindowsMainControlV1Dlg::init_mainmode()
{
	if (MAINMODE == 1)
	{
		m_NaviMode.SetCurSel(0);
		m_HeightMode.SetCurSel(0);
		edit_data_f = 200;
		UpdateData(false);
	}
	if (MAINMODE == 2)
	{
		m_NaviMode.SetCurSel(5);
		m_HeightMode.SetCurSel(4);
		edit_data_f = 100;
		UpdateData(false);
	}
}
////变量重置
void CWindowsMainControlV1Dlg::init_var()
{
	fid_FOSN = NULL;
	fid_PHINS = NULL;
	fid_gps = NULL;
	fid_zt = NULL;
	fid_Cal = NULL;

	is_cardReceive = 0;
	is_startCal = 0;
	//is_InitNaviVar=0;
	is_start_phins = 0;
	is_data_used = false;

	isStartCalOk = false;

	memset(bufFOSN, 0, sizeof(bufFOSN));
	
	m_cnt_err = 0;

	sysc.state = _T("还没开始Zz");
	m_cnt_s = 0;
	saveStart = 0;

	is_timeSave = 0;
	m_save_time = 600;

	m_PRecNum = 0;
	mquit = 0;
	if (MAINMODE == 2)
	{
		calipara.bias_gyro[0] = -3.94251980810677e-05;
		calipara.bias_gyro[1] = -4.75641514044445e-06;
		calipara.bias_gyro[2] = 1.35189552763437e-05;
	}
}
////combo初始化
bool CWindowsMainControlV1Dlg::init_Combo()
{
	int judge_tf;
	int i;

	CString str2[] = { _T("无"),_T("解析法/没写"),_T("惯性系法/没写"),_T("凝固解析法"),_T("四元数法/没写"),_T("自定义方法1"),_T("自定义方法2") };
	for (i = 0; i < 7; i++)
	{
		judge_tf = m_CoarseAignMode.InsertString(i, str2[i]);
		if ((judge_tf == CB_ERR) || (judge_tf == CB_ERRSPACE))
		{
			MessageBox(_T("build baud error!"));
			return false;
		}
	}
	m_CoarseAignMode.SetCurSel(0);

	CString str3[] = { _T("无"),_T("罗经法"),_T("零速校正"),_T("yucia完美双位置"),_T("自定义方法"),_T("自抗扰对准"),_T("自定义方法") };
	for (i = 0; i < 7; i++)
	{
		judge_tf = m_FineAignMode.InsertString(i, str3[i]);
		if ((judge_tf == CB_ERR) || (judge_tf == CB_ERRSPACE))
		{
			MessageBox(_T("build baud error!"));
			return false;
		}
	}
	m_FineAignMode.SetCurSel(0);

	CString str4[] = { _T("仅纯惯性"),_T("速度+航向"),_T("速度+位置+姿态(没写)"),_T("GPS位置组合"),_T("GPS速度组合(没写)"),_T("haishi组合"),_T("haishi降噪"),_T("PS位置组合"),_T("PS速度组合n"),_T("PS速度组合b"),_T("b系速度 + 航向(没写)") };
	for (i = 0; i < 11; i++)
	{
		judge_tf = m_NaviMode.InsertString(i, str4[i]);
		if ((judge_tf == CB_ERR) || (judge_tf == CB_ERRSPACE))
		{
			MessageBox(_T("build baud error!"));
			return false;
		}
	}
	m_NaviMode.SetCurSel(5);

	CString str5[] = { _T("无阻尼"),_T("有阻尼（暂无）"),_T("旋转矢量法"),_T("横向导航"),_T("haishi炮位") ,_T("haishi雷达位") ,_T("haishi无杆臂") };
	for (i = 0; i < 7; i++)
	{
		judge_tf = m_HeightMode.InsertString(i, str5[i]);
		if ((judge_tf == CB_ERR) || (judge_tf == CB_ERRSPACE))
		{
			MessageBox(_T("build baud error!"));
			return false;
		}
	}
	m_HeightMode.SetCurSel(3);

	CString str6[] = { _T("转台实验"),_T("车载实验"),_T("纯录数（不含phins）"),_T("纯录数（含phins）"),_T("备用1"),_T("备用2"),_T("读数仿真"),_T("模拟数据仿真"),_T("PC104数据接收") };
	for (i = 0; i < 9; i++)
	{
		judge_tf = m_TestMode.InsertString(i, str6[i]);
		if ((judge_tf == CB_ERR) || (judge_tf == CB_ERRSPACE))
		{
			MessageBox(_T("build baud error!"));
			return false;
		}
	}
	m_TestMode.SetCurSel(0);
	return true;
}
////网络初始化
bool CWindowsMainControlV1Dlg::Init_Net(void)
{
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;
	wVersionRequested = MAKEWORD(2, 2);
	err = WSAStartup(wVersionRequested, &wsaData);
	if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2)
	{
		WSACleanup();
		return false;
	}
	m_socketPHINSDataRec = socket(AF_INET, SOCK_DGRAM, 0);
	if (INVALID_SOCKET == m_socketPHINSDataRec)
	{
		MessageBox(_T(" 数据接收socket创建失败"));
		return false;
	}
	SOCKADDR_IN addrRecSock;
	addrRecSock.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	addrRecSock.sin_family = AF_INET;
	addrRecSock.sin_port = htons(8112);
	int retval = bind(m_socketPHINSDataRec, (SOCKADDR*)& addrRecSock, sizeof(SOCKADDR));
	if (SOCKET_ERROR == retval)
	{
		closesocket(m_socketPHINSDataRec);
		return false;
	}

	m_socketPC104DataRec = socket(AF_INET, SOCK_DGRAM, 0);
	if (INVALID_SOCKET == m_socketPC104DataRec)
	{
		MessageBox(_T(" 数据接收socket创建失败"));
		return false;
	}
	
	addrRecSock.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	addrRecSock.sin_family = AF_INET;
	addrRecSock.sin_port = htons(6005);
	retval = bind(m_socketPC104DataRec, (SOCKADDR*)& addrRecSock, sizeof(SOCKADDR));
	if (SOCKET_ERROR == retval)
	{
		closesocket(m_socketPC104DataRec);
		return false;
	}

	return true;
}
#pragma endregion InitFunc
#pragma region SmallFuc
////文件夹路径获取
CString CWindowsMainControlV1Dlg::GetDirectory()////返回文件夹路径
{
	BROWSEINFO bi;
	char name[MAX_PATH];
	ZeroMemory(&bi, sizeof(BROWSEINFO));
	bi.hwndOwner = AfxGetMainWnd()->GetSafeHwnd();
	bi.pszDisplayName = LPWSTR(name);
	bi.lpszTitle = _T("选择保存的文件夹");
	bi.ulFlags = BIF_RETURNFSANCESTORS;
	LPITEMIDLIST idl = SHBrowseForFolder(&bi);
	if (idl == NULL)
		return _T("");
	CString strDirectoryPath;
	SHGetPathFromIDList(idl, strDirectoryPath.GetBuffer(MAX_PATH));
	strDirectoryPath.ReleaseBuffer();
	if (strDirectoryPath.IsEmpty())
		return _T("");
	if (strDirectoryPath.Right(1) != "\\")
		strDirectoryPath += "\\";
	return strDirectoryPath;
}
////数据刷新消息响应函数
LRESULT CWindowsMainControlV1Dlg::updateData(WPARAM wParam, LPARAM lParam)////显示刷新响应函数
{
	UpdateData(false);
	return 0L;
}
////存数停止消息响应行数
LRESULT CWindowsMainControlV1Dlg::saveCancel(WPARAM wParam, LPARAM lParam)
{
	OnBnClickedCancel();
	return 0L;
}
//200数据计数函数(sysc.Fs决定，正常是200)
bool CWindowsMainControlV1Dlg::Rec200times(int &count200)
{
	if (count200 == sysc.Fs)
	{
		count200 = 1;
		return 1;
	}
	else
	{
		count200++;
		return 0;
	}
}
#pragma endregion SmallFuc

///////////板卡数据接收处理函数/////////////////
#pragma region DataPro 
int CWindowsMainControlV1Dlg::FOSNChannel()
{
	WORD rt = 0;
	int i;

	if (strm[2].IsProtocol)
	{
		//	Protocol Mode		
		if (Sio_Rx_IsFrameOver(hCard, 2))
		{
			Sio_ReadFrame(hCard, 2, bufFOSN, &rt);
			fosn.Conv(bufFOSN);
			for (i = 0; i < 3; i++)
			{
				IMUout.acce_b[i] = fosn.fb[i];
				IMUout.gyro_b[i] = fosn.wb[i];
			}

			if (is_startCal)fosn.recnum++;
			return 1;
		}
		else
			return 0;
	}
	return 0;
}
void CWindowsMainControlV1Dlg::ZTChannel()
{
	WORD rt = 0;
	static int old_buf2 = 0;
	if (strm[0].IsProtocol)
	{
		//	Protocol Mode
#ifndef CARD_DEBUG
		if (Sio_Rx_IsFrameOver(hCard, 0))
		{
			old_buf2 = BufZT[2];
			Sio_ReadFrame(hCard, 0, BufZT, &rt);

			if (BufZT[0] == 0xFF && BufZT[1] == 0xFF)
			{
				CString c;
				int temp[8];
				int i;
				float angFybt[3] = { 0.0 };
				short int jiaoyan = 0;
				int tem = 0;
				static int DrawNum = 0;
				for (i = 0; i < 3; i++)
				{
					temp[0] = BufZT[i * 4 + 3] / 16;
					temp[1] = BufZT[i * 4 + 3] % 16;
					temp[2] = BufZT[i * 4 + 1 + 3] / 16;
					temp[3] = BufZT[i * 4 + 1 + 3] % 16;
					temp[4] = BufZT[i * 4 + 2 + 3] / 16;
					temp[5] = BufZT[i * 4 + 2 + 3] % 16;
					temp[6] = BufZT[i * 4 + 3 + 3] / 16;
					temp[7] = BufZT[i * 4 + 3 + 3] % 16;
					angFybt[i] = temp[0] * 100 + temp[1] * 10 + temp[2] + temp[3] * 0.1 + temp[5] * 0.01 + temp[6] * 0.001 + temp[7] * 0.0001;
				}
				if (BufZT[2] - old_buf2 < 0) ZT.Frame += BufZT[2] - old_buf2 + 256;
				else ZT.Frame += BufZT[2] - old_buf2;

				if (is_startCal)
				{
					ZT.cnt++;
					m_cnt_err = ZT.cnt - fosn.recnum;
				}

				for (i = 0; i < 3; i++)
				{
					if (angFybt[i] > 180)
						angFybt[i] = angFybt[i] - 360.0;
				}
				ZT.ang[0] = angFybt[1];   //纵摇角
				ZT.ang[1] = angFybt[0];   //横摇角
				ZT.ang[2] = angFybt[2];   //航向角

			}
		}
#endif
	}
}
int CWindowsMainControlV1Dlg::GPSChannel()
{
	WORD rt = 0;	
	//	Protocol Mode
	if (Sio_Rx_IsFrameOver(hCard, 3))
	{
		Sio_ReadFrame(hCard, 3, BufGPS, &rt);
		gps.flag = gps.Conv(BufGPS);
		if (is_startCal) gps.cnt++;
		if (gps.flag) return 1;
		else return 0;
	}
	else
		return 0;
}
void CWindowsMainControlV1Dlg::CalVarInit(char mode)
{
	if (mode == 1)
	{
		initial_latitude = phins.pos[0];
		initial_longitude = phins.pos[1];
		initial_height = phins.pos[2];
	}	
	infor.pos[0] = initial_latitude* D2R;
	infor.pos[1] = initial_longitude* D2R;
	infor.pos[2] = initial_height;
	memcpy(real_pos, infor.pos, sizeof(infor.pos));
	infor.initial_pos[0] = initial_latitude* D2R;
	infor.initial_pos[1] = initial_longitude* D2R;
	infor.initial_pos[2] = initial_height;

}
void CWindowsMainControlV1Dlg::getfileData()
{
	double temp;
	if (feof(RS_para.RdataFilefid))//读到结尾
	{
		RS_para.RS_mode = 0;
		return;
	}
	if (RS_para.file_mode == 1)//对应纯录数模式（不含PHINS）
	{
		fscanf_s(RS_para.RdataFilefid, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			&m_PRecNum, &ZT.Frame, &fosn.time,
			&IMUout.gyro_b[0], &IMUout.gyro_b[1], &IMUout.gyro_b[2],
			&IMUout.acce_b[0], &IMUout.acce_b[1], &IMUout.acce_b[2],
			&ZT.ang[0], &ZT.ang[1], &ZT.ang[2],
			&fosn.ang[0], &fosn.ang[1], &fosn.ang[2]);
	}
	if (RS_para.file_mode == 0)//对应车载录数模式//2017.12.12改成47位，首日的录数已经不适用此
	{
		fscanf_s(RS_para.RdataFilefid, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			&fosn.recnum, &ZT.cnt, &fosn.time, &gps.time, &phins.utc,
			&IMUout.gyro_b[0], &IMUout.gyro_b[1], &IMUout.gyro_b[2],
			&IMUout.acce_b[0], &IMUout.acce_b[1], &IMUout.acce_b[2],
			&temp, &temp, &temp,
			&temp, &temp, &temp,
			&temp, &temp, &temp,
			&phins.ang[0], &phins.ang[1], &phins.ang[2],
			&phins.vel[0], &phins.vel[1], &phins.vel[2],
			&phins.pos[0], &phins.pos[1], &phins.pos[2],
			&gps.pos[0], &gps.pos[1], &gps.pos[2],
			&gps.vel[0], &gps.vel[1], &gps.vel[2],
			&fosn.ang[0], &fosn.ang[1], &fosn.ang[2],
			&fosn.vel[0], &fosn.vel[1], &fosn.vel[2],
			&fosn.pos[0], &fosn.pos[1], &fosn.pos[2],
			&temp, &temp, &temp);
		memcpy(real_pos, phins.pos, sizeof(phins.pos));
		memcpy(ZT.ang, phins.ang, sizeof(phins.ang));
		real_pos[0] = real_pos[0] * D2R;
		real_pos[1] = real_pos[1] * D2R;

	}
	if (RS_para.file_mode == 2)//对应转台实验模式（不含kalman估计的）
	{
		fscanf_s(RS_para.RdataFilefid, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			&fosn.recnum, &ZT.cnt,
			&IMUout.gyro_b[0], &IMUout.gyro_b[1], &IMUout.gyro_b[2],
			&IMUout.acce_b[0], &IMUout.acce_b[1], &IMUout.acce_b[2],
			&INScal.ang[0], &INScal.ang[1], &INScal.ang[2],
			&INScal.vel[0], &INScal.vel[1], &INScal.vel[2],
			&INScal.pos[0], &INScal.pos[1], &INScal.pos[2],
			&ZT.ang[0], &ZT.ang[1], &ZT.ang[2],
			&fosn.ang[0], &fosn.ang[1], &fosn.ang[2]);

	}
	if (RS_para.file_mode == 3)//对应仿真数据模式1
	{
	                	//             |           |           |           |           |           |       |
		fscanf_s(RS_para.RdataFilefid, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			&fosn.time,
			&IMUout.gyro_b[0], &IMUout.gyro_b[1], &IMUout.gyro_b[2],
			&IMUout.acce_b[0], &IMUout.acce_b[1], &IMUout.acce_b[2],
			&ZT.ang[0], &ZT.ang[1], &ZT.ang[2],
			&fosn.vel[0], &fosn.vel[1], &fosn.vel[2],
			&fosn.pos[0], &fosn.pos[1], &fosn.pos[2],
			&temp, &temp,
			&temp, &temp, &temp,
			&temp, &temp, &temp);
		IMUout.gyro_b[0] = IMUout.gyro_b[0]*R2D;
		IMUout.gyro_b[1] = IMUout.gyro_b[1]*R2D;
		IMUout.gyro_b[2] = IMUout.gyro_b[2]*R2D;
		ZT.ang[0] = ZT.ang[0]*R2D;
		ZT.ang[1] = ZT.ang[1]*R2D;
		ZT.ang[2] = ZT.ang[2]*R2D;		
		fosn.pos[0] = fosn.pos[0]*R2D;
		fosn.pos[1] = fosn.pos[1]*R2D;
		memcpy(phins.pos, fosn.pos, sizeof(fosn.pos));
		memcpy(real_pos, phins.pos, sizeof(phins.pos));
		real_pos[0] = real_pos[0] * D2R;
		real_pos[1] = real_pos[1] * D2R;

	}
	if (RS_para.file_mode == 4)//对应仿真数据模式2
	{
		//               |           |           |           |           |           |       |
		fscanf_s(RS_para.RdataFilefid, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			&fosn.time,
			&temp, &temp, &temp,
			&temp, &temp, &temp,
			&ZT.ang[0], &ZT.ang[1], &ZT.ang[2],
			&fosn.vel[0], &fosn.vel[1], &fosn.vel[2],
			&fosn.pos[0], &fosn.pos[1], &fosn.pos[2],
			&temp, &temp,
			&IMUout.gyro_b[0], &IMUout.gyro_b[1], &IMUout.gyro_b[2],
			&IMUout.acce_b[0], &IMUout.acce_b[1], &IMUout.acce_b[2]);
		IMUout.gyro_b[0] = IMUout.gyro_b[0]*R2D;
		IMUout.gyro_b[1] = IMUout.gyro_b[1]*R2D;
		IMUout.gyro_b[2] = IMUout.gyro_b[2]*R2D;
		ZT.ang[0] = ZT.ang[0]*R2D;
		ZT.ang[1] = ZT.ang[1]*R2D;
		ZT.ang[2] = ZT.ang[2]*R2D;
		fosn.pos[0] = fosn.pos[0]*R2D;
		fosn.pos[1] = fosn.pos[1]*R2D;
		memcpy(phins.pos, fosn.pos, sizeof(fosn.pos));
		memcpy(real_pos, phins.pos, sizeof(phins.pos));
		real_pos[0] = real_pos[0] * D2R;
		real_pos[1] = real_pos[1] * D2R;
	}
	if (RS_para.file_mode == 5)//yucia数据测试
	{
		fscanf_s(RS_para.RdataFilefid, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
			&fosn.time,
			&IMUout.gyro_b[0], &IMUout.gyro_b[1], &IMUout.gyro_b[2],  //单位 °
			&IMUout.acce_b[0], &IMUout.acce_b[1], &IMUout.acce_b[2],
			&phins.ang[0], &phins.ang[1], &phins.ang[2],
			&phins.vel[0], &phins.vel[1], &phins.vel[2],
			&phins.pos[0], &phins.pos[1], &phins.pos[2]);
		memcpy(real_pos, phins.pos, sizeof(phins.pos));
		memcpy(ZT.ang, phins.ang, sizeof(ZT.ang));
		real_pos[0] = real_pos[0] * D2R;
		real_pos[1] = real_pos[1] * D2R;
	}
	if (RS_para.file_mode == 7)//haishi数据测试
	{
	//	1~3陀螺；4~6加表；7~9 原始计算姿态；10~12 原始计算速度； 13~15 外部参考姿态；16~18外部参考速度；19纬度 20经度,21高，22计数
		                    // 22列                |           |           |           |           |           |           |
		fscanf_s(RS_para.RdataFilefid, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",
			&IMUout.gyro_b[0], &IMUout.gyro_b[1], &IMUout.gyro_b[2],
			&IMUout.acce_b[0], &IMUout.acce_b[1], &IMUout.acce_b[2],
			&fosn.ang[0], &fosn.ang[1], &fosn.ang[2],
			&fosn.vel[0], &fosn.vel[1], &fosn.vel[2],
			&phins.ang[0], &phins.ang[1], &phins.ang[2],
			&phins.vel[0], &phins.vel[1], &phins.vel[2],
			&phins.pos[0], &phins.pos[1], &phins.pos[2],
			&phins.cnt);
		memcpy(ZT.ang, phins.ang, sizeof(phins.ang));
		memcpy(fosn.pos, phins.pos, sizeof(phins.pos));
		memcpy(real_pos, phins.pos, sizeof(phins.pos));
		real_pos[0] = real_pos[0] * D2R;
		real_pos[1] = real_pos[1] * D2R;
	}
	fosn.s = (int)fosn.time;
	fosn.ms = (fosn.time - fosn.s) * 1000;
	//首次读取判断是否需要进行初始化数据的姿态和位置到数据第一帧读取的姿态和位置
	if (RS_para.isstart == 0)
	{
		if (RS_para.ReadInitPos == 1)//读取初始位置
		{
			if (RS_para.file_mode == 0)
			{
				infor.pos[0] = phins.pos[0] * D2R;
				infor.pos[1] = phins.pos[1] * D2R;	
				infor.pos[2] = phins.pos[2];
				initial_latitude = infor.pos[0] * R2D;
				initial_longitude = infor.pos[1] * R2D;
				initial_height = infor.pos[2];
				infor.vel_n[0] = phins.vel[0];       //20171129
				infor.vel_n[1] = phins.vel[1];
				infor.vel_n[2] = phins.vel[2];
			}
			if (RS_para.file_mode == 2)
			{
				memcpy(infor.initial_pos, INScal.pos, sizeof(INScal.pos));
				memcpy(infor.pos, infor.initial_pos, sizeof(infor.initial_pos));
				initial_latitude = infor.pos[0];
				initial_longitude = infor.pos[1];
				initial_height = infor.pos[2];
			}
			if (RS_para.file_mode == 3 || RS_para.file_mode == 4 )
			{
				initial_latitude = fosn.pos[0];
				initial_longitude = fosn.pos[1];
				initial_height = fosn.pos[2];
				infor.pos[0] = initial_latitude* D2R;
				infor.pos[1] = initial_longitude* D2R;
				infor.pos[2] = initial_height;
				infor.initial_pos[0] = initial_latitude* D2R;
				infor.initial_pos[1] = initial_longitude* D2R;
				infor.initial_pos[2] = initial_height;
				
			}
			if (RS_para.file_mode == 5)
			{
				infor.pos[0] = phins.pos[0] * D2R;
				infor.pos[1] = phins.pos[1] * D2R;
				infor.pos[2] = phins.pos[2];
				initial_latitude = infor.pos[0] * R2D;
				initial_longitude = infor.pos[1] * R2D;
				initial_height = infor.pos[2];
				infor.vel_n[0] = phins.vel[0];       //20171129
				infor.vel_n[1] = phins.vel[1];
				infor.vel_n[2] = phins.vel[2];
			}
			if (RS_para.file_mode == 7)
			{				
				infor.pos[0] = phins.pos[0] * D2R;
				infor.pos[1] = phins.pos[1] * D2R;
				infor.pos[2] = phins.pos[2] * D2R;
				initial_latitude = phins.pos[0];
				initial_longitude = phins.pos[1];
				initial_height = phins.pos[2];
				memcpy(infor.vel_n, phins.vel, sizeof(phins.vel));

				infor.att_angle[0] = fosn.ang[0] * D2R;
				infor.att_angle[1] = fosn.ang[1] * D2R;
				infor.att_angle[2] = fosn.ang[2] * D2R;
				ang2cnb(infor.cnb_mat, infor.att_angle);
				cnb2q(infor.cnb_mat, infor.quart);
				optq(infor.quart);
			}
			RS_para.isstart = 1;
		}
		if (RS_para.ReadInitAtt == 1&& RS_para.file_mode<7)//读取初始姿态
		{
			infor.att_angle[0] = ZT.ang[0] *D2R;
			infor.att_angle[1] = ZT.ang[1] *D2R;
			infor.att_angle[2] = ZT.ang[2] *D2R;
			ang2cnb(infor.cnb_mat, infor.att_angle);
			cnb2q(infor.cnb_mat, infor.quart);
			optq(infor.quart);
		}
		RS_para.isstart = 1;
	}
}
void CWindowsMainControlV1Dlg::IMUdataCount()
{
	sysc.data_cnt++;
	if (0 == sysc.data_cnt % sysc.Fs)
	{
		sysc.cnt_s++;
		if (is_timeSave)
			m_save_time--;//倒计时显示
	}
	m_cnt_s = sysc.cnt_s;	
	is_data_used = 0;
}
void CWindowsMainControlV1Dlg::SaveData()
{
	if (saveStart && !is_SingleFile)
	{
		if (TestModeNum != 1)
		{
			fprintf_s(fid_zt, "%d,%d,%d,%lf,%lf,%lf\n",
				ZT.Frame, ZT.cnt, fosn.recnum,
				ZT.ang[0], ZT.ang[1], ZT.ang[2]);
			fflush(fid_zt);
		}

		fprintf_s(fid_Cal, "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			fosn.recnum,
			INScal.ang[0], INScal.ang[1], INScal.ang[2],
			INScal.vel[0], INScal.vel[1], INScal.vel[2],
			INScal.pos[0], INScal.pos[1], INScal.pos[2],
			INScal.err_ang[0], INScal.err_ang[1], INScal.err_ang[2],
			INScal.err_pos[0], INScal.err_pos[1], INScal.err_pos[2]);
		fflush(fid_Cal);

		fprintf_s(fid_FOSN, "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			fosn.recnum, fosn.time,
			IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
			IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
			fosn.ang[0], fosn.ang[1], fosn.ang[2],
			fosn.vel[0], fosn.vel[1], fosn.vel[2],
			fosn.pos[0], fosn.pos[1], fosn.pos[2]);
		fflush(fid_FOSN);

		fprintf_s(fid_gps, "%d,%lf,%lf,%lf,%lf\n",
			fosn.recnum, gps.time,
			gps.pos[0], gps.pos[1], gps.pos[2]);
		fflush(fid_gps);

		fprintf_s(fid_PHINS, "%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			phins.cnt, fosn.recnum, phins.utc,
			phins.ang[0], phins.ang[1], phins.ang[2],
			phins.vel[0], phins.vel[1], phins.vel[2],
			phins.pos[2], phins.pos[0], phins.pos[1]);
		fflush(fid_PHINS);

	}
	if (saveStart&&is_SingleFile)
	{
		if (TestModeNum == 0)//转台实验
		{	
			if (FineModeNum == FINE_Yucia)
				fprintf_s(fid_Cal, "%d,%d,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
					fosn.recnum, ZT.cnt,
					IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
					IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
					INScal.ang[0], INScal.ang[1], INScal.ang[2],
					INScal.vel[0], INScal.vel[1], INScal.vel[2],
					INScal.pos[0], INScal.pos[1], INScal.pos[2],
					ZT.ang[0], ZT.ang[1], ZT.ang[2],
					fosn.ang[0], fosn.ang[1], fosn.ang[2],
					fkalman.X_vector[9], fkalman.X_vector[10], fkalman.X_vector[11],
					fkalman.X_vector[12], fkalman.X_vector[13], fkalman.X_vector[14]);
			else
				fprintf_s(fid_Cal, "%d,%d,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
					fosn.recnum, ZT.cnt,
					IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
					IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
					INScal.ang[0], INScal.ang[1], INScal.ang[2],
					INScal.vel[0], INScal.vel[1], INScal.vel[2],
					INScal.pos[0], INScal.pos[1], INScal.pos[2],
					ZT.ang[0], ZT.ang[1], ZT.ang[2],
					fosn.ang[0], fosn.ang[1], fosn.ang[2]);
		}
#pragma region Datacz
			if (TestModeNum == 1|| TestModeNum == 8)//车载实验数据标准格式
				/*1~5 帧号录数统计，转台帧号/多功能版帧号/GPS有效性，fosn时间,gps时间，phins时间
				6~11  陀螺加表（陀螺单位为 度/S）
				12~14 解算姿态
				15~17 解算速度
				18~20 解算位置
				21~23 PHINS姿态
				24~26 PHINS速度
				27~29 PHINS位置
				30~35 GPS位置，速度（没有）
				36~38 FOSN姿态
				39~41 FOSN速度
				42~44 FOSN位置
				45~47 空（转台姿态）
				|陀螺                 加表                |解算姿态            速度                  位置                |PS姿态               速度                位置                 |GPS位置              速度                |FOSN姿态            速度                 位置    */
			{
				if (PureINSModeNum == PURE_SINS_TRANSVERSE)
				{
					fprintf_s(fid_Cal, "%d,%d,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
						fosn.recnum, ZT.cnt, fosn.time, gps.time, phins.utc,
						IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
						IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
						INScal.ang[0], INScal.ang[1], INScal.ang[2],
						INScal.vel[0], INScal.vel[1], INScal.vel[2],
						INScal.pos[0], INScal.pos[1], INScal.pos[2],
						phins.ang[0], phins.ang[1], phins.ang[2],
						phins.vel[0], phins.vel[1], phins.vel[2],
						phins.pos[0], phins.pos[0], phins.pos[2],
						gps.pos[0], gps.pos[1], gps.pos[2],
						0.0, 0.0, 0.0,
						fosn.ang[0], fosn.ang[1], fosn.ang[2],
						fosn.vel[0], fosn.vel[1], fosn.vel[2],
						fosn.pos[0], fosn.pos[1], fosn.pos[2],
						inforS.att_angle_S[0], inforS.att_angle_S[1], inforS.att_angle_S[2],
						inforS.vel_S[0], inforS.vel_S[1], inforS.vel_S[2],
						inforS.lati_S, inforS.longi_S, inforS.high_S, 
						inforS.att_angle[0], inforS.att_angle[1], inforS.att_angle[2],
						inforS.vel_n[0], inforS.vel_n[1], inforS.vel_n[2],
						inforS.lati, inforS.longi, inforS.high);
				}
				else
				{
					fprintf_s(fid_Cal, "%d,%d,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%lf,%lf,%lf\n",
						fosn.recnum, gps.flag, fosn.time, gps.time, phins.utc,
						IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
						IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
						INScal.ang[0], INScal.ang[1], INScal.ang[2],
						INScal.vel[0], INScal.vel[1], INScal.vel[2],
						INScal.pos[0], INScal.pos[1], INScal.pos[2],
						phins.ang[0], phins.ang[1], phins.ang[2],
						phins.vel[0], phins.vel[1], phins.vel[2],
						phins.pos[0], phins.pos[1], phins.pos[2],
						gps.pos[0], gps.pos[1], gps.pos[2],
						gps.vel[0], gps.vel[1], gps.vel[2],
						fosn.ang[0], fosn.ang[1], fosn.ang[2],
						fosn.vel[0], fosn.vel[1], fosn.vel[2],
						fosn.pos[0], fosn.pos[1], fosn.pos[2],						 
						0.0, 0.0, 0.0);
				}
			}

#pragma endregion Datacz
		if (TestModeNum == 2)//3号，3陀螺，3加表，3转台姿态，3FOSN姿态
			fprintf_s(fid_Cal, "%d,%d,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
				m_PRecNum, ZT.Frame, fosn.time,
				IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
				IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
				ZT.ang[0], ZT.ang[1], ZT.ang[2],
				fosn.ang[0], fosn.ang[1], fosn.ang[2]);
		if (TestModeNum == 3)//4号，3陀螺，3加表，3转台姿态，3FOSN姿态，3Phins姿态速度位置
			fprintf_s(fid_Cal, "%d,%d,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
				m_PRecNum, ZT.Frame, fosn.time, phins.utc,
				IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
				IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
				ZT.ang[0], ZT.ang[1], ZT.ang[2],
				fosn.ang[0], fosn.ang[1], fosn.ang[2],
				phins.ang[0], phins.ang[1], phins.ang[2],
				phins.vel[0], phins.vel[1], phins.vel[2],
				phins.pos[2], phins.pos[0], phins.pos[1]);
		if (TestModeNum == 6)
		{
			if (NaviModeNum == NAVI_PHINS_VEL2)
			{
				fprintf_s(fid_Cal, "%lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
					fosn.time,
					IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
					IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
					INScal.ang[0], INScal.ang[1], INScal.ang[2],
					INScal.vel[0], INScal.vel[1], INScal.vel[2],
					INScal.pos[0], INScal.pos[1], INScal.pos[2],
					kalman_dvl.X_vector[9], kalman_dvl.X_vector[10], kalman_dvl.X_vector[11],
					kalman_dvl.X_vector[12], kalman_dvl.X_vector[13], kalman_dvl.X_vector[14], kalman_dvl.X_vector[15]);
				fflush(fid_Cal);
				return;
			}
			if (FineModeNum == FINE_Yucia)
			{			
				fprintf_s(fid_Cal, "%lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
					fosn.time,
					IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
					IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
					INScal.ang[0], INScal.ang[1], INScal.ang[2],
					INScal.vel[0], INScal.vel[1], INScal.vel[2],
					INScal.pos[0], INScal.pos[1], INScal.pos[2],
					ZT.ang[0], ZT.ang[1], ZT.ang[2],
					fkalman.X_vector[9], fkalman.X_vector[10], fkalman.X_vector[11],
					fkalman.X_vector[12], fkalman.X_vector[13], fkalman.X_vector[14]);
				fflush(fid_Cal);
				return;
			}
			if (PureINSModeNum == PURE_SINS_TRANSVERSE)
			{
				fprintf_s(fid_Cal, "%lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
					fosn.time,
					IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
					IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
					inforS.att_angle[0], inforS.att_angle[1], inforS.att_angle[2],
					inforS.vel_n[0], inforS.vel_n[1], inforS.vel_n[2],
					inforS.lati, inforS.longi, inforS.high,
					inforS.att_angle_S[0], inforS.att_angle_S[1], inforS.att_angle_S[2],
					inforS.vel_S[0], inforS.vel_S[1], inforS.vel_S[2],
					inforS.lati_S, inforS.longi_S, inforS.high_S);
				fflush(fid_Cal);
				return;
			}				
			if (MAINMODE==2)
			{          //                               |                    |                    |                    |                    |                    |                    |                    |
				fprintf_s(fid_Cal, "%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf\n",
					INScal.ang[0], INScal.ang[1], INScal.ang[2],
					INScal.vel[0], INScal.vel[1], INScal.vel[2],
					INScal.pos[0], INScal.pos[1], INScal.pos[2],
					fosn.ang[0], fosn.ang[1], fosn.ang[2],
					fosn.vel[0], fosn.vel[1], fosn.vel[2],					
					phins.ang[0], phins.ang[1], phins.ang[2],
					phins.vel[0], phins.vel[1], phins.vel[2],
					phins.pos[0], phins.pos[1], phins.pos[2], 
					IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2]);
				fflush(fid_Cal);
				return;
			}
			fprintf_s(fid_Cal, "%d,%d,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%.16lf,%lf,%lf,%lf\n",
				fosn.recnum, ZT.cnt, fosn.time, gps.time, phins.utc,
				IMUout.gyro_b[0], IMUout.gyro_b[1], IMUout.gyro_b[2],
				IMUout.acce_b[0], IMUout.acce_b[1], IMUout.acce_b[2],
				INScal.ang[0], INScal.ang[1], INScal.ang[2],
				INScal.vel[0], INScal.vel[1], INScal.vel[2],
				INScal.pos[0], INScal.pos[1], INScal.pos[2],
				phins.ang[0], phins.ang[1], phins.ang[2],
				phins.vel[0], phins.vel[1], phins.vel[2],
				phins.pos[0], phins.pos[1], phins.pos[2],
				gps.pos[0], gps.pos[1], gps.pos[2],
				0.0, 0.0, 0.0,
				fosn.ang[0], fosn.ang[1], fosn.ang[2],
				fosn.vel[0], fosn.vel[1], fosn.vel[2],
				fosn.pos[0], fosn.pos[1], fosn.pos[2],
				0.0, 0.0, 0.0);
		}
		fflush(fid_Cal);
	}
}
void CWindowsMainControlV1Dlg::DataTrans()
{
	avecmul(3, INScal.ang, infor.att_angle, R2D);	
	memcpy(INScal.vel, infor.vel_n, sizeof(infor.vel_n));	
	INScal.pos[0] = infor.pos[0]*R2D;
	INScal.pos[1] = infor.pos[1]*R2D;
	INScal.pos[2] = infor.pos[2];

	double temp_fixanlge[3] = { 0 };
	calipara.fixup(temp_fixanlge, infor.cnb_mat);
	if (0 == TestModeNum)
	{
		INScal.err_ang[0] = temp_fixanlge[0] * R2D - ZT.ang[0];
		INScal.err_ang[1] = temp_fixanlge[1] * R2D - ZT.ang[1];
		INScal.err_ang[2] = temp_fixanlge[2] * R2D - ZT.ang[2];
		INScal.err_pos[0] += infor.vel_n[0] * sysc.Ts;
		INScal.err_pos[1] += infor.vel_n[1] * sysc.Ts;
		INScal.err_pos[2] = sqrt(INScal.err_pos[0]*INScal.err_pos[0] + INScal.err_pos[1]*INScal.err_pos[1]);
		memcpy(INScal.err_vel, infor.vel_n, sizeof(INScal.err_vel));
	}
	if (1 == TestModeNum)
	{
		INScal.err_ang[0] = temp_fixanlge[0] * R2D - phins.ang[0];
		INScal.err_ang[1] = temp_fixanlge[1] * R2D - phins.ang[1];
		INScal.err_ang[2] = temp_fixanlge[2] * R2D - phins.ang[2];
		INScal.err_pos[0] = (infor.pos[0] - phins.pos[0] * D2R)*RE;
		INScal.err_pos[1] = (infor.pos[1] - phins.pos[1] * D2R)*RE;
		INScal.err_pos[2] = sqrt(INScal.err_pos[0]*INScal.err_pos[0] + INScal.err_pos[1]*INScal.err_pos[1]);
		INScal.err_vel[0] = infor.vel_n[0] - phins.vel[0];
		INScal.err_vel[1] = infor.vel_n[1] - phins.vel[1];
		INScal.err_vel[2] = infor.vel_n[2] - phins.vel[2];
	}
	if (6 == TestModeNum)
	{
		INScal.err_ang[0] = temp_fixanlge[0] * R2D - ZT.ang[0];
		INScal.err_ang[1] = temp_fixanlge[1] * R2D - ZT.ang[1];
		INScal.err_ang[2] = temp_fixanlge[2] * R2D - ZT.ang[2];
		INScal.err_pos[0] = (infor.pos[0] - real_pos[0])*RE;
		INScal.err_pos[1] = (infor.pos[1] - real_pos[1])*RE;
		INScal.err_pos[2] = sqrt(INScal.err_pos[0]*INScal.err_pos[0] + INScal.err_pos[1]*INScal.err_pos[1]);
		INScal.err_vel[0] = infor.vel_n[0] - phins.vel[0];
		INScal.err_vel[1] = infor.vel_n[1] - phins.vel[1];
		INScal.err_vel[2] = infor.vel_n[2] - phins.vel[2];
	}


}
#pragma endregion DataPro

//////////////////处理函数及线程///////////////////////////////
#pragma region ProFunc
////粗对准
void CWindowsMainControlV1Dlg::CoarseThread()
{
	int j;
	sysc.coarse_time = sysc.coarse_time;
	if (!sysc.f_coarse_over)
	{
		if (is_data_used == 0)
		{
			/* 导航数据读取 */
			for (j = 0; j < 3; j++)
			{
				/* 存储上一帧数据 */
				if (1 == sysc.data_cnt)
				{
					infor.gyro_old[j] = IMUout.gyro_b[j] * D2R;
					infor.acce_old[j] = IMUout.acce_b[j] * D2R;
				}
				else
				{
					infor.gyro_old[j] = infor.gyro_wib_b[j]; // 在陀螺量测没有更新之前复制
					infor.acce_old[j] = infor.acce_b[j];
				}
				infor.gyro_wib_b[j] = IMUout.gyro_b[j] * D2R; // 新的陀螺量测
				infor.acce_b[j] = IMUout.acce_b[j];		 // 加表数据	
			}
			calipara.IMUcalibrate(infor);
			switch (CoarseModeNum)
			{
			case COARSE_NG:coarse_ng(); sysc.state = _T("凝固解析法"); break;
			}
			is_data_used = 1;

		}
	}
	return;
}
////精对准
void CWindowsMainControlV1Dlg::FineThread()
{
	int i, j;
	double temp_ob[3] = { 0 };	
	double gyro[2][3] = { 0 };
	if (!sysc.f_fine_over&&sysc.f_coarse_over)
	{

		if (is_data_used == 0)
		{
			/* 导航数据读取 */
			for (j = 0; j < 3; j++)
			{
				if (1 == sysc.data_cnt)
				{
					infor.gyro_old[j] = IMUout.gyro_b[j] * D2R;
					infor.acce_old[j] = IMUout.acce_b[j] * D2R;
				}
				else
				{
					infor.gyro_old[j] = infor.gyro_wib_b[j]; // 在陀螺量测没有更新之前复制
					infor.acce_old[j] = infor.acce_b[j];
				}
				infor.gyro_wib_b[j] = IMUout.gyro_b[j] * D2R; // 新的陀螺量测
				infor.acce_b[j] = IMUout.acce_b[j];		 // 加表数据	
			}
			calipara.IMUcalibrate(infor);//零偏补偿
			for (i = 0; i < 3; i++)
			{
				infor.gyro_wib_b[i] = infor.gyro_wib_b[i] - cmp.wc_b[i];
				infor.acce_b[i] = infor.acce_b[i] - cmp.fc_b[i];
			}
			//纯惯性模式选择
			switch (PureINSModeNum)
			{
			case PURE_SINS_UNDUMP:sinscal_zundamp(sysc.Ts); break;
			case PURE_SINS_RV:sinscal_rv(sysc.Ts);break;
			case PURE_SINS_TRANSVERSE:
				sinscal_zundamp(sysc.Ts);
				for (int i = 0; i<3; i++)
				{
					gyro[0][i] = infor.gyro_old[i];
					gyro[1][i] = infor.gyro_wib_b[i];
				}
				for (int i = 0; i<3; i++) inforS.acce_b[i] = infor.acce_b[i];
				sinscal_TRANSVERSE(inforS, sysc.Ts, gyro);
				sysc.state = _T("横向纯惯性");
				break;	
			default:break;
			}
			switch (FineModeNum)
			{
			case FINE_CMPS:fine_cmps(); sysc.state = _T("罗经法精对准"); break;
			case FINE_Yucia:
				switch (TestModeNum)
				{
				case 0:avecmul(3, temp_ob, infor.initial_pos, 1); break;
				case 1:
					avecmul(3, temp_ob, phins.pos, 1);
					temp_ob[0] *= D2R;
					temp_ob[1] *= D2R;
					break;
				case 6:
					avecmul(3, temp_ob, phins.pos, 1);
					temp_ob[0] *= D2R;
					temp_ob[1] *= D2R;
					break;
				default:break;
				}
				fine_yucia(fkalman, temp_ob, YA_POS);
				sysc.state = _T("鱼叉姐完美对准");
				break;
			case FINE_0su:
				switch (TestModeNum)
				{
				case 0:memset(temp_ob,0,sizeof(temp_ob)); break;
				case 1:avecmul(3, temp_ob, phins.vel, 1.0); break;
				case 6:avecmul(3, temp_ob, phins.vel, 0.0); break;	
				default:break;
				}
				fine_yucia(fkalman, temp_ob, YA_VEL);
				sysc.state = _T("零速校正对准");
				break;
			case FINE_ADRC:fine_adrc(); sysc.state = _T("DR.Yang的瞎JB对准"); break;

			}
			is_data_used = 1;
		}
	}
	return;
}
////导航
void CWindowsMainControlV1Dlg::NaviThread(void)
{
	int j;
	double tempob[3] = {0};//15/3滤波器的3维观测量
	double tempob_v[3] = { 0 }; //速度观测量
	double tempob_vb[3] = { 0 }; //计程仪（phins）b系速度观测量  
	double tempob_att[3] = { 0 };//姿态观测量
	double gyro[2][3] = { 0 };
	double cnb_phins[3][3] = { 0 };//用来存放phins的Cnb,INS/DVL组合用
	double fix_err[3] = {0.0153*D2R,-0.1891*D2R,-0.1724*D2R}, Cpb[3][3] = { 0 };
	double delta_v[3]= { 0 };
	ang2cnb(Cpb, fix_err);   //p（phins系）为下标

	if (!sysc.f_navi_over&&sysc.f_coarse_over&&sysc.f_fine_over)
	{
		temp_test = false;
		if (is_data_used == 0)
		{
			/* 导航数据读取 */
			for (j = 0; j < 3; j++)
			{
				if (1 == sysc.data_cnt)
				{
					infor.gyro_old[j] = IMUout.gyro_b[j] * D2R;
					infor.acce_old[j] = IMUout.acce_b[j] * D2R;
				}
				else
				{
					infor.gyro_old[j] = infor.gyro_wib_b[j]; // 在陀螺量测没有更新之前复制
					infor.acce_old[j] = infor.acce_b[j];
				}				
				infor.gyro_wib_b[j] = IMUout.gyro_b[j] * D2R; // 新的陀螺量测
				infor.acce_b[j] = IMUout.acce_b[j];		 // 加表数据			
			}
			calipara.IMUcalibrate(infor);
			//haishi情况下和杆臂参数有关
			switch (PureINSModeNum)
			{
			case PURE_SINS_UNDUMP:
				sinscal_zundamp(sysc.Ts);
				break;
			case PURE_SINS_RV:
				sinscal_rv(sysc.Ts);
				break;
			case PURE_SINS_TRANSVERSE:
				
				sinscal_zundamp(sysc.Ts);
				for (int i = 0; i<3; i++)
				{
					gyro[0][i] = infor.gyro_old[i];
					gyro[1][i] = infor.gyro_wib_b[i];
				}
				for (int i = 0; i<3; i++) inforS.acce_b[i] = infor.acce_b[i];
				sinscal_TRANSVERSE(inforS, sysc.Ts, gyro);
				sysc.state = _T("横向纯惯性");
				break;
			case PURE_SINS_HAISHI_P:
				infor.rp[0] = 0.05; infor.rp[1] = 50; infor.rp[2] = 1;
				break;
			case PURE_SINS_HAISHI_L:
				infor.rp[0] = 0.05; infor.rp[1] = 15; infor.rp[2] = 15;
				break;
			case PURE_SINS_HAISHI_0RP:
				infor.rp[0] = 0; infor.rp[1] = 0; infor.rp[2] = 0;
				break;
			default: break;
			}
			switch (NaviModeNum)
			{
			case NAVI_SINS_UNDUMP:sysc.state = _T("纯惯性"); break;
			case NAVI_SG:
				tempob[0] = gps.pos[0] * D2R;
				tempob[1] = gps.pos[1] * D2R;
				tempob[2] = gps.pos[2];
				/*tempob[0] = fosn.pos[0] * D2R;
				tempob[1] = fosn.pos[1] * D2R;
				tempob[2] = fosn.pos[2];*/
				vecsub(3, tempob, infor.pos, tempob);
				//输入的观测量已经是差值
				navi_Kal_15_3(nkalman, tempob, YA_POS);
				sysc.state = _T("GPS位置组合");
				break;
			case NAVI_VEL:
				avecmul(3, tempob_v, gps.vel, 1);//观测量的获得方式
				vecsub(3, tempob, infor.vel_n, tempob_v);
				//输入的观测量已经是差值
				navi_Kal_15_3(nkalman, tempob, YA_VELANDAZ);
				sysc.state = _T("GPS速度组合");
				break;
			case NAVI_PHINS_POS:
				tempob[0] = phins.pos[0] * D2R;
				tempob[1] = phins.pos[1] * D2R;
				tempob[2] = phins.pos[2];
				vecsub(3, tempob, infor.pos, tempob);
				//输入的观测量已经是差值
				navi_Kal_15_3(nkalman, tempob, YA_POS);
				sysc.state = _T("PS位置组合");
				break;

			case NAVI_VELANDAZ:
				avecmul(3, tempob_att, phins.ang, D2R);//观测量的获得方式
				avecmul(3, tempob_v, phins.vel , 1);
				if (tempob_att[2] > PAI)
					tempob_att[2] -= 2 * PAI;
				if (tempob_att[2] < -PAI)
					tempob_att[2] += 2 * PAI;
				vecsub(3, tempob, infor.att_angle, tempob_att);				
				DeltaAtt2Phi(infor, tempob, tempob);//姿态误差角到失准角处理
				vecsub(2, tempob, infor.vel_n, tempob_v);

				cvecmul(infor.vel_arm, infor.wnb_b_arm, infor.rp);
				vecmul(3, 3, infor.vel_arm, (double *)infor.cbn_mat, infor.vel_arm);
				tempob[0] = tempob[0] - infor.vel_arm[0];
				tempob[1] = tempob[1] - infor.vel_arm[1];
				//输入的观测量已经是差值
				navi_Kal_15_3(nkalman,tempob, YA_VELANDAZ);
				sysc.state = _T("n系速度+航向组合");
				break;
			case NAVI_PHINS_VEL1:
				avecmul(3, tempob_v, phins.vel, 1);//观测量的获得方式
				vecsub(3, tempob, infor.vel_n, tempob_v);
				//输入的观测量已经是差值
				navi_Kal_15_3(nkalman,tempob, YA_VELANDAZ);
				sysc.state = _T("PSn系速度组合");
				break;
			case NAVI_HAISHI_BASIC:
				navigation(phins.vel[0], phins.vel[1], phins.ang[2] * D2R, NAVI_HAISHI_BASIC); 
				sysc.state = _T("haishi一般组合"); 
				break;
			case NAVI_HAISHI_JZ:
				navigation(phins.vel[0], phins.vel[1], phins.ang[2] * D2R, NAVI_HAISHI_JZ); 
				sysc.state = _T("haishi降噪滤波"); 
				break;
			case NAVI_PHINS_VEL2:                           //20171128	
				avecmul(3, tempob_v, phins.ang, D2R);//乘以刻度因子
				ang2cnb(cnb_phins, tempob_v);
				vecmul(3, 3, phins.vel_b, (double*)cnb_phins, phins.vel);
				avecmul(3, tempob_v, phins.vel_b, 1.04);//乘以刻度因子
				vecmul(3, 3, tempob, (double*)Cpb, tempob_v); //phins系速度转到fosn系
				navi_Kal_16_3(kalman_dvl, tempob); 
				sysc.state = _T("计程仪速度组合");
				break;
			default:break;
			}

			is_data_used = 1;
			datanavinum++;
			//	temp_test = true;
		}
	}
	return;
}
////板卡数据接收线程
UINT CWindowsMainControlV1Dlg::CardRec(LPVOID pParam)
{
	HWND hwnd = ((THREADPARAM*)pParam)->hwnd;
	delete pParam;

	static int count200 = 1;//200个数据的计数器(sysc.Fs决定是200个还是多少个，通常200)
	static long int rec_count = 0;
	while (!mquit)
	{
		if (is_cardReceive)
		{
			if (is_cardReset)
			{
				Sio_Rx_ResetFIFO(hCard, 0);//复位板卡
				Sio_Rx_ResetFIFO(hCard, 1);
				Sio_Rx_ResetFIFO(hCard, 2);
				is_cardReset = 0;
			}
			if (FOSNChannel() == 0)
				continue;
			if (TestModeNum != 1)ZTChannel();
			if (Rec200times(count200)) 
				gps.flag = GPSChannel();

			if (is_startCal&&isStartCalOk)
			{

				IMUdataCount();
				DataTrans();
				CoarseThread();
				FineThread();
				NaviThread();
			}
			rec_count++;
			if (1 == is_UpdateData)
			{
				if (rec_count % (refresh_time / 5) == 0)
				{
					::PostMessage(hwnd, WM_UPDATEDATA, NULL, NULL);
				}
			}
			if (is_timeSave&&is_startCal)
			{
				if (m_cnt_s - temp_cnt_s == save_time)
					::PostMessage(hwnd, WM_TIMESAVE, NULL, NULL);
			}

			if (TestModeNum < 2)//导航模式下，运算开始之后才录数
			{
				if (is_startCal&&saveStart)
					SaveData();
			}
			else               //纯录数模式下，就直接录数
			{
				if (saveStart)
				{
					m_PRecNum++;
					SaveData();
				}

			}

		}
		else
			return 0;
	}
	return 0;
}
////半物理仿真数据线程
UINT CWindowsMainControlV1Dlg::SimulateThread(LPVOID pParam)
{
	HWND hwnd = ((THREADPARAM*)pParam)->hwnd;
	delete pParam;

	static long int rec_count = 0;
	while (!mquit)
	{
		if (RS_para.canCal&&isStartCalOk)     //isStartCalOk解算开始之后才会开始解算，所以保证了初始化在解算之前  20171128 
		{
			getfileData();
			if (RS_para.RS_mode == 0)
			{
				if (RS_para.RdataFilefid != NULL) fclose(RS_para.RdataFilefid);
				sysc.state = _T("数据文件运算到头了");
				::PostMessage(hwnd, WM_UPDATEDATA, NULL, NULL);
				rec_count = 0;
				return 0;
			}
			IMUdataCount();
			if (sysc.cnt_s == RS_para.skiptime)//设定一个跳过数据的长度，该段不解算
			{
				sysc.cnt_s = 0;
				RS_para.skiptime = -1;

				infor.pos[0] = phins.pos[0] * D2R;
				infor.pos[1] = phins.pos[1] * D2R;
				infor.pos[2] = phins.pos[2];
				initial_latitude = infor.pos[0] * R2D;
				initial_longitude = infor.pos[1] * R2D;
				initial_height = infor.pos[2];
				infor.vel_n[0] = phins.vel[0];       
				infor.vel_n[1] = phins.vel[1];
				infor.vel_n[2] = phins.vel[2];
				infor.att_angle[0] = (phins.ang[0] + 0.1) * D2R;
				infor.att_angle[1] = (phins.ang[1] + 0.1) * D2R;
				infor.att_angle[2] = (phins.ang[2] + 0.5) * D2R;
				ang2cnb(infor.cnb_mat, infor.att_angle);
				cnb2q(infor.cnb_mat, infor.quart);
				optq(infor.quart);

				
			}
			if (sysc.cnt_s > RS_para.skiptime)
			{
				CoarseThread();
				FineThread();
				NaviThread();
			}	

			DataTrans();
			if (RS_para.delay5ms == 1) RS_para.canCal = 0;

			rec_count++;
			if (1 == is_UpdateData)
			{
				if (rec_count % (refresh_time / 5) == 0)
				{
					::PostMessage(hwnd, WM_UPDATEDATA, NULL, NULL);
				}
			}
			if (is_timeSave&&is_startCal)
			{
				if (m_cnt_s - temp_cnt_s == save_time)
					::PostMessage(hwnd, WM_TIMESAVE, NULL, NULL);
			}

			if (is_startCal&&saveStart)
				SaveData();
		}

	}
	return 0;
}
////phins数据接收的网络线程
UINT CWindowsMainControlV1Dlg::PHINSThread(LPVOID pParam)
{
	int len = sizeof(SOCKADDR);
	SOCKADDR_IN addrFrom;
	SOCKET sock = ((SOCKETPARAM*)pParam)->sock;
	HWND hwnd = ((SOCKETPARAM*)pParam)->hwnd;
	delete pParam;
	static bool firstps = true;

	while (is_start_phins)
	{
		int retval = recvfrom(sock, m_Recv_PHINS_Buff, 256, 0, (SOCKADDR*)&addrFrom, &len);
		if (retval == 42 && m_Recv_PHINS_Buff[0] == 0x71)
			phins.Conv(m_Recv_PHINS_Buff);
		//获得phins数据且解算初始化OK之后，通过phins赋一次姿态和位置初值
		if (firstps == true)
		{
			if (isStartCalOk)
			{
				infor.pos[0] = phins.pos[0] * D2R;
				infor.pos[1] = phins.pos[1] * D2R;
				infor.pos[2] = initial_height;
				avecmul(3, infor.att_angle, phins.ang, D2R);
				ang2cnb(infor.cnb_mat, infor.att_angle);
				cnb2q(infor.cnb_mat, infor.quart);
				optq(infor.quart);
				firstps = false;
			}
		}	
	}

	return 0;
}
////PC104数据接收线程
UINT CWindowsMainControlV1Dlg::PC104RecThread(LPVOID pParam)
{
	SOCKET sock = ((SOCKETPARAM*)pParam)->sock;
	HWND hwnd = ((SOCKETPARAM*)pParam)->hwnd;
	delete pParam;
	int len = sizeof(SOCKADDR);
	char recvBuf[512] = { 0 };
	SOCKADDR_IN addrFrom;
	int i;	
	DB9CH db9ch;
	static long int rec_count = 0;
	while (!pc104RecQuit)
	{
		int retval = recvfrom(sock, recvBuf, 512, 0, (SOCKADDR*)&addrFrom, &len);
		if ( retval == 512)
		{
			memcpy(bufFOSN, recvBuf + 4, sizeof(bufFOSN));
			fosn.Conv(bufFOSN);
			for (i = 0; i < 3; i++)
			{
				IMUout.acce_b[i] = fosn.fb[i];
				IMUout.gyro_b[i] = fosn.wb[i];
			}
			fosn.recnum++;

			memcpy(BufGPS, recvBuf + 74, sizeof(BufGPS));
			gps.flag = gps.Conv(BufGPS);
			gps.cnt++;

			memcpy(m_Recv_PHINS_Buff, recvBuf + 254, sizeof(m_Recv_PHINS_Buff));
			phins.Conv(m_Recv_PHINS_Buff);			
		
			memcpy(db9ch.ch, recvBuf + 296, sizeof(db9ch.ch));
			infor.att_angle[1] = db9ch.db[0];	infor.att_angle[2] = db9ch.db[1];	infor.att_angle[0] = db9ch.db[2];
			infor.vel_n[1] = db9ch.db[3];		infor.vel_n[2] = db9ch.db[4];		infor.vel_n[0] = db9ch.db[5];
			infor.pos[0] = db9ch.db[6] ;		infor.pos[1] = db9ch.db[7] ;		infor.pos[2] = db9ch.db[8];
			DataTrans();

			rec_count++;
			if (1 == is_UpdateData)
			{
				if (rec_count % (refresh_time / 5) == 0)
				{
					::PostMessage(hwnd, WM_UPDATEDATA, NULL, NULL);
				}
			}
			if (saveStart)
			{
				m_PRecNum++;
				SaveData();
			}
		}
	}
	return 0;
}
#pragma endregion ProFunc

//定时器回调函数
void CALLBACK TimeDalay(UINT uID, UINT uMsg, DWORD dwUsers, DWORD dw1, DWORD dw2)
{
	RS_para.canCal = 1;
}







