
// WindowsMainControlV1Dlg.h : 头文件

#pragma once
#include "afxwin.h"

#define MAINMODE 1      //1 normal，2 haishi
// CWindowsMainControlV1Dlg 对话框
class CWindowsMainControlV1Dlg : public CDialogEx
{
// 构造
public:
	CWindowsMainControlV1Dlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_WINDOWSMAINCONTROLV1_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	HWND ghWnd;//用于保存当前对话框的句柄

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnStartcard();
	afx_msg void OnBnClickedBtnStartphins();
	afx_msg void OnBnClickedBtnStartcal();
	afx_msg void OnBnClickedBtnSavepath();
	afx_msg void OnBnClickedBtnSaveall();
	afx_msg void OnBnClickedBtnSinglefile();
	afx_msg void OnBnClickedBtnTimeset();
	afx_msg void OnBnClickedBtnReadme();
	afx_msg void OnBnClickedBtnSaveHelp();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedBtnReset();
	afx_msg void OnBnClickedBtnSavetime();
	//数据更新消息
	afx_msg LRESULT updateData(WPARAM wParam,LPARAM lParam);
	afx_msg LRESULT saveCancel(WPARAM wParam,LPARAM lParam);

public:
	//****自定义功能函数
	void init_var();
	void init_mainmode();
	CString GetDirectory();//文件夹路径获取
	bool init_Combo();////combo初始化
	bool Init_Net(void);
	int edit_data_f;//数据输入的频率；
	SOCKET m_socketPHINSDataRec;
	SOCKET m_socketPC104DataRec;
	
	//Combo控制变量
	CComboBox m_CoarseAignMode;
	CComboBox m_FineAignMode;
	CComboBox m_NaviMode;
	CComboBox m_HeightMode;
	CComboBox m_TestMode;


	static UINT CardRec(LPVOID pParam);//板卡数据接收线程函数
	static void CoarseThread();
	static void FineThread();//
	static void NaviThread();//
	static UINT PHINSThread(LPVOID pParam);
	static UINT SimulateThread(LPVOID pParam);//仿真数据线程
	static UINT PC104RecThread(LPVOID pParam);//PC104数据接收线程
	static bool Rec200times(int &count200);//每接收数据200个返回1
	static int FOSNChannel();
	static void ZTChannel();	
	static int GPSChannel();	
	static void IMUdataCount();//从FOSN获得IMU数据
	static void getfileData();//从文件读取数据
	static void CalVarInit(char mode);//从界面读取的数据向导航传递
	
	static void SaveData();
	static void DataTrans();
	
	
	afx_msg void OnBnClickedBtnStoprefresh();
	afx_msg void OnBnClickedBtnCalibration();
	afx_msg void OnCbnSelchangeTextMode();

	
	afx_msg void OnCbnSelchangeComboFinealignmode();
};


#define WM_UPDATEDATA WM_USER+1
#define WM_TIMESAVE WM_USER+2
//线程结构体，用来传递对话框句柄
struct THREADPARAM           
{	
	HWND hwnd;
};
struct SOCKETPARAM           
{	
	SOCKET sock;
	HWND hwnd;
};