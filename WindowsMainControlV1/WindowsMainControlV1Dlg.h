
// WindowsMainControlV1Dlg.h : ͷ�ļ�

#pragma once
#include "afxwin.h"

#define MAINMODE 1      //1 normal��2 haishi
// CWindowsMainControlV1Dlg �Ի���
class CWindowsMainControlV1Dlg : public CDialogEx
{
// ����
public:
	CWindowsMainControlV1Dlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_WINDOWSMAINCONTROLV1_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


// ʵ��
protected:
	HICON m_hIcon;

	HWND ghWnd;//���ڱ��浱ǰ�Ի���ľ��

	// ���ɵ���Ϣӳ�亯��
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
	//���ݸ�����Ϣ
	afx_msg LRESULT updateData(WPARAM wParam,LPARAM lParam);
	afx_msg LRESULT saveCancel(WPARAM wParam,LPARAM lParam);

public:
	//****�Զ��幦�ܺ���
	void init_var();
	void init_mainmode();
	CString GetDirectory();//�ļ���·����ȡ
	bool init_Combo();////combo��ʼ��
	bool Init_Net(void);
	int edit_data_f;//���������Ƶ�ʣ�
	SOCKET m_socketPHINSDataRec;
	SOCKET m_socketPC104DataRec;
	
	//Combo���Ʊ���
	CComboBox m_CoarseAignMode;
	CComboBox m_FineAignMode;
	CComboBox m_NaviMode;
	CComboBox m_HeightMode;
	CComboBox m_TestMode;


	static UINT CardRec(LPVOID pParam);//�忨���ݽ����̺߳���
	static void CoarseThread();
	static void FineThread();//
	static void NaviThread();//
	static UINT PHINSThread(LPVOID pParam);
	static UINT SimulateThread(LPVOID pParam);//���������߳�
	static UINT PC104RecThread(LPVOID pParam);//PC104���ݽ����߳�
	static bool Rec200times(int &count200);//ÿ��������200������1
	static int FOSNChannel();
	static void ZTChannel();	
	static int GPSChannel();	
	static void IMUdataCount();//��FOSN���IMU����
	static void getfileData();//���ļ���ȡ����
	static void CalVarInit(char mode);//�ӽ����ȡ�������򵼺�����
	
	static void SaveData();
	static void DataTrans();
	
	
	afx_msg void OnBnClickedBtnStoprefresh();
	afx_msg void OnBnClickedBtnCalibration();
	afx_msg void OnCbnSelchangeTextMode();

	
	afx_msg void OnCbnSelchangeComboFinealignmode();
};


#define WM_UPDATEDATA WM_USER+1
#define WM_TIMESAVE WM_USER+2
//�߳̽ṹ�壬�������ݶԻ�����
struct THREADPARAM           
{	
	HWND hwnd;
};
struct SOCKETPARAM           
{	
	SOCKET sock;
	HWND hwnd;
};