// ReadSimuDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "WindowsMainControlV1.h"
#include "ReadSimuDlg.h"
#include "afxdialogex.h"


// ReadSimuDlg 对话框

IMPLEMENT_DYNAMIC(ReadSimuDlg, CDialogEx)

ReadSimuDlg::ReadSimuDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(ReadSimuDlg::IDD, pParent)
{
	
}

ReadSimuDlg::~ReadSimuDlg()
{
}

void ReadSimuDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_COMBO_FileFormat, RS_FileFormat);
	DDX_Control(pDX, IDC_CK_Delay5ms, RS_Delay5ms);
	DDX_Control(pDX, IDC_CK_ReadInitPos, RS_ReadInitPos);
	DDX_Control(pDX, IDC_CK_ReadInitAtt, RS_ReadInitAtt);

	init_RScombo();//我真醉了，这个初始化不能放在构造函数里。猜测原因可能是因为这个对话框还有没有初始化好，上面的变量也没有绑定好
}


BEGIN_MESSAGE_MAP(ReadSimuDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BTN_ChoosTXT, &ReadSimuDlg::OnBnClickedBtnChoostxt)
	ON_BN_CLICKED(IDC_BTN_Formatexp, &ReadSimuDlg::OnBnClickedBtnFormatexp)
	ON_BN_CLICKED(IDOK, &ReadSimuDlg::OnBnClickedOk)
END_MESSAGE_MAP()


// ReadSimuDlg 消息处理程序
void ReadSimuDlg::init_RScombo()
{
	int judge_tf;
	int i;
	CString str[] = { _T("车载44位标准格式"),_T("纯录数格式1"),_T("转台模式录数"),_T("仿真数据格式1"),_T("仿真数据格式2"),_T("鱼叉姐数据"),_T("matlab仿真格式1"),_T("haishi录数格式") };
	for (i = 0; i<8; i++)
	{
		judge_tf = RS_FileFormat.InsertString(i, str[i]);
		if ((judge_tf == CB_ERR) || (judge_tf == CB_ERRSPACE))
		{
			MessageBox(_T("build baud error!"));
		}
	}
	RS_FileFormat.SetCurSel(0);
	RS_Delay5ms.SetCheck(0);
	RS_ReadInitPos.SetCheck(1);
	RS_ReadInitAtt.SetCheck(1);
	state_rtn = 0;
}

void ReadSimuDlg::OnBnClickedBtnChoostxt()
{
	// TODO: 在此添加控件通知处理程序代码
	CString szFilter;
	szFilter = "*.txt|*.txt|All Files (*.*)|*.*||";
	CFileDialog m_FileDialog0(true, NULL, NULL, 0, szFilter, NULL);
	if (m_FileDialog0.DoModal())
	{
		if ((filename = m_FileDialog0.GetPathName()) == "")
		{
			state_rtn = 5;//读取文件出错
			return;
		}		
		else state_rtn = 6;
	}
}


void ReadSimuDlg::OnBnClickedBtnFormatexp()
{
	// TODO: 在此添加控件通知处理程序代码
	CString c;
	c = "纯录数格式1：\n\
1~3   录数统计，转台数据号，fosn时间\n\
4~9   陀螺加表数据\n\
10~12 转台姿态\n\
13~15 FOSN姿态\n\n\
仿真数据格式1\n\
1     时间\n\
2~7   陀螺加表数据\n\
8~10  三个姿态\n\
11~13 三个速度\n\
14~16 经纬高\n\
15~16 不用管\n\
17~22 无误差陀螺加表数据\n\n\
车载44位标准格式\n\
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
42~44 FOSN位置 \n\nby Dr.Yang";
	MessageBox(c, _T("★★★Windows导航平台读数仿真程序格式说明★★★"), MB_OK);

}


void ReadSimuDlg::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	//CDialogEx::OnOK();
	combonum = RS_FileFormat.GetCurSel();
	Delay5ms=RS_Delay5ms.GetCheck();//0表示未选中状态，1表示选中状态，2表示不确定状态（仅用于复选框）
	ReadInitAtt=RS_ReadInitAtt.GetCheck();
	ReadInitPos = RS_ReadInitPos.GetCheck();
	EndDialog(state_rtn);
}
