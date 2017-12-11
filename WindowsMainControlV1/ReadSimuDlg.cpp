// ReadSimuDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "WindowsMainControlV1.h"
#include "ReadSimuDlg.h"
#include "afxdialogex.h"


// ReadSimuDlg �Ի���

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

	init_RScombo();//�������ˣ������ʼ�����ܷ��ڹ��캯����²�ԭ���������Ϊ����Ի�����û�г�ʼ���ã�����ı���Ҳû�а󶨺�
}


BEGIN_MESSAGE_MAP(ReadSimuDlg, CDialogEx)
	ON_BN_CLICKED(IDC_BTN_ChoosTXT, &ReadSimuDlg::OnBnClickedBtnChoostxt)
	ON_BN_CLICKED(IDC_BTN_Formatexp, &ReadSimuDlg::OnBnClickedBtnFormatexp)
	ON_BN_CLICKED(IDOK, &ReadSimuDlg::OnBnClickedOk)
END_MESSAGE_MAP()


// ReadSimuDlg ��Ϣ�������
void ReadSimuDlg::init_RScombo()
{
	int judge_tf;
	int i;
	CString str[] = { _T("����44λ��׼��ʽ"),_T("��¼����ʽ1"),_T("ת̨ģʽ¼��"),_T("�������ݸ�ʽ1"),_T("�������ݸ�ʽ2"),_T("��������"),_T("matlab�����ʽ1"),_T("haishi¼����ʽ") };
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
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString szFilter;
	szFilter = "*.txt|*.txt|All Files (*.*)|*.*||";
	CFileDialog m_FileDialog0(true, NULL, NULL, 0, szFilter, NULL);
	if (m_FileDialog0.DoModal())
	{
		if ((filename = m_FileDialog0.GetPathName()) == "")
		{
			state_rtn = 5;//��ȡ�ļ�����
			return;
		}		
		else state_rtn = 6;
	}
}


void ReadSimuDlg::OnBnClickedBtnFormatexp()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString c;
	c = "��¼����ʽ1��\n\
1~3   ¼��ͳ�ƣ�ת̨���ݺţ�fosnʱ��\n\
4~9   ���ݼӱ�����\n\
10~12 ת̨��̬\n\
13~15 FOSN��̬\n\n\
�������ݸ�ʽ1\n\
1     ʱ��\n\
2~7   ���ݼӱ�����\n\
8~10  ������̬\n\
11~13 �����ٶ�\n\
14~16 ��γ��\n\
15~16 ���ù�\n\
17~22 ��������ݼӱ�����\n\n\
����44λ��׼��ʽ\n\
1~5   ¼��ͳ�ƣ�ת̨/�๦�ܰ�֡��(����)��fosnʱ��,gpsʱ�䣬phinsʱ��\n\
6~11  ���ݼӱ�\n\
12~14 ������̬\n\
15~17 �����ٶ�\n\
18~20 ����λ��\n\
21~23 PHINS��̬\n\
24~26 PHINS�ٶ�\n\
27~29 PHINSλ��\n\
30~35 GPSλ�ã��ٶȣ�û�У�\n\
36~38 FOSN��̬\n\
39~41 FOSN�ٶ�\n\
42~44 FOSNλ�� \n\nby Dr.Yang";
	MessageBox(c, _T("����Windows����ƽ̨������������ʽ˵������"), MB_OK);

}


void ReadSimuDlg::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	//CDialogEx::OnOK();
	combonum = RS_FileFormat.GetCurSel();
	Delay5ms=RS_Delay5ms.GetCheck();//0��ʾδѡ��״̬��1��ʾѡ��״̬��2��ʾ��ȷ��״̬�������ڸ�ѡ��
	ReadInitAtt=RS_ReadInitAtt.GetCheck();
	ReadInitPos = RS_ReadInitPos.GetCheck();
	EndDialog(state_rtn);
}
