#pragma once
#include "afxwin.h"


// ReadSimuDlg �Ի���

class ReadSimuDlg : public CDialogEx
{
	DECLARE_DYNAMIC(ReadSimuDlg)

public:
	ReadSimuDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~ReadSimuDlg();

// �Ի�������
	enum { IDD = IDD_DLG_ReadSimu };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnChoostxt();
	CComboBox RS_FileFormat;
	int combonum;
	int state_rtn;//����״̬
	CString filename;
	afx_msg void OnBnClickedBtnFormatexp();
	afx_msg void OnBnClickedOk();
	CButton RS_Delay5ms;
	CButton RS_ReadInitPos;
	CButton RS_ReadInitAtt;
	int Delay5ms, ReadInitPos, ReadInitAtt;
	int Skiptime;
	void init_RScombo();
};
