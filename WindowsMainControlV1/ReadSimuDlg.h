#pragma once
#include "afxwin.h"


// ReadSimuDlg 对话框

class ReadSimuDlg : public CDialogEx
{
	DECLARE_DYNAMIC(ReadSimuDlg)

public:
	ReadSimuDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~ReadSimuDlg();

// 对话框数据
	enum { IDD = IDD_DLG_ReadSimu };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnChoostxt();
	CComboBox RS_FileFormat;
	int combonum;
	int state_rtn;//返回状态
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
