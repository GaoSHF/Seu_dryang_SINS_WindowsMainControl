#include "Struct.h"
#pragma once


// Calibration 对话框

class Calibration : public CDialogEx
{
	DECLARE_DYNAMIC(Calibration)

public:
	Calibration(CWnd* pParent = NULL,bool iserr_input=false);   // 标准构造函数
	virtual ~Calibration();
	bool err_input;
// 对话框数据
	enum { IDD = IDD_DLG_Calibration };
	CALIPMT calipmt;
protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnLoad();
};
