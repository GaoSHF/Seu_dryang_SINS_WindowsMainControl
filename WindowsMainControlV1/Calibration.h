#include "Struct.h"
#pragma once


// Calibration �Ի���

class Calibration : public CDialogEx
{
	DECLARE_DYNAMIC(Calibration)

public:
	Calibration(CWnd* pParent = NULL,bool iserr_input=false);   // ��׼���캯��
	virtual ~Calibration();
	bool err_input;
// �Ի�������
	enum { IDD = IDD_DLG_Calibration };
	CALIPMT calipmt;
protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnLoad();
};
