#include "Struct.h"
#include "CreateRoute.h"
#pragma once


// SimuDataDlg 对话框

class SimuDataDlg : public CDialogEx
{
	DECLARE_DYNAMIC(SimuDataDlg)

public:
	SimuDataDlg(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~SimuDataDlg();

// 对话框数据
	enum { IDD = IDD_DLG_SimuData };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CALIPMT err_pmt;//误差参数
	IMUOUT IMUSimuout;
	Route route_show;//控件显示的保存量；懒得每个控件创建变量了,只用摇摆的的0和wt0，at0。wt0保存角速度或者速度，at0保存加速度或者时间
	Route route[50];
	int routenum;
	int ordernum;//显示的数列长度
	bool justaddswing;//上一条也是添加的摇摆
	CString filename;
	FILE * Iputfid;
	CComboBox SD_Combo_turn, SD_Combo_AxisXmode, SD_Combo_AxisYmode, SD_Combo_AxisZmode, SD_Combo_AngularAxis, SD_Combo_PositionAxis;
	CListBox SD_list_Moveorder;
	double lati, longi, high;
	bool creat_swing;//创建摇摆成功
	bool is_combomove;//是否是复合运动
	afx_msg void OnBnClickedBtnAdderr();
	afx_msg void OnBnClickedBtnSavesimufile();
	afx_msg void OnBnClickedBtnAddswing();
	afx_msg void OnBnClickedBtn();
	afx_msg void OnBnClickedBtnAddmove();
	afx_msg void OnCbnSelchangeComboMovestate();
	afx_msg void OnBnClickedBtnCreatesimudata();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnBnClickedOk();
	void init_combo();
	afx_msg void OnCbnSelchangeComboChoosemode();
	afx_msg void OnBnClickedBtnAddrecombmove();
};
