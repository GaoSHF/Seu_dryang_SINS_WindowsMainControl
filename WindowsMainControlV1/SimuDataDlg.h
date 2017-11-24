#include "Struct.h"
#include "CreateRoute.h"
#pragma once


// SimuDataDlg �Ի���

class SimuDataDlg : public CDialogEx
{
	DECLARE_DYNAMIC(SimuDataDlg)

public:
	SimuDataDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~SimuDataDlg();

// �Ի�������
	enum { IDD = IDD_DLG_SimuData };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	CALIPMT err_pmt;//������
	IMUOUT IMUSimuout;
	Route route_show;//�ؼ���ʾ�ı�����������ÿ���ؼ�����������,ֻ��ҡ�ڵĵ�0��wt0��at0��wt0������ٶȻ����ٶȣ�at0������ٶȻ���ʱ��
	Route route[50];
	int routenum;
	int ordernum;//��ʾ�����г���
	bool justaddswing;//��һ��Ҳ����ӵ�ҡ��
	CString filename;
	FILE * Iputfid;
	CComboBox SD_Combo_turn, SD_Combo_AxisXmode, SD_Combo_AxisYmode, SD_Combo_AxisZmode, SD_Combo_AngularAxis, SD_Combo_PositionAxis;
	CListBox SD_list_Moveorder;
	double lati, longi, high;
	bool creat_swing;//����ҡ�ڳɹ�
	bool is_combomove;//�Ƿ��Ǹ����˶�
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
