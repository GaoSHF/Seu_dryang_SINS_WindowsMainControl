
// WindowsMainControlV1.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CWindowsMainControlV1App:
// �йش����ʵ�֣������ WindowsMainControlV1.cpp
//

class CWindowsMainControlV1App : public CWinApp
{
public:
	CWindowsMainControlV1App();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CWindowsMainControlV1App theApp;