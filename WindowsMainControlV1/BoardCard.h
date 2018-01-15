////板卡相关////
///包含相关变量的定义和初始化函数

#include "aecmfc4b_lib.h"
#pragma comment(lib,"WS2_32.lib") 

///通道0为转台接收，1为GPS接收，2为IMU接收

//板卡通道数目
#define CH_TMAX	4
#define CH_RMAX	4

extern HANDLE hCard=NULL; // handle to the card
BYTE btCardID; // card's ID
DATAFORMAT_STRUCT stdf[4]; // data format of channel 1~4
FRAMEFORMAT_STRUCT stff[4]; // frame format fo channel 1~4
REVMODE_STRUCT strm[4]; // receive mode of channel 1~4
BYTE wmode[4];
WORD wdWordInterval[4];
BYTE TailA[4],TailB[4];
WORD TimeoutT[4];
BOOL TimeOutEn[4];
char AppPath[260];


void Init_CardVar();
bool Init_Card(HWND tempHwnd, char navigationmodel, BYTE channel);

void Init_CardVar()
{
	btCardID=0;
	TailA[0]=0;TailA[1]=0;TailA[2]=0;TailA[3]=0;
	TailB[0]=0;TailB[1]=0;TailB[2]=0;TailB[3]=0;
	TimeoutT[0]=0;TimeoutT[1]=0;TimeoutT[2]=0;TimeoutT[3]=0;
	TimeOutEn[0]=0;TimeOutEn[1]=0;TimeOutEn[2]=0;TimeOutEn[3]=0;

}
bool Init_Card(HWND tempHwnd,char navigationmodel=2, BYTE channel=0x0f)
{
	hCard = NULL; // handle to the card
	btCardID = 0; // card's ID
	int i;

	for (i=0; i<CH_RMAX; i++)
	 {
		 if(0==((channel>>i)&0x01))
			 continue;
		 // data format of channel 1~4
		 stdf[i].BaudRate = 614400;//1843200; // bits rate       
		 stdf[i].DataLength = 8; // data bits
		 stdf[i].ParityEnable = FALSE; // parity enabled bits
		 stdf[i].EvenParity = FALSE; // parity selection
		 stdf[i].StopBits = 1; // 1 stop bits
		 
		 // frame format fo channel 1~4
		 stff[i].HDR = 0x99;
		 stff[i].EDR = 0x66;
		 TailA[i] = 0x00;
		 TailB[i] = 0x00;
		 stff[i].LENR = 42;
		 
		 // receive mode of channel 1~4
		 strm[i].IsProtocol = TRUE; //FALSE transparent
		 strm[i].ProtocolSel = 0;
		 strm[i].SumCheckEN = FALSE;
		 strm[i].HeadIncluded = FALSE;
		 
		 // work mode sel
		 wmode[i] = 1;//0; // default to 232
		 
		 wdWordInterval[i] = 0; // word interval default to 0 ms
		 
		 TimeOutEn[i] = TRUE;
		 TimeoutT[i] = 100;
	 }
	 // 
	 stdf[0].BaudRate = 460800;//1843200; // bits rate  //转台串口
	 strm[0].ProtocolSel = 1;
	 stff[0].HDR = 0xFF;
	 stff[0].EDR = 0xFF;
	 stff[0].LENR = 13;
	 wmode[0] = 1;//422

	 stdf[3].BaudRate = 115200;// bits rate// GPS串口，bestposb
	 stff[3].HDR = 0xAA;
	 stff[3].EDR = 0x44;
	 wmode[3] = 0;//232	 
 	 strm[3].ProtocolSel = 1;
	 strm[3].IsProtocol = TRUE; // transparent
	 stff[3].LENR =177;//bestvel+bestpos-head=76+104-3 

		 
#ifndef CARD_DEBUG
	//  Open the card and setting up it
	//
	if (!AECMFC4B_Open(&hCard, btCardID))
	{
		MessageBox(NULL,TEXT("程序找不到板卡,即将退出!"), TEXT("错误"), MB_OK | MB_ICONERROR);
		ExitProcess(0);
		return FALSE; // quit program
	}
	if (hCard==NULL)
	{
		MessageBox(NULL,TEXT("程序找不到板卡,即将退出!\n\n(板卡句柄获取失败)"), TEXT("错误"), MB_OK | MB_ICONERROR);
		ExitProcess(0);
		return FALSE; // quit program
	}

	//  setting up the card to default
	//
	if (!Sio_Reset(hCard)) // reset all FIFO
	{
		MessageBox(NULL,TEXT("复位板卡失败,即将退出程序!"), TEXT("错误"), MB_OK | MB_ICONERROR);
		if (!AECMFC4B_Close(hCard))
		{
			MessageBox(NULL,TEXT("关闭板卡失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
		}
		ExitProcess(0);
		return FALSE; // quit program
	}

	for (i=0; i<CH_TMAX; i++)
	{
		 if(0==((channel>>i)&0x01))
			 continue;
		// enable 485
		if (!Sio_485BusEnable(hCard, i, FALSE))
		{
			MessageBox(NULL,TEXT("取消485模式使能出错!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("关闭板卡失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			}
			ExitProcess(0);
			return FALSE; // quit program
		}
	}

	//	configure the work mode
	//
	for (i=0; i<CH_TMAX; i++)
	{
		if(0==((channel>>i)&0x01))
			 continue;
		if (!Sio_SetWorkMode(hCard, i, wmode[i]))
		{
			MessageBox(NULL,TEXT("设置板卡工作模式出错!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("关闭板卡失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			}
			ExitProcess(0);
			return FALSE; // quit program
		}
		if (wmode[i] == 2)
		{
			if (!Sio_485BusEnable(hCard, i, TRUE))
			{
				MessageBox(NULL,TEXT("485模式使能出错!"), TEXT("错误"), MB_OK | MB_ICONERROR);
				if (!AECMFC4B_Close(hCard))
				{
					MessageBox(NULL,TEXT("关闭板卡失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
				}
				ExitProcess(0);
				return FALSE; // quit program
			}
		}
	}

	//	settting up Rx Mode
	//
	for (i=0; i<CH_TMAX; i++)
	{
		if(0==((channel>>i)&0x01))
			 continue;
		if (!Sio_SetRevMode(hCard, i, &(strm[i])))
		{
			MessageBox(NULL,TEXT("设置接收模式失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("关闭板卡失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			}
			ExitProcess(0);
			return FALSE; // quit program
		}
	}

	//	configuring Data fromat
	//
	for (i=0; i<CH_TMAX; i++)
	{
		if(0==((channel>>i)&0x01))
			 continue;
		if (!Sio_SetDataFormat(hCard, i, &stdf[i]))
		{
			MessageBox(NULL,TEXT("设置通讯数据格式失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("关闭板卡失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			}
			ExitProcess(0);
			return FALSE; // quit program
		}
	}

	//	configuring frame fromat
	//
	for (i=0; i<CH_TMAX; i++)
	{
		if(0==((channel>>i)&0x01))
			 continue;
		if (!Sio_SetFrameFormat(hCard, i, &stff[i]))
		{
			MessageBox(NULL,TEXT("设置通讯数据帧格式失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("关闭板卡失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			}
			ExitProcess(0);
			return FALSE; // quit program
		}
	}

	for (i=0; i<CH_TMAX; i++)
	{
		if(0==((channel>>i)&0x01))
			 continue;
		if (!Sio_Tx_WordInterval (hCard, i, wdWordInterval[i]))
		{
			MessageBox(NULL,TEXT("设置发送字间间隔失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("关闭板卡失败!"), TEXT("错误"), MB_OK | MB_ICONERROR);
			}
			ExitProcess(0);
			return FALSE; // quit program
		}
	}

#endif

	memset(AppPath, 0, sizeof(AppPath));

	if (GetCurrentDirectory(sizeof(AppPath), LPWSTR(AppPath))==0)
	{
		MessageBox(NULL,TEXT("加载环境配置信息失败!"), TEXT("错误"), MB_OK);
		return 0;
	}
	return 1;
}