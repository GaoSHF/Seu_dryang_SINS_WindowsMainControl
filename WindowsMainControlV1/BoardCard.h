////�忨���////
///������ر����Ķ���ͳ�ʼ������

#include "aecmfc4b_lib.h"
#pragma comment(lib,"WS2_32.lib") 

///ͨ��0Ϊת̨���գ�1ΪGPS���գ�2ΪIMU����


//�忨ͨ����Ŀ
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
bool Init_Card(HWND tempHwnd, char mode, BYTE channel,char var);//mode,0x0?,ΪFOSNģʽѡ��0Ϊ��FOSN��1Ϊ��FOSN;channelΪ��ʼ����ͨ����varΪ���ñ���

void Init_CardVar()
{
	btCardID=0;
	TailA[0]=0;TailA[1]=0;TailA[2]=0;TailA[3]=0;
	TailB[0]=0;TailB[1]=0;TailB[2]=0;TailB[3]=0;
	TimeoutT[0]=0;TimeoutT[1]=0;TimeoutT[2]=0;TimeoutT[3]=0;
	TimeOutEn[0]=0;TimeOutEn[1]=0;TimeOutEn[2]=0;TimeOutEn[3]=0;

}
bool Init_Card(HWND tempHwnd,char mode =0x00, BYTE channel=0x0f, char var=0x00)
{
	hCard = NULL; // handle to the card
	btCardID = 0; // card's ID
	int i;

	for (i=0; i<CH_RMAX; i++)
	 {
		 if(0==((channel>>i)&0x01))
			 continue;
		 // data format of channel 1~4
		     
		 stdf[i].DataLength = 8; // data bits
		 stdf[i].ParityEnable = FALSE; // parity enabled bits
		 stdf[i].EvenParity = FALSE; // parity selection
		 stdf[i].StopBits = 1; // 1 stop bits
		 
		 // frame format fo channel 1~4
		 
		 TailA[i] = 0x00;
		 TailB[i] = 0x00;
		 // receive mode of channel 1~4
		 strm[i].IsProtocol = TRUE; //FALSE transparent		 
		 strm[i].SumCheckEN = FALSE;
		 strm[i].HeadIncluded = FALSE;
		 
		 
		 wdWordInterval[i] = 0; // word interval default to 0 ms
		 
		 TimeOutEn[i] = TRUE;
		 TimeoutT[i] = 100;
	 }
	// 
	stdf[0].BaudRate = 614400;//default;
	strm[0].ProtocolSel = 1;
	stff[0].HDR = 0xFF;
	stff[0].EDR = 0xFF;
	stff[0].LENR = 10;
	wmode[0] = 1;//422

	stdf[1].BaudRate = 460800;//1843200; // bits rate  //ת̨����
	strm[1].ProtocolSel = 1;
	stff[1].HDR = 0xFF;
	stff[1].EDR = 0xFF;
	stff[1].LENR = 13;
	wmode[1] = 1;//422


	stdf[2].BaudRate = 614400;//FOSN
	strm[2].ProtocolSel = 0;
	stff[2].HDR = 0x99;
	stff[2].EDR = 0x66;
	stff[2].LENR = 10;
	wmode[2] = 1;

	stdf[3].BaudRate = 115200;// bits rate// GPS���ڣ�bestposb
	stff[3].HDR = 0xAA;
	stff[3].EDR = 0x44;
	wmode[3] = 0;//232	 
	strm[3].ProtocolSel = 1;
	// strm[3].IsProtocol = TRUE; 
	stff[3].LENR = 177;//bestvel+bestpos-head=76+104-3 

	if (mode == 0x01)
	{
		//var��λΪ�忨ͨ���ţ���λλFOSN��com�ں�
		int cn = (var & 0xf0) >> 4;
		int com = (var & 0x0f) ;
		stdf[cn].BaudRate = 614400;//FOSN_new  0,1-614400//2-921600
		if (com == 2) stdf[cn].BaudRate = 921600;
		strm[cn].ProtocolSel = 0;
		stff[cn].HDR = 0x55;
		stff[cn].EDR = 0xaa;
		stff[cn].LENR = 154;//0-154,1/2-28
		if (com == 2|| com == 1) stff[cn].LENR = 28;
		wmode[cn] = 1;
	}



		 
#ifndef CARD_DEBUG
	//  Open the card and setting up it
	//
	if (!AECMFC4B_Open(&hCard, btCardID))
	{
		MessageBox(NULL,TEXT("�����Ҳ����忨,�����˳�!"), TEXT("����"), MB_OK | MB_ICONERROR);
		ExitProcess(0);
		return FALSE; // quit program
	}
	if (hCard==NULL)
	{
		MessageBox(NULL,TEXT("�����Ҳ����忨,�����˳�!\n\n(�忨�����ȡʧ��)"), TEXT("����"), MB_OK | MB_ICONERROR);
		ExitProcess(0);
		return FALSE; // quit program
	}

	//  setting up the card to default
	//
	if (!Sio_Reset(hCard)) // reset all FIFO
	{
		MessageBox(NULL,TEXT("��λ�忨ʧ��,�����˳�����!"), TEXT("����"), MB_OK | MB_ICONERROR);
		if (!AECMFC4B_Close(hCard))
		{
			MessageBox(NULL,TEXT("�رհ忨ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
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
			MessageBox(NULL,TEXT("ȡ��485ģʽʹ�ܳ���!"), TEXT("����"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("�رհ忨ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
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
			MessageBox(NULL,TEXT("���ð忨����ģʽ����!"), TEXT("����"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("�رհ忨ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
			}
			ExitProcess(0);
			return FALSE; // quit program
		}
		if (wmode[i] == 2)
		{
			if (!Sio_485BusEnable(hCard, i, TRUE))
			{
				MessageBox(NULL,TEXT("485ģʽʹ�ܳ���!"), TEXT("����"), MB_OK | MB_ICONERROR);
				if (!AECMFC4B_Close(hCard))
				{
					MessageBox(NULL,TEXT("�رհ忨ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
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
			MessageBox(NULL,TEXT("���ý���ģʽʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("�رհ忨ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
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
			MessageBox(NULL,TEXT("����ͨѶ���ݸ�ʽʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("�رհ忨ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
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
			MessageBox(NULL,TEXT("����ͨѶ����֡��ʽʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("�رհ忨ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
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
			MessageBox(NULL,TEXT("���÷����ּ���ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
			if (!AECMFC4B_Close(hCard))
			{
				MessageBox(NULL,TEXT("�رհ忨ʧ��!"), TEXT("����"), MB_OK | MB_ICONERROR);
			}
			ExitProcess(0);
			return FALSE; // quit program
		}
	}

#endif

	memset(AppPath, 0, sizeof(AppPath));

	if (GetCurrentDirectory(sizeof(AppPath), LPWSTR(AppPath))==0)
	{
		MessageBox(NULL,TEXT("���ػ���������Ϣʧ��!"), TEXT("����"), MB_OK);
		return 0;
	}
	return 1;
}