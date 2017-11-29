#include "BasicNaviCal.h"
#include "Struct.h"

#pragma once 

#define INIT_A3 0

extern PHINS phins;                                         // PHINS �ο���Ϣ
extern SYS_ELEMENT infor;
extern SYS_ELEMENTtransverse inforS;
extern OUTIMU IMUout;
extern COARSE_ALGI c_infor;                                 //�ֶ�׼����
extern COMPALIGN   cmp;
extern ADRC_S adrc;
extern NAVPARA SINSpara;
extern GPS gps;
extern CALIPMT calipara;//�궨����
extern ZTPARA ZT;
extern SKALMAN_15_3 fkalman;
extern SKALMAN_15_3 nkalman;
extern SKALMAN_16_3 kalman_dvl;          //20171128         

extern INSCAL INScal;
extern FOSN fosn;
extern SYSTEMCTRL sysc;
extern FILTER kal;//�豸�����õ�kalman�˲��ṹ�����


void init_basicnavi(void);                    //��������������ʼ��
void init_coarsealign(void);                  //�ֶ�׼������ʼ��
void init_cmp(void);                          //�޾�������ʼ��
void init_adrc(void);
void Kal_Init_P_15(SKALMAN_15_3& temp_kal,char mode);
<<<<<<< HEAD
void Kal_Init_P_16(SKALMAN_16_3& temp_kal);   
=======
>>>>>>> 018adce56d7b29bd6af60d98f26b9c800fbe988a
void kalinitial(void);