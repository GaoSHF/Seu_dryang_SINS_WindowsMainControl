#include "BasicNaviCal.h"
#include "Struct.h"

#pragma once 

#define INIT_A3 0

extern PHINS phins;                                         // PHINS 参考信息
extern SYS_ELEMENT infor;
extern SYS_ELEMENTtransverse inforS;
extern OUTIMU IMUout;
extern COARSE_ALGI c_infor;                                 //粗对准参数
extern COMPALIGN   cmp;
extern ADRC_S adrc;
extern NAVPARA SINSpara;
extern GPS gps;
extern CALIPMT calipara;//标定参数
extern ZTPARA ZT;
extern SKALMAN_15_3 fkalman;
extern SKALMAN_15_3 nkalman;
extern SKALMAN_16_3 kalman_dvl;          //20171128         

extern INSCAL INScal;
extern FOSN fosn;
extern SYSTEMCTRL sysc;
extern FILTER kal;//设备程序用的kalman滤波结构体对象


void init_basicnavi(void);                    //基础导航参数初始化
void init_coarsealign(void);                  //粗对准参数初始化
void init_cmp(void);                          //罗经参数初始化
void init_adrc(void);
void Kal_Init_P_15(SKALMAN_15_3& temp_kal,char mode);
<<<<<<< HEAD
void Kal_Init_P_16(SKALMAN_16_3& temp_kal);   
=======
>>>>>>> 018adce56d7b29bd6af60d98f26b9c800fbe988a
void kalinitial(void);