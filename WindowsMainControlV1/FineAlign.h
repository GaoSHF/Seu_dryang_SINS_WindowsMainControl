#include "Init_Navigation.h"
#pragma once 

void fine_cmps(void);
void fine_adrc(void);
void fine_yucia(SKALMAN_15_3& temp_kal, double observer[3], char mode);
void navi_Kal_15_3(SKALMAN_15_3& temp_kal, double observer[3], char mode);
void navi_Kal_16_3(SKALMAN_16_3& temp_kal, double observer[3]);
void navi_Kal_16_3_transverse(SKALMAN_16_3& temp_kal, double observer[3]);
void navi_DVL_Cmp_Depth(SKALMAN_19_6& dvlkalman, SKALMAN_15_1& cmpkalman, SKALMAN_15_1& depkalman, SKALMAN_15_3& zupkalman, SDVLCmpDepth& dObserver);   //20180116