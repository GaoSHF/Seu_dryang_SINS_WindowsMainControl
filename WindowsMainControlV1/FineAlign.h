#include "Init_Navigation.h"
#pragma once 

void fine_cmps(void);
void fine_adrc(void);
void fine_yucia(SKALMAN_15_3& temp_kal, double observer[3], char mode);
void navi_Kal_15_3(SKALMAN_15_3& temp_kal, double observer[3], char mode);
void navi_Kal_16_3(SKALMAN_16_3& temp_kal, double observer[3]);