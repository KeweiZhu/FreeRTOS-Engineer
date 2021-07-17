#ifndef _CAN2_H_
#define _CAN2_H_
#include "main.h"

void CAN2_Config(void);
float get_yaw_angle(void);
float get_gz(void);
void chassis_current_send(int a, int b, int c, int d);
//void track_current_send(int left, int right);

#endif


