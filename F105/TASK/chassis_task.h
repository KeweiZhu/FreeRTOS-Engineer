#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_
#include "main.h"

void chassis_vel_pid_init(float p, float i, float d, float i_max);
void chassis_task(void);
void chassis_cal(int vx, int vy, int vw);
void chassis_power_off(void);
void all_move(int vx, int vy);
void w_offset(float speed);
void w_turn(float speed);
void Chassis_task(void *pvParameters);
int  Chassis_mode_static(int v, int v_set);			//静步模式速度计算
#endif
