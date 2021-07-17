#ifndef _CAN1_H_
#define _CAN1_H_
#include "main.h"

void CAN1_Config(void);
void can1_Master2Slave(void);
void Data_Send_to_Slave_Init(void);
void ForkLift_current_send(int left, int right);
#endif
