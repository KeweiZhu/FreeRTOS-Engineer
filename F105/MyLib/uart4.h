#ifndef _UART4_H
#define _UART4_H
#include "main.h"

#define JudgeBufBiggestSize 45	//接收数据最大长度
#define SEND_MAX_SIZE    128    //上传数据最大的长度

void UART4_Configuration(void);
void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen);

#endif
