#ifndef __NVIC_H__
#define __NVIC_H__
#include "main.h"

void NVIC_Config(uint8_t NVIC_IRQChannel,uint8_t priority,uint8_t SubPriority);

#endif
