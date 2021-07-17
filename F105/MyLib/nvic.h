#ifndef __NVIC_H__
#define __NVIC_H__
#include "stm32f10x.h"
#include "misc.h"

void NVIC_Config(uint8_t NVIC_IRQChannel,uint8_t priority,uint8_t SubPriority);

#endif
