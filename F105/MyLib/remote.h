#ifndef _REMOTE_H_
#define _REMOTE_H_
#include "main.h"

/***********RC***********/
typedef struct{
	struct
	{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned char s1;
		unsigned char s2;
	}rc;
	struct
	{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	struct
	{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
	}key;
    
}rc_ctrl_t;

void remote_receive(volatile unsigned char rx_buffer[]);
void rc_reset(void);
void key_refresh(void);
void key_rising_check(void);

extern rc_ctrl_t rc_ctrl, rc_ctrl_last;

#endif
