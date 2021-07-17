#ifndef __MAIN_H_
#define __MAIN_H_

/*Library*/
#include "stm32f10x.h"
#include "misc.h"
#include "string.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_i2c.h"

/*Mylib*/
#include "can1.h"
#include "can2.h"
#include "led.h"
	//#include "usart.h"
#include "delay.h"
#include "drv_dma.h"
#include "nvic.h"
#include "remote.h"
#include "tim.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
#include "uart4.h"
#include "uart5.h"
#include "JudgeSend.h"

/*Algorithm*/
#include "pid.h"
#include "algorithmOfCRC.h"

/*TASK*/
#include "start_task.h"
#include "state_task.h"
#include "chassis_task.h"

/*FreeRTOS*/
#include "FreeRTOS.h"
#include "task.h"

/*******************宏定义*************************/
#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

//控制模式
#define RC_MODE     					0X01		//遥控模式
#define KEY_MODE    					0X02		//键鼠模式
#define POWER_OFF_MODE 				0X03		//掉电模式
//地盘模式
#define CHASSIS_MODE					0X04		//底盘运动模式
#define CHASSIS_MODE_STATIC		0X05		//底盘静步模式
#define CHASSIS_MODE3					0X06		//底盘模式3
//上层模式
#define SENIOR_UP_MODE				0X07		//上层上升模式
#define SENIOR_MODE2					0X08		//上层模式2
#define SENIOR_MODE3					0X09		//上层模式3

//#define RC_MODE     		0X01		//遥控模式
//#define KEY_MODE    		0X02		//键鼠模式
//#define GET_BULLET_MODE 	0X03		//自动取弹模式
//#define LANDING_ON_MODE 	0X04		//自动登岛模式
//#define POWER_OFF_MODE 		0X05		//掉电模式
//#define INITIALIZE_MODE     0X06        //所有装置回位
//#define MANUAL_LAND_MODE   0x07     //手动控制登岛
//#define MANUAL_GET_BULLET_MODE  0x08  //手动取弹模式
//#define LIFT_UP_MODE    0x09//上层上升模式
//#define SAVE_MODE       0x10//救援模式
//#define GET_BULLET_MODE1 	0X13		//自动取弹模式
//#define CHOOSE_MODE       0x14//选择模式

//#define MAGAZINE_TARGET		0X10		//控制上层
//#define CHASSIS_TARGET 		0X11		//控制底盘移动 
//#define EXPAND_TARGET 		0x12		//控制底盘升降
//#define RESCUE_TARGET		0X13		//控制救援机械手

#define LIFT_UP           1
#define LIFT_DOWN         2

//IO??????
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO?????
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n) 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n) 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n) 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n) 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n) 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n) 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n) 

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n) 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  

#define PIout(n)   BIT_ADDR(GPIOI_ODR_Addr,n)  
#define PIin(n)    BIT_ADDR(GPIOI_IDR_Addr,n)  

#define YAW_INFROUT      ServoGimbal_yaw(5000)
#define YAW_BEHIND       ServoGimbal_yaw(1520); 
#define PITCH_DOWN       ServoGimbal_pitch(2700); 
#define PITCH_UP         ServoGimbal_pitch(5800); 
/*******************结构体定义*************************/
typedef union
{
	float fdata;			//4?
	unsigned long idata;
}
FloatLongType;

typedef union
{
	float fdata;			//4?
	unsigned char idata[4];
}
FloatCharType;

typedef struct
{
    s16 angle;					//角度返回值
	s16 last_angle;				//前一次角度返回值
	s16 cycle;					//圈数
	s32 real_posion;			//实际位置
    s16 speed;					//实时速度
	s16 flow;					//实时电流
	s16 temperature;			//实时温度
}rmc620_t;

typedef struct
{
	unsigned short angle;  //0~8191(0x1fff)
	short speed;       //     RPM
	short last_angle;
	s16 cycle;
	s16 real_posion;
	short Disconnect;
}rmc610_t;

typedef struct
{
	uint8_t RobotID;
	uint8_t RobotLevel;
	uint8_t bulletFreq;
	uint16_t remainHP;
	uint16_t shooterHeat17;
	uint16_t shooterHeat42;
	float bulletSpeed;
	float realChassisOutV;
	float realChassisOutA;
	float realChassispower;
	float remainEnergy;       //剩余能量
}tGameInfo;

typedef enum
{
	STATE0 = 0, STATE1 , STATE2 , STATE3 , STATE4 , STATE5 , STATE6 , STATE7 , STATE8 , STATE9 , STATE10 , STATE11 ,STATE12
}landing_state_e;
typedef enum
{
	state0 = 0, state1 , state2 , state3 , state4 , state5, state6 , state7
}initialize_state_e;
typedef enum
{
  b_state0 = 0, b_state1 , b_state2 , b_state3 , b_state4 , b_state5 , b_state6 , b_state7
}get_bullet_state_e;
typedef enum
{
  g_state0 = 0, g_state1 , g_state2 , g_state3 , g_state4 , g_state5 , g_state6 , g_state7,g_state8,g_state9
}get_bullet_state_e1;

typedef struct
{
	u8 control_mode;           //控制模式
	u8 control_target;         //控制目标
	
	u8 can1_connect;
	u32 can1_cnt;
	
	u8 can2_connect;
	u32 can2_cnt;
	
	u8 rc_connect;
	u32 rc_cnt;
	
	u8 gyro_connect;
	u32 gyro_cnt;
	u8 gyro_use_flag;         //是否启用陀螺仪
	
	u8 initial_flag;
	u8 offline_flag;
	
	landing_state_e landing_state;         //自动登岛状态标志
  initialize_state_e initialize_state;    //初始化状态
	get_bullet_state_e get_bullet_state;    //自动取弹状态
	get_bullet_state_e1 get_bullet_state1;  //自动取弹状态
	
	u8 lift_state;
	u8 lift_finish;
	u8 lift_flag;
	u8 gyro_initial_flag;
}flag_t;


void full_bullet(void);
void initialize(void);
void expend_text(void);
void Change_servo(void);
void Align_step(void);
void turn_90(void);
void Save_control(void);
void viewpoint_save_switch(void);
void viewpoint_liftup_switch(void);
void viewpoint_key_switch(void);
/*******************全局引用数据*************************/
extern flag_t g_flag;
extern int cur_error_num;				//断线错误号 

#endif



