#include "main.h"

flag_t g_flag;
extern int vx_set, vy_set, vw_set;
void BSP_init(void);

int main(void)
{
   BSP_init();
   delay_ms(100);
	 startTast();
   vTaskStartScheduler();
   while (1)
   {
        ;
   }
}


void BSP_init(void)
{
/*-------------------------------------------外设初始化-----------------------------------------------------------*/

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	
	CAN1_Config();
	CAN2_Config();

	TIM4_Init();
	TIM6_Configration();
	TIM7_Configration();
	
/*-----------------------------------------功能初始化-------------------------------------------------------------*/
	LED_Config();
	USART3_Configuration();
	UART4_Configuration();

/*------------------------------------------参数初始化---------------------------------------------------------------*/
	rc_reset();                                    		//遥控器初始化函数（所有位清零）
	g_flag.gyro_use_flag = 0;                      		//是否启用陀螺仪（默认为不使用）
  chassis_vel_pid_init(7.5f, 2.5f, 10.0f, 100.0f);	//底盘PID初始化
	
	/*底盘速度初始设定为0*/
	vx_set = 0;
	vy_set = 0;
	vw_set = 0;
}
