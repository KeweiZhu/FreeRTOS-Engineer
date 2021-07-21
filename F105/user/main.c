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
/*-------------------------------------------�����ʼ��-----------------------------------------------------------*/

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_2 );
	
	CAN1_Config();
	CAN2_Config();

	TIM4_Init();
	TIM6_Configration();
	TIM7_Configration();
	
/*-----------------------------------------���ܳ�ʼ��-------------------------------------------------------------*/
	LED_Config();
	USART3_Configuration();
	UART4_Configuration();

/*------------------------------------------������ʼ��---------------------------------------------------------------*/
	rc_reset();                                    		//ң������ʼ������������λ���㣩
	g_flag.gyro_use_flag = 0;                      		//�Ƿ����������ǣ�Ĭ��Ϊ��ʹ�ã�
  chassis_vel_pid_init(7.5f, 2.5f, 10.0f, 100.0f);	//����PID��ʼ��
	
	/*�����ٶȳ�ʼ�趨Ϊ0*/
	vx_set = 0;
	vy_set = 0;
	vw_set = 0;
}
