#include "main.h" 


void BSP_init(void);
/*text*/
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
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	TIM2_Init();
	TIM3_Init();
	TIM4_Init();
}
