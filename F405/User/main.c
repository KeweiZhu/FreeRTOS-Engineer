#include "main.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

void System_Init(void);
void System_Configration(void);
/**
  * @brief  Main Function
  * @param  None
  * @retval None
  */
int main()
{
	System_Configration();
	System_Init();

	delay_ms(100);
	startTast();
  vTaskStartScheduler();
	
	while(1){}
} 
/**
  * @brief  Configuration 
  * @param  None
  * @retval None
  */
void System_Configration(void)
{

}

void System_Init(void)
{
   TIM4_Init();
}




/**
  * @brief  ∫¡√Î—” ±
  * @param  None
  * @retval None
  */
void delay_ms(unsigned int t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=10300;
 		while(a--);
	}
}
