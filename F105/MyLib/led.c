#include "led.h"

/**
  * @brief  ∫Ï¿∂LEDµ∆≈‰÷√
  * @param  None
  * @retval None
  */
void LED_Config(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&gpio);

	GPIO_ResetBits(GPIOC,GPIO_Pin_13 | GPIO_Pin_14);   //’‚ «ø™∆Ùledµ∆
}
