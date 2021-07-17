/************************************************************************************
CAN1控制底盘移动以及陀螺仪数据接收，ID号如下：

陀螺仪数据接收：（0x101）   B板数据接收c板：0x301,  B板向C板发送数据：0x401
***************************************************************************************/

#include "can1.h"

/*NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)*/

rmc620_t forklift_motor[2];

int lift_switch_flag;

unsigned char Data_Receive_from_F103[8];//Data_Receive存的是接收到的从C板传来的数据
unsigned char Data2C_tx[8]; //发送到c板的数据
unsigned char Laser_Left,Laser_Right,Laser_Mid;

/**
  * @brief  配置CAN1
  * @param  None
  * @retval None
  */
void CAN1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* 打开GPIO时钟、AFIO时钟，CAN时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


	/* CAN1 RX PB8 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* CAN1 TX PB9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);  // CAN1 remap

	/* CAN1 Enabling interrupt */	
	NVIC_Config(CAN1_RX0_IRQn, 0, 0);			
	NVIC_Config(CAN1_RX1_IRQn, 0, 1);	
	NVIC_Config(CAN1_TX_IRQn, 1, 0);
	
	/* CAN  BaudRate = RCC_APB1PeriphClock/(CAN_SJW+CAN_BS1+CAN_BS2)/CAN_Prescaler */
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);   

	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=ENABLE;
	CAN_InitStructure.CAN_AWUM=ENABLE;
	CAN_InitStructure.CAN_NART=ENABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=ENABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;   
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_5tq;  
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;	
	CAN_InitStructure.CAN_Prescaler=4;

	CAN_Init(CAN1,&CAN_InitStructure);	// CAN1											


	CAN_FilterInitStructure.CAN_FilterNumber=0;	 
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;	 // 标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;   // 32位过滤器
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x301 <<5;			// 过滤器标识符
	CAN_FilterInitStructure.CAN_FilterIdLow=0x302 <<5;				
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x303 <<5;		// 过滤器屏蔽标识符
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x304 <<5;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0指向过滤器
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	
	CAN_FilterInitStructure.CAN_FilterNumber=1;	 
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;	 // 标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;   // 32位过滤器
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x205 <<5;			// 过滤器标识符
	CAN_FilterInitStructure.CAN_FilterIdLow=0x206 <<5;				
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x207 <<5;		// 过滤器屏蔽标识符
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x208 <<5;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO1;	 // FIFO0指向过滤器
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
		CAN_ITConfig(CAN1,CAN_IT_FMP1,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

/**
  * @brief  CAN1接收中断 FIFO0
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
	int i;
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		
		switch(rx_message.StdId)
        {
			
			case 0x301:
				
				for(i = 0;i < 8; i++)
				{
					Data_Receive_from_F103[i] = rx_message.Data[i];
					Laser_Left = (Data_Receive_from_F103[1])&0x01;
					Laser_Right = (Data_Receive_from_F103[1]>>1)&0x01;
					Laser_Mid = (Data_Receive_from_F103[1]>>2)&0x01;
					lift_switch_flag = Data_Receive_from_F103[2];
				}
				 
				break;
			
			
			
			default:
				break;

		}
		

		
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
	}
}	





/**
  * @brief  CAN1接收中断，FIFO1
  * @param  None
  * @retval None
  */
void CAN1_RX1_IRQHandler(void)
{
	CanRxMsg rx_message;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP1)!= RESET) 
	{
		CAN_Receive(CAN1, CAN_FIFO1, &rx_message);
		
		switch(rx_message.StdId)
        {
           
			
			case 0x205:
				forklift_motor[0].angle = (rx_message.Data[0] << 8) | rx_message.Data[1];
				forklift_motor[0].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
			break;
				
			case 0x206:
				forklift_motor[1].angle = (rx_message.Data[0] << 8) | rx_message.Data[1];
				forklift_motor[1].speed = (rx_message.Data[2] << 8) | rx_message.Data[3];
			break;
				
				
			default:
				break;
		}
		
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP1);
	}
}



/**
  * @brief  CAN1发送中断
  * @param  None
  * @retval None
  */
void CAN1_TX_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}



/**
  * @brief  CAN1发送数据 从B板向C板（Master -> Slave）
  * @param  a：
            b：
            c：
            d：
  * @retval None
  */
void can1_Master2Slave(void)
{
    CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x401;
	
	Data2C_tx[0] = '!';

    tx_message.Data[0] = Data2C_tx[0];
    tx_message.Data[1] = Data2C_tx[1];
    tx_message.Data[2] = Data2C_tx[2];
    tx_message.Data[3] = Data2C_tx[3];
    tx_message.Data[4] = Data2C_tx[4];
    tx_message.Data[5] = Data2C_tx[5];
    tx_message.Data[6] = Data2C_tx[6];
    tx_message.Data[7] = Data2C_tx[7];

    CAN_Transmit(CAN1, &tx_message);
}

/**
  * @brief  B板到C板发送数据初始化
  * @param  None
  * @retval None
  */
void Data_Send_to_Slave_Init(void)
{
	Data2C_tx[0] = '!';
	Data2C_tx[1] = 0;//控制模式（1：遥控器；2：键鼠；）
	Data2C_tx[2] = 0;//运动模式（1：底盘模式；2：救援混合模式；3：自动取矿模式；4：上升模式；5：手动取矿模式；6：小资源岛取矿底盘模式;7:上层复位模式；
	//8：小资源岛标定模式；9：大资源岛空中取矿模式;10:自检模式；11：取小资源岛矿石(第一次)；12：取小资源岛矿石（第二次））；13：大资源岛空中夹取模式
	//14：大资源岛凹槽取矿；15：大资源岛凹槽夹取模式；16：救援模式；17：复活模式；18：叉车模式；19：兑换模式；20：兑换模式1；21：兑换模式2
	//22:大资源岛空中标定模式；23：大资源岛凹槽标定模式 ; 24:掉电模式）
	Data2C_tx[3] = 0;//在自动取矿模式下（9：取大资源岛凹槽矿石；10：取大资源岛空中矿石；11：取小资源岛矿石(第一次)；12：取小资源岛矿石（第二次））；在手动取矿模式下
	//（1：抬升mid；2：有杆前移；3：大前移；4:夹取）（5：降落；6：有杆收回；7：大前移收回；8：夹取松开）；在救援模式下：12：放下救援爪；13：收回
	Data2C_tx[4] = 0;//(1:一级抬升；0：一级下落)
	Data2C_tx[5] = 0;//鼠标右键
	Data2C_tx[6] = 0;
	Data2C_tx[7] = 0;
	
//	Data2C_tx[6] = rc_ctrl.key.z|rc_ctrl.key.x<<1|rc_ctrl.key.c<<2|rc_ctrl.key.v<<3|rc_ctrl.key.b<<4|rc_ctrl.key.f<<5|rc_ctrl.key.g<<6|rc_ctrl.key.ctrl<<7;
//	Data2C_tx[7] = rc_ctrl.key.a|rc_ctrl.key.d<<1|rc_ctrl.key.w<<2|rc_ctrl.key.s<<3|rc_ctrl.key.q<<4|rc_ctrl.key.e<<5|rc_ctrl.key.r<<6| 0<<7;


	
}


/**
  * @brief  CAN1发送数据,内带电流限幅 -16384 ~ 0 ~ 16384
  * @param  left：0x205电流给定
			right：0x206电流给定
  * @retval None
  */
void ForkLift_current_send(int left, int right)
{
	CanTxMsg tx_message;
	int i;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x1FF;
	
    left = LIMIT_MAX_MIN(left, 15000, -15000);
    right = LIMIT_MAX_MIN(right, 15000, -15000);
	  
    tx_message.Data[0] = (unsigned char)((left >> 8) & 0xff);
    tx_message.Data[1] = (unsigned char)(left & 0xff);  
    tx_message.Data[2] = (unsigned char)((right >> 8) & 0xff);
    tx_message.Data[3] = (unsigned char)(right & 0xff);
	for(i=0;i<4;i++)
		tx_message.Data[4+i] = 0;
		
    CAN_Transmit(CAN1, &tx_message);
}





