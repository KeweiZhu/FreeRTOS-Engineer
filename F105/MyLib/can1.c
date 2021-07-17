/************************************************************************************
CAN1���Ƶ����ƶ��Լ����������ݽ��գ�ID�����£�

���������ݽ��գ���0x101��   B�����ݽ���c�壺0x301,  B����C�巢�����ݣ�0x401
***************************************************************************************/

#include "can1.h"

/*NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)*/

rmc620_t forklift_motor[2];

int lift_switch_flag;

unsigned char Data_Receive_from_F103[8];//Data_Receive����ǽ��յ��Ĵ�C�崫��������
unsigned char Data2C_tx[8]; //���͵�c�������
unsigned char Laser_Left,Laser_Right,Laser_Mid;

/**
  * @brief  ����CAN1
  * @param  None
  * @retval None
  */
void CAN1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/* ��GPIOʱ�ӡ�AFIOʱ�ӣ�CANʱ�� */
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
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;	 // ��ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;   // 32λ������
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x301 <<5;			// ��������ʶ��
	CAN_FilterInitStructure.CAN_FilterIdLow=0x302 <<5;				
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x303 <<5;		// ���������α�ʶ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x304 <<5;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	 // FIFO0ָ�������
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	
	CAN_FilterInitStructure.CAN_FilterNumber=1;	 
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdList;	 // ��ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;   // 32λ������
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x205 <<5;			// ��������ʶ��
	CAN_FilterInitStructure.CAN_FilterIdLow=0x206 <<5;				
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x207 <<5;		// ���������α�ʶ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x208 <<5;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO1;	 // FIFO0ָ�������
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	

    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
		CAN_ITConfig(CAN1,CAN_IT_FMP1,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

/**
  * @brief  CAN1�����ж� FIFO0
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
  * @brief  CAN1�����жϣ�FIFO1
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
  * @brief  CAN1�����ж�
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
  * @brief  CAN1�������� ��B����C�壨Master -> Slave��
  * @param  a��
            b��
            c��
            d��
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
  * @brief  B�嵽C�巢�����ݳ�ʼ��
  * @param  None
  * @retval None
  */
void Data_Send_to_Slave_Init(void)
{
	Data2C_tx[0] = '!';
	Data2C_tx[1] = 0;//����ģʽ��1��ң������2�����󣻣�
	Data2C_tx[2] = 0;//�˶�ģʽ��1������ģʽ��2����Ԯ���ģʽ��3���Զ�ȡ��ģʽ��4������ģʽ��5���ֶ�ȡ��ģʽ��6��С��Դ��ȡ�����ģʽ;7:�ϲ㸴λģʽ��
	//8��С��Դ���궨ģʽ��9������Դ������ȡ��ģʽ;10:�Լ�ģʽ��11��ȡС��Դ����ʯ(��һ��)��12��ȡС��Դ����ʯ���ڶ��Σ�����13������Դ�����м�ȡģʽ
	//14������Դ������ȡ��15������Դ�����ۼ�ȡģʽ��16����Ԯģʽ��17������ģʽ��18���泵ģʽ��19���һ�ģʽ��20���һ�ģʽ1��21���һ�ģʽ2
	//22:����Դ�����б궨ģʽ��23������Դ�����۱궨ģʽ ; 24:����ģʽ��
	Data2C_tx[3] = 0;//���Զ�ȡ��ģʽ�£�9��ȡ����Դ�����ۿ�ʯ��10��ȡ����Դ�����п�ʯ��11��ȡС��Դ����ʯ(��һ��)��12��ȡС��Դ����ʯ���ڶ��Σ��������ֶ�ȡ��ģʽ��
	//��1��̧��mid��2���и�ǰ�ƣ�3����ǰ�ƣ�4:��ȡ����5�����䣻6���и��ջأ�7����ǰ���ջأ�8����ȡ�ɿ������ھ�Ԯģʽ�£�12�����¾�Ԯצ��13���ջ�
	Data2C_tx[4] = 0;//(1:һ��̧����0��һ������)
	Data2C_tx[5] = 0;//����Ҽ�
	Data2C_tx[6] = 0;
	Data2C_tx[7] = 0;
	
//	Data2C_tx[6] = rc_ctrl.key.z|rc_ctrl.key.x<<1|rc_ctrl.key.c<<2|rc_ctrl.key.v<<3|rc_ctrl.key.b<<4|rc_ctrl.key.f<<5|rc_ctrl.key.g<<6|rc_ctrl.key.ctrl<<7;
//	Data2C_tx[7] = rc_ctrl.key.a|rc_ctrl.key.d<<1|rc_ctrl.key.w<<2|rc_ctrl.key.s<<3|rc_ctrl.key.q<<4|rc_ctrl.key.e<<5|rc_ctrl.key.r<<6| 0<<7;


	
}


/**
  * @brief  CAN1��������,�ڴ������޷� -16384 ~ 0 ~ 16384
  * @param  left��0x205��������
			right��0x206��������
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





