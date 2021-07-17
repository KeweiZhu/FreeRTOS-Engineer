#include "usart.h"

#define U2_TX_LEN   11
#define JudgeBufBiggestSize 34

typedef union
{
	float fdata;
	unsigned long idata;
}
FloatlongType;


extern key_control key;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
uint8_t uart1_RxBuffer[8];
uint8_t uart1_TxBuffer[8];
uint8_t uart2_RxBuffer[7];
uint8_t uart2_TxBuffer[U2_TX_LEN];
uint8_t uart3_RxBuffer[18];

unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
unsigned char JudegeSend[28];
unsigned char SaveBuffer[68];
FloatlongType Chassis_a,Chassis_v,Chassis_energy;
FloatlongType UserMes;
tGameInfo JudgeReceive;
short DataLen;

/*
 * USART1 -- TX->PA9 RX->PA10
 * USART2 -- TX->PA2 RX->PA3
 * USART3 -- RX->PB11
 * UART4 --- TX->PC10 RX->PC11
 * UART5 --- TX->PC12 RX->PD2
 * */
void USART_Config(void)
{
 	huart1.UART_BASEx = USART1;
	huart2.UART_BASEx = USART2;
	huart3.UART_BASEx = USART3;
	huart4.UART_BASEx = UART4;
	huart5.UART_BASEx = UART5;
	
	{//tim
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3|RCC_APB1Periph_UART4|RCC_APB1Periph_UART5,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	}
	{//GPIO
		UART_GPIO_RX_Config(&huart1,GPIO_Pin_10,GPIOA);
		UART_GPIO_TX_Config(&huart1,GPIO_Pin_9,GPIOA);
		UART_GPIO_RX_Config(&huart2,GPIO_Pin_3,GPIOA);
		UART_GPIO_TX_Config(&huart2,GPIO_Pin_2,GPIOA);
		UART_GPIO_RX_Config(&huart3,GPIO_Pin_11,GPIOB);
		UART_GPIO_RX_Config(&huart4,GPIO_Pin_11,GPIOC);
		UART_GPIO_TX_Config(&huart4,GPIO_Pin_10,GPIOC);
		UART_GPIO_RX_Config(&huart5,GPIO_Pin_12,GPIOC);
		UART_GPIO_TX_Config(&huart5,GPIO_Pin_2,GPIOD);
	}
	{//USART
		UART_Config(&huart1, 115200,8,USART_StopBits_1,USART_Parity_No,USART_Mode_Tx|USART_Mode_Rx);
		UART_Config(&huart2,115200,8,USART_StopBits_1,USART_Parity_No,USART_Mode_Tx|USART_Mode_Rx);
		UART_Config(&huart3,100000,8,USART_StopBits_1,USART_Parity_Even,USART_Mode_Rx);
		UART_Config(&huart4,115200,8,USART_StopBits_1,USART_Parity_No,USART_Mode_Tx|USART_Mode_Rx);
//		UART_Config(&huart5,115200 ,8,USART_StopBits_1,USART_Parity_Even,USART_Mode_Tx|USART_Mode_Rx);

		USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
		USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
		USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
		USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
		USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);
		
		USART_Cmd(USART1, ENABLE);
		USART_Cmd(USART2, ENABLE);
		USART_Cmd(USART3, ENABLE);
		USART_Cmd(UART4, ENABLE);
		
		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
		USART_DMACmd(USART2,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
		USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
		USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
	}
	{//NVIC
		NVIC_Config(USART1_IRQn, 6, 0);
		NVIC_Config(USART2_IRQn, 7, 0);
		NVIC_Config(USART3_IRQn, 5, 0);
		NVIC_Config(UART4_IRQn, 8, 0);
		NVIC_Config(UART5_IRQn, 9, 0);
	}
	{//DMA
		DMA_Config(DMA1_Channel5,(uint32_t)&huart1.UART_BASEx->DR,(uint32_t)uart1_RxBuffer,DMA_DIR_PeripheralSRC,JudgeBufBiggestSize,
		           DMA_PeripheralInc_Disable,DMA_MemoryInc_Enable,DMA_PeripheralDataSize_Byte,DMA_MemoryDataSize_Byte,
                   DMA_Mode_Circular,DMA_Priority_VeryHigh,DMA_M2M_Disable);
	  DMA_Config(DMA1_Channel4,(uint32_t)&huart1.UART_BASEx->DR,(uint32_t)uart1_TxBuffer,DMA_DIR_PeripheralDST,28,
		           DMA_PeripheralInc_Disable,DMA_MemoryInc_Enable,DMA_PeripheralDataSize_Byte,DMA_MemoryDataSize_Byte,
                   DMA_Mode_Normal,DMA_Priority_VeryHigh,DMA_M2M_Disable);
		DMA_Config(DMA1_Channel6,(uint32_t)&huart2.UART_BASEx->DR,(uint32_t)uart2_RxBuffer,DMA_DIR_PeripheralSRC,8,
		           DMA_PeripheralInc_Disable,DMA_MemoryInc_Enable,DMA_PeripheralDataSize_Byte,DMA_MemoryDataSize_Byte,
                   DMA_Mode_Circular,DMA_Priority_VeryHigh,DMA_M2M_Disable);
		DMA_Config(DMA1_Channel7,(uint32_t)&huart2.UART_BASEx->DR,(uint32_t)uart2_TxBuffer,DMA_DIR_PeripheralDST,U2_TX_LEN,
		           DMA_PeripheralInc_Disable,DMA_MemoryInc_Enable,DMA_PeripheralDataSize_Byte,DMA_MemoryDataSize_Byte,
                   DMA_Mode_Normal,DMA_Priority_VeryHigh,DMA_M2M_Disable);
		DMA_Config(DMA1_Channel3,(uint32_t)&huart3.UART_BASEx->DR,(uint32_t)uart3_RxBuffer,DMA_DIR_PeripheralSRC,30,
		           DMA_PeripheralInc_Disable,DMA_MemoryInc_Enable,DMA_PeripheralDataSize_Byte,DMA_MemoryDataSize_Byte,
                   DMA_Mode_Circular,DMA_Priority_VeryHigh,DMA_M2M_Disable);
		DMA_Config(DMA2_Channel3,(uint32_t)&huart4.UART_BASEx->DR,(uint32_t)JudgeReceiveBuffer,DMA_DIR_PeripheralSRC,JudgeBufBiggestSize,
		           DMA_PeripheralInc_Disable,DMA_MemoryInc_Enable,DMA_PeripheralDataSize_Byte,DMA_MemoryDataSize_Byte,
                   DMA_Mode_Circular,DMA_Priority_VeryHigh,DMA_M2M_Disable);
	  DMA_Config(DMA2_Channel5,(uint32_t)&huart4.UART_BASEx->DR,(uint32_t)JudegeSend,DMA_DIR_PeripheralDST,28,
		           DMA_PeripheralInc_Disable,DMA_MemoryInc_Enable,DMA_PeripheralDataSize_Byte,DMA_MemoryDataSize_Byte,
                   DMA_Mode_Normal,DMA_Priority_VeryHigh,DMA_M2M_Disable);
		//U1 TX
		DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel4,DISABLE);    
		NVIC_Config(DMA1_Channel4_IRQn, 7, 2);
		//U1 RX
		DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel5,ENABLE);    
		NVIC_Config(DMA1_Channel5_IRQn, 7, 2);
		//U2 TX
		DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel7,DISABLE);    
		NVIC_Config(DMA1_Channel7_IRQn, 7, 0);
		//U2 RX
		DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel6,ENABLE);
		NVIC_Config(DMA1_Channel6_IRQn, 7, 0);
		//U3 RX
		DMA_Cmd(DMA1_Channel3,ENABLE);   
		//U4 TX
		DMA_ITConfig(DMA2_Channel5,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA2_Channel5,DISABLE);    
		NVIC_Config(DMA2_Channel5_IRQn, 7, 2);
		//U4 RX
		DMA_ITConfig(DMA2_Channel3,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA2_Channel3,ENABLE);    
		NVIC_Config(DMA2_Channel3_IRQn, 1, 0);		
	}
}
/**
  * @brief  Initializes the USARTx peripheral 
  * @param  USART_BaudRate: 
  *         @arg 9600
  *         @arg 115200
  *         @arg 100000
  * @param  USART_WordLength: 
  *         @arg USART_WordLength_8b                 
  *         @arg USART_WordLength_9b                
  * @param  USART_StopBits: 
  *         @arg USART_StopBits_1                     
  *         @arg USART_StopBits_0_5                 
  *         @arg USART_StopBits_2                   
  *         @arg USART_StopBits_1_5              
  * @param  USART_Parity: 
  *         @arg USART_Parity_No                     
  *         @arg USART_Parity_Even                   
  *         @arg USART_Parity_Odd                    
  * @param  USART_Mode: 
  *         @arg USART_Mode_Rx                      
  *         @arg USART_Mode_Tx    
  *         @arg USART_Mode_Tx|USART_Mode_Rx     
  * @retval None
  */
void UART_Config(UART_HandleTypeDef *uart,uint32_t USART_BaudRate, 
	           uint16_t USART_WordLength,uint16_t USART_StopBits,
               uint16_t USART_Parity,uint16_t USART_Mode)
{
	USART_DeInit(uart->UART_BASEx);
	uart->UARTx.USART_BaudRate = USART_BaudRate;
	uart->UARTx.USART_WordLength = USART_WordLength;
	uart->UARTx.USART_StopBits = USART_StopBits;
	uart->UARTx.USART_Parity = USART_Parity;
	uart->UARTx.USART_Mode = USART_Mode;
	uart->UARTx.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(uart->UART_BASEx,&uart->UARTx);
}
/**
  * @brief  Initializes the GPIOx for usart RX
  * @param  GPIO_PinRx: GPIO_Pin_x,where x can be (0..15) to select the GPIO pin.      
  * @param  GPIORx :GPIOx,where x can be (A..G) to select the GPIO peripheral.
  * @retval None
  */
void UART_GPIO_RX_Config(UART_HandleTypeDef *uart, 
	                uint16_t GPIO_PinRx,GPIO_TypeDef *GPIORx)
{
	uart->UARTx_Rx.GPIO_Pin = GPIO_PinRx;
	uart->UARTx_Rx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	uart->UARTx_Rx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIORx,&uart->UARTx_Rx); 
}
/**
  * @brief  Initializes the GPIOx for usart TX
  * @param  GPIO_PinTx: GPIO_Pin_x,where x can be (0..15) to select the GPIO pin.      
  * @param  GPIOTx :GPIOx,where x can be (A..G) to select the GPIO peripheral.
  * @retval None
  */
void UART_GPIO_TX_Config(UART_HandleTypeDef *uart, 
	                uint16_t GPIO_PinTx,GPIO_TypeDef *GPIOTx)
{
	uart->UARTx_Tx.GPIO_Pin = GPIO_PinTx;
	uart->UARTx_Tx.GPIO_Mode = GPIO_Mode_AF_PP;
	uart->UARTx_Tx.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOTx,&uart->UARTx_Tx); 
}

/**
  * @brief  USART1 Interrupt Handler.
  * @param  None      
  * @retval None
  */
void UART4_IRQHandler(void)
{   
	if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
	{
		(void)UART4->SR;
		(void)UART4->DR;
		DMA_Cmd(DMA2_Channel3,DISABLE);
		DataLen=JudgeBufBiggestSize-DMA_GetCurrDataCounter(DMA2_Channel3);
		JudgeBuffReceive(JudgeReceiveBuffer,0);

		DMA_SetCurrDataCounter(DMA2_Channel3,JudgeBufBiggestSize);
		USART_ClearITPendingBit(UART4,USART_IT_IDLE);
		USART_ClearFlag(UART4,USART_FLAG_IDLE);
		DMA_Cmd(DMA2_Channel3,ENABLE);
	}	
}
void DMA2_Channel3_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_FLAG_TC3))
	{
		DMA_ClearFlag(DMA2_FLAG_TC3);
		DMA_ClearITPendingBit(DMA2_FLAG_TC3);
		JudgeBuffReceive(JudgeReceiveBuffer,0);
	}
}
void DMA2_Channel5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_IT_TC5)!=RESET)
	{
		DMA_ClearFlag(DMA2_FLAG_TC5);
		DMA_Cmd(DMA2_Channel5, DISABLE);
		DMA_SetCurrDataCounter(DMA1_Channel4, 28);
		DMA_Cmd(DMA2_Channel5, ENABLE);
		
	}	
}
/**
  * @brief  USART2 Interrupt Handler.
  * @param  None      
  * @retval None
  */
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)  
	{ 
		(void)USART2->SR;
		(void)USART2->DR;	
	} 
}

/**
  * @brief  USART2 TX-DMA发送完成中断
  * @param  None      
  * @retval None
  */
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC7) == SET)
	{
		DMA_Cmd(DMA1_Channel7, DISABLE);
		DMA_SetCurrDataCounter(DMA1_Channel7, U2_TX_LEN);
		DMA_ClearFlag(DMA1_FLAG_TC7);
	}
}

void master2slave_task(void)
{
//	//User Code.此处修改发送数组
//	uart2_TxBuffer[0] = '!';
//	uart2_TxBuffer[1] =solenoid_control[1];
//	uart2_TxBuffer[2] =solenoid_control[2];
//	uart2_TxBuffer[3] =solenoid_control[3];
//	uart2_TxBuffer[4] =solenoid_control[4];
//	uart2_TxBuffer[5] =solenoid_control[5];
//	uart2_TxBuffer[6] =solenoid_control[6];
//	uart2_TxBuffer[7] =solenoid_control[7];
//	uart2_TxBuffer[8] =solenoid_control[8];
//	uart2_TxBuffer[9] =solenoid_control[9];
//	Append_CRC8_Check_Sum(uart2_TxBuffer, 11);
//	
//	//开启U2 TX_DMA
	DMA_Cmd(DMA1_Channel7, ENABLE);
}
/**
  * @brief  USART2 RX-DMA发送完成中断
  * @param  None      
  * @retval None
  */
int x3;
unsigned char temp[7], temp_rx[7*2],temp1[7];
void DMA1_Channel6_IRQHandler(void)
{
	short i, n;
	if(DMA_GetFlagStatus(DMA1_FLAG_TC6) == SET)
	{
		x3 ++;
		memcpy(temp_rx + 7, uart2_RxBuffer, 7);
		for(n=0;n<7;n++)
		{
			if(temp_rx[n] == '!' )//
			{
				for(i=0;i<7;i++)
				{
					temp[i] = temp_rx[n+i];
				}
				
				break;
			}
			
		}
		if(temp[0] == '!'&&Verify_CRC8_Check_Sum(temp, 7) == 1)
		{
				for(n=0;n<7;n++)
			{
				temp1[n] = temp[n];
			}
		}
		memcpy(temp_rx, temp_rx + 7, 7);
		DMA_ClearFlag(DMA1_FLAG_TC6);
}
}

/**
  * @brief  USART3 Interrupt Handler to receive remote data.
  * @param  None      
  * @retval None
  */
void USART3_IRQHandler(void)
{
	static int DATA_LENGTH=0;
	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)    //闲时中断
	{
		(void)USART3->SR;
		(void)USART3->DR;	
		DMA_Cmd(DMA1_Channel3,DISABLE);
		DATA_LENGTH = RX_USART3_BUFFER - DMA_GetCurrDataCounter(DMA1_Channel3);
		if(DATA_LENGTH==18)
		{
			remote_receive(uart3_RxBuffer);
			
		}
		DMA_SetCurrDataCounter(DMA1_Channel3,RX_USART3_BUFFER);	
		DMA_Cmd(DMA1_Channel3,ENABLE);
    }	
}

/**
  * @brief  USART4 Interrupt Handler.
  * @param  None      
  * @retval None
  */
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_IDLE)!=RESET)  
	{  
		USART_ClearITPendingBit(USART1,USART_IT_IDLE);//清除空闲标志位
	} 
}

/**
  * @brief  UART5 Interrupt Handler.
  * @param  None      
  * @retval None
  */
void UART5_IRQHandler(void)
{
	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)  
	{  
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除发送完成标志位
	} 
}

void JudgeBuffReceive(unsigned char ReceiveBuffer[],uint16_t DataLen)
{
	uint16_t cmd_id;
	short PackPoint;
	memcpy(&SaveBuffer[JudgeBufBiggestSize],&ReceiveBuffer[0],JudgeBufBiggestSize);
	for(PackPoint=0;PackPoint<JudgeBufBiggestSize;PackPoint++)
	{
		if(SaveBuffer[PackPoint]==0xA5) 
		{	
			if((Verify_CRC8_Check_Sum(&SaveBuffer[PackPoint],5)==1))
			{
				cmd_id=(SaveBuffer[PackPoint+6])&0xff;
				cmd_id=(cmd_id<<8)|SaveBuffer[PackPoint+5];  
				DataLen=SaveBuffer[PackPoint+2]&0xff;
				DataLen=(DataLen<<8)|SaveBuffer[PackPoint+1];
				if((cmd_id==0x0201)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9))) 
				{
					JudgeReceive.RobotID=SaveBuffer[PackPoint+7];
					JudgeReceive.RobotLevel=SaveBuffer[PackPoint+8];
					JudgeReceive.remainHP=(SaveBuffer[PackPoint+10]<<8)|SaveBuffer[PackPoint+9]; 
				}
				if((cmd_id==0x0207)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					if(SaveBuffer[PackPoint+7+0]==1)
					{
						JudgeReceive.bulletFreq= SaveBuffer[PackPoint+7+1];
						memcpy(&JudgeReceive.bulletSpeed,&SaveBuffer[PackPoint+7+2],4);
					}
				}
				if((cmd_id==0x0202)&&(Verify_CRC16_Check_Sum(&SaveBuffer[PackPoint],DataLen+9)))
				{
					memcpy(&JudgeReceive.realChassisOutV,&SaveBuffer[PackPoint+7+0],2);
					JudgeReceive.realChassisOutV = JudgeReceive.realChassisOutV /1000.0f;
					memcpy(&JudgeReceive.realChassisOutA,&SaveBuffer[PackPoint+7+2],2);
					memcpy(&JudgeReceive.realChassispower,&SaveBuffer[PackPoint+7+4],4);
					memcpy(&JudgeReceive.remainEnergy,&SaveBuffer[PackPoint+7+8],2);
					memcpy(&JudgeReceive.shooterHeat17,&SaveBuffer[PackPoint+7+10],2);
				}
			}
		}
	}
	memcpy(&SaveBuffer[0],&SaveBuffer[JudgeBufBiggestSize],JudgeBufBiggestSize);
}

void JudgeSendFill(float data1,float data2,float data3 ,unsigned char mask)
{	
	/*FrameHeader(5 Bytes)*/
//	unsigned char SOF = 0xA5;
	unsigned short DataLength = 19;
	unsigned short DataID = 0xD180;
	unsigned short SenderID ;//红方ID：0x0002   蓝方ID：0x0012
	unsigned short ReserverID ;//红方ID：0x0102   蓝方ID：0x0112    
	if(JudgeReceive.RobotID == 2)
	{
		SenderID = 2;
		ReserverID = 0x0102;
	}
	if(JudgeReceive.RobotID == 12)
	{
		SenderID = 12;
		ReserverID = 0x0112;
	}
	static unsigned char Seq = 0;
	/*CmdID(2 Bytes)*/
	unsigned short UserID = 0x0301;   //The User's ID is 0x0301.
	JudegeSend[0] = 0xA5;
	JudegeSend[1] = (unsigned char)(DataLength&0xff);
	JudegeSend[2] = (unsigned char)((DataLength >> 8)&0xff);
	JudegeSend[3] =  Seq;  //包序号 0~255
	Append_CRC8_Check_Sum(JudegeSend, 5);
	//CmdID
	JudegeSend[5] = (unsigned char)((UserID&0xff)&0xff);
	JudegeSend[6] = (unsigned char)((UserID >> 8)&0xff);
	/*Data(13 Bytes)*/
	JudegeSend[7] = (unsigned char)( DataID&0xff);
	JudegeSend[8] = (unsigned char)((DataID >> 8)&0xff);
	
	JudegeSend[9] = (unsigned char)( SenderID&0xff);
	JudegeSend[10] = (unsigned char)((SenderID >> 8)&0xff);
	
	JudegeSend[11] = (unsigned char)( ReserverID&0xff);
	JudegeSend[12] = (unsigned char)((ReserverID >> 8)&0xff);
	/*FrameTail(2 Bytes, CRC16)*/
	
	UserMes.fdata=data1;
	JudegeSend[16] = (UserMes.idata>>24)&0xff;
	JudegeSend[15] = (UserMes.idata>>16)&0xff;
	JudegeSend[14] = (UserMes.idata>>8)&0xff;
	JudegeSend[13] = UserMes.idata&0xff;
		
	UserMes.fdata=data2;
	JudegeSend[20] = (UserMes.idata>>24)&0xff;
	JudegeSend[19] = (UserMes.idata>>16)&0xff;
	JudegeSend[18] = (UserMes.idata>>8)&0xff;
	JudegeSend[17] = UserMes.idata&0xff;
		
	UserMes.fdata=data3;
	JudegeSend[24] = (UserMes.idata>>24)&0xff;
	JudegeSend[23] = (UserMes.idata>>16)&0xff;
	JudegeSend[22] = (UserMes.idata>>8)&0xff;
	JudegeSend[21] = UserMes.idata&0xff;

	JudegeSend[25] = mask;
	
	Append_CRC16_Check_Sum(JudegeSend, 28);
	DMA_Cmd(DMA1_Channel4,ENABLE);
}
int LandingState,manual_get_bullet_State,manual_land_State;
char modeState;
void ReturnState(void *pvParameters)
{
	LandingState = g_flag.landing_state;
	manual_get_bullet_State = key.flag_manual_get_buttel_mode;
  manual_land_State = key.flag_manual_landing_mode;
	
	JudgeSendFill(LandingState, manual_get_bullet_State, manual_land_State, modeState);
}
