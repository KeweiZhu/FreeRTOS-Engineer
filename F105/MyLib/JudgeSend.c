#include "JudgeSend.h"


unsigned char JudegeSend[28];
extern tGameInfo JudgeReceive;
FloatLongType UserMes;

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
//int LandingState,manual_get_bullet_State,manual_land_State;
//char modeState;
//void ReturnState(void *pvParameters)
//{
//	LandingState = g_flag.landing_state;
//	manual_get_bullet_State = key.flag_manual_get_buttel_mode;
//  manual_land_State = key.flag_manual_landing_mode;
//	
//	JudgeSendFill(LandingState, manual_get_bullet_State, manual_land_State, modeState);
//}

