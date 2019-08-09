#include "bsp.h"

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

uint32_t ADC_Data[9];
uint8_t Uart1RX_Data,Uart3RX_Data;

void ADC_DataLinkInit(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_Data,4);	
	
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)&ADC_Data[4],5);	
}



void USART_IT_Start(void)
{
	HAL_UART_Receive_IT(&huart1,&Uart1RX_Data,1);
	//HAL_UART_Receive_IT(&huart2,&Uart2RX_Data,1);
	HAL_UART_Receive_IT(&huart3,&Uart3RX_Data,1);
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		//HAL_UART_Transmit(&huart3,&Uart1RX_Data,1,0xFFFF);
		Usart1_Data_Recv();
        HAL_UART_Receive_IT(&huart1,&Uart1RX_Data,1);
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_12);
	}
	
	//if(huart->Instance == USART2)
	//{
	///	HAL_UART_Transmit(&huart2,&Uart2RX_Data,1,0xFFFF);    
  //      HAL_UART_Receive_IT(&huart2,&Uart2RX_Data,1);      
	//}
	
	if(huart->Instance == USART3)
	{
		//HAL_UART_Transmit(&huart1,&Uart3RX_Data,1,0xFFFF);   
		Usart3_Data_Recv(); 
        HAL_UART_Receive_IT(&huart3,&Uart3RX_Data,1);
	}
}


uint8_t Uart1_RxBuff[7];             //����1���մ��������ݵĻ�������
uint8_t Uart1_Rx_Cnt = 0;           //����1���յ��ۼ��ֽ���
uint8_t Uart3_Rx_Cnt = 0;		     //����3���յ��ۼ��ֽ���
uint8_t flag_usart1_rev = 0;          //����1����������ɱ�־
uint8_t flag_usart3_rev = 0;              //����3����������ɱ�־
uint8_t SPEED_LEVEL_REV = 0;

/*******��PC���͵����ݷֱ𱣴�******/
#define LEN_USART3_REV = 35;                       //Ҫ��������ݳ���
uint8_t SendPcCmd[5] = {0x92,0x02,0x02,0x0F};       //35�ֽڵ�ͷ4�ֽ����ݣ�����У��
uint8_t Uart3_RxBuff[35];            //����3��������PC�����ݵĻ�������
uint8_t Data_Pressure[3];   //��ѹ    A1A2A3
uint8_t Data_FronAngle[3];  //ǰ�����B1B2B3
uint8_t Data_SideAngle[3];  //�������C1C2C3
uint8_t Data_Temprture[3];   //�¶�    D1D2D3
uint8_t Data_Voltage[3];    //��ѹ    E1E2E3
uint8_t Data_Height[3];     //�߶�    F1F2F3
uint8_t Data_Meter[3];      //����    G1G2G3
uint8_t Data_Speed[3];      //�ٶ�    H1H2H3
uint8_t Data_Multi[3];


/*******MCU���͸�������������******/

//�������ݸ�������������ǰҪ�ȼ�3��FF����ֹ���ݽ��մ���
uint8_t END_CMD[3]={0xFF,0xFF,0xFF};

//mcu���͸�����������¼����ʾ״̬
uint8_t Str_RecordOn1[25]="gm2.aph=127";//¼��������
uint8_t Str_RecordOn2[25]="p12.aph=0";
uint8_t Str_RecordOn3[25]="play 0,9,0";
uint8_t Flag_Record = 0;        //¼��״̬ת���־��0�رգ�1����
uint8_t Str_RecordOff1[25]="gm2.aph=0";//¼��ر�����
uint8_t Str_RecordOff2[25]="p12.aph=127";
uint8_t Str_RecordOff3[25]="play 0,8,0";



/*****************************************************************
************************������1�ʹ�3������**********************
**********************************************************
*********************************************************/

void HandleUsart3Pc(void)                 //from Usart3
{
		uint8_t i;
		
		if(flag_usart3_rev == 1)
		{
		
				for(i=0;i<4;i++)       //У��ǰ��4�ֽ�
				{
						if(Uart3_RxBuff[i] != SendPcCmd[i])
						{
							flag_usart3_rev = 0;
							Uart3_Rx_Cnt = 0;
							memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff));
							return;
						}
				}
				
			  if(flag_usart3_rev == 1 && check_crc(Uart3_RxBuff, 35))  //check crc
			   {
			   			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0|GPIO_PIN_1);
					   Send_Press_Fun();	//send press to screen
					   HAL_Delay(20);               //????
					   Send_FrnAgle_Fun();       //send front angle to screen
					   HAL_Delay(20);
					   Send_SideAgle_Fun();               //send side angle to screen
					   HAL_Delay(20);
					   Send_Height_Fun();
					   HAL_Delay(20);
					   Send_Length_Fun();
					   HAL_Delay(20);
					   Send_Speed_Fun();
					   HAL_Delay(20);
					   Send_Level_Fun();

				}

			    Uart3_Rx_Cnt = 0;
				flag_usart3_rev = 0;
				memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff));

		}
}



void HandleUsart1Screen(void)  //������1�����Դ����������ݺ󷢸�PC
{
	
		if(flag_usart1_rev == 1)
		{
			
			switch (Uart1_RxBuff[0])
			{
				case 0x82:	if(check_crc(Uart1_RxBuff,7))		                      //У��
						

								HAL_UART_Transmit(&huart3, Uart1_RxBuff, 7, 0xFFFF);   //����82��ͷ������  7�ֽ�
						

							break;
							
				default:	break;
			}
			
			Uart1_Rx_Cnt = 0;
			flag_usart1_rev = 0;
			memset(Uart1_RxBuff,0x0,sizeof(Uart1_RxBuff));

		}
	
}

/****************recv data in usart1 from screen*********
*********************************************************/

void Usart1_Data_Recv(void)          //����1���մӴ��������͵�����
{
	if(flag_usart1_rev == 0)
	{

			Uart1_RxBuff[Uart1_Rx_Cnt++] = Uart1RX_Data;

			switch(Uart1_RxBuff[0])              //���ݵ�һ�ֽڣ��ֱ𱣴治һ�����ȵ�����
			{
				case 0x82:
			                       //����������Ҫ����7�ֽ�
							if(Uart1_Rx_Cnt == 7)
							{
								flag_usart1_rev = 1;      //����7�ֽ������λ
							}
							break;

							
				default:                           //���ֽڲ�����Ҫ�����ݣ����
							Uart1_Rx_Cnt = 0;           
							memset(Uart1_RxBuff,0x00,sizeof(Uart1_RxBuff));
							break;
			 }
	
	}
	
}

/****************recv data in usart3 from PC*********
*****************************************************/
void Usart3_Data_Recv(void)
{
		if(flag_usart3_rev == 0)
		{

			Uart3_RxBuff[Uart3_Rx_Cnt++] = Uart3RX_Data;
				
			switch (Uart3_RxBuff[0] )          //�жϽ��յĵ�һ�ֽ�����
			{
			
				case 0x92:	
							if(Uart3_Rx_Cnt == 35)
							{
								flag_usart3_rev = 1;   //��һ�ֽڷ��ϣ��������������
							}
							break;

							
			    default:  	Uart3_Rx_Cnt = 0;
						    memset(Uart3_RxBuff,0x00,sizeof(Uart3_RxBuff));
						    break;
			}
			
		}
      
}
	
/************************************************************************
***********************************���ִ����Ӻ���**********************
**********************************************************
*********************************************************/

void Send_Press_Fun(void)//����С����ѹ����С�����޸���
{
		uint8_t len = 0;
		uint8_t i=0;
		
		uint8_t data_Pressure[8];// t6.txt="XX"  ��ѹ
		uint8_t Str_Pressure[30]="main.t6.txt=\"";// t6.txt="XX"  ��ѹ


		for(i=0;i<3;i++)  //���������
	   {
	   		data_Pressure[i] = Uart3_RxBuff[4+i];   
	   }
	   uint32_t value=0;
	   value = data_Pressure[0] << 8 | data_Pressure[1];//��ԭΪԭ������ 
	   
	   if(value !=0)
	  {
		   uint8_t tmp1 = value/100;
		   uint8_t tmp2 = value/10%10;
		   uint8_t tmp3 = value%10;	   
			  
			data_Pressure[0] = tmp1 +'0';
			data_Pressure[1] = tmp2 +'0';
			data_Pressure[2] = tmp3 +'0';//len+3
			data_Pressure[3] ='\0';
			
			mydeleteChar(data_Pressure,'0',&len);//���˿�ʼ��0�ַ�

			for(i=0;i<len;i++)
			{
			 	Str_Pressure[13+i] = data_Pressure[i];
			}
	  }
	  else if(value == 0)
	  {
		  len = 0;
		  Str_Pressure[13+len] = '0';
		  len++;

	  }
		Str_Pressure[13+len]='\"';

		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,Str_Pressure,13+len+1,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������

}

void Send_FrnAgle_Fun(void)//����ǰ����ǣ��и���
{
	uint8_t len = 0;
	uint8_t i=0;
	uint8_t tmp1 =0;
	uint8_t tmp2 =0;
    int16_t value=0;
	uint8_t data_FronAngle[9];// t8.txt="XX"  �������
	uint8_t Str_FronAngle[30]="main.t8.txt=\"";// t8.txt="XX"  �������


	   for(i=0;i<3;i++)  //���������
	   {
			data_FronAngle[i] = Uart3_RxBuff[7+i];
	   }
	   value = data_FronAngle[0] << 8 | data_FronAngle[1];


	 if(value != 0)
	 {
	   if(value > 0)//����
	   {
		   tmp1 = value/10%10;
		   tmp2 = value%10;
		   data_FronAngle[0] = tmp1 +'0';
		   data_FronAngle[1] = tmp2 +'0';
		   data_FronAngle[2] = '\0';
	   }  
	   else if(value < 0)//���� 
	   {
			value = -value;//�õ�����
		    tmp1 = value/10%10;
		    tmp2 = value%10;
			data_FronAngle[0] = '-';
			data_FronAngle[1] = tmp1+'0';  
			data_FronAngle[2] = tmp2+'0';
			data_FronAngle[3] = '\0';
	   }
	   mydeleteChar(data_FronAngle,'0',&len);//���˿�ʼ��0�ַ�
	   for(i=0;i<len;i++)
	   {
		   Str_FronAngle[13+i] = data_FronAngle[i];
	   }
		
	 }
	 else if(value == 0)
	 {
	 	len = 0;
		Str_FronAngle[13+len] = '0';
		len++;
	 }
	 
	  Str_FronAngle[13+len]='\"';

	   HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
	   HAL_UART_Transmit(&huart1,Str_FronAngle,13+len+1,0xFFFF);//���͸�������
	   HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������

}

void Send_SideAgle_Fun(void)//����������ǣ��и���
{
	uint8_t len = 0;
	uint8_t i=0;
	uint8_t tmp1 =0;
	uint8_t tmp2 =0;
    int16_t value=0;
	uint8_t data_SideAngle[9];// t9.txt="XX"  ǰ�����
	uint8_t Str_SideAngle[30]="main.t9.txt=\"";// t9.txt="XX"  ǰ�����
	
	   for(i=0;i<3;i++)  //���������
	   {
			data_SideAngle[i] = Uart3_RxBuff[10+i];
			   
	   }
	   value = data_SideAngle[0] << 8 | data_SideAngle[1];

	   if(value != 0)
	  {

	 	 if(value > 0)//����
		 {
			 tmp1 = value/10%10;
			 tmp2 = value%10;
			 data_SideAngle[0] = tmp1 +'0';
			 data_SideAngle[1] = tmp2 +'0';
			 data_SideAngle[2] = '\0';
		 }	
		 else if(value < 0)//���� 
		 {
			  value = -value;//�õ�����
			  tmp1 = value/10%10;
			  tmp2 = value%10;
			  data_SideAngle[0] = '-';
			  data_SideAngle[1] = tmp1+'0';  
			  data_SideAngle[2] = tmp2+'0';
			  data_SideAngle[3] = '\0';
		 }
		 mydeleteChar(data_SideAngle,'0',&len);//���˿�ʼ��0�ַ�
		 
		 for(i=0;i<len;i++)
		 {
			 Str_SideAngle[13+i] = data_SideAngle[i];
		 }
	  }
	  else if(value == 0)
	  {
	 	len = 0;
		Str_SideAngle[13+len] = '0';
		len++;
	  }
		 Str_SideAngle[13+len]='\"';

		 HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		 HAL_UART_Transmit(&huart1,Str_SideAngle,13+len+1,0xFFFF);//���͸�������
		 HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		 
}

void Send_Height_Fun(void)//����ƽ̨�߶ȣ���������С��
{
		uint8_t len = 0;
		uint8_t i=0;
		uint8_t data_Height[9];    // t4.txt="XX"   �߶�
		uint8_t Str_Height[30]="main.t4.txt=\"";// t4.txt="XX"   �߶�
		
	   for(i=0;i<3;i++)  //���������
	   {
			data_Height[i] = Uart3_RxBuff[19+i];
			   
	   }
	   int16_t value = data_Height[0] << 8 | data_Height[1];

	  if(value != 0)
	  {
			uint8_t tmp1 = value/100;
		   uint8_t tmp2 = value/10%10;
		   uint8_t tmp3 = value%10;

			data_Height[0] = tmp1 +'0';
			data_Height[1] = tmp2 +'0';
			data_Height[2] = tmp3 +'0';
			data_Height[3] = '\0';

			mydeleteChar(data_Height,'0',&len);//���˿�ʼ��0�ַ�

			for(i=0;i<len;i++)
			{
			 	Str_Height[13+i] = data_Height[i];
			}
	   }
	  else if(value == 0)
	  {
	 	len = 0;
		Str_Height[13+len] = '0';
		len++;
	  }
		Str_Height[13+len]='\"';

		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,Str_Height,13+len+1,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������

	
}

void Send_Length_Fun(void)//���ͼ��ף��и�������С��
{
		uint8_t len = 0;
		uint8_t i=0;
		uint8_t tmp1 =0;
		uint8_t tmp2 =0;
		uint8_t tmp3 =0;
		uint8_t data_Length[9];
		uint8_t Str_Length[20]="main.t0.txt=\"";//С������
		
	   for(i=0;i<3;i++)  //���������
	   {
			data_Length[i] = Uart3_RxBuff[22+i];
			   
	   }
	   int16_t value = data_Length[0] << 8 | data_Length[1];
	   
	   uint8_t mmp1 = data_Length[2]/10%10;//С��
	   uint8_t mmp2 = data_Length[2]%10;

	   i=0;
       if(value !=0)
       {
		   if(value < 0)//���� 
		   {
				value = -value;//�õ�����
				data_Length[i]='-';
				i++;
		   }
		   
	       tmp1 = value/100;
		   tmp2 = value/10%10;
		   tmp3 = value%10;
		   data_Length[i++] = tmp1 +'0';
		   data_Length[i++] = tmp2 +'0';
	   	   data_Length[i++] = tmp3 +'0';
		   data_Length[i++] = '.';		//С������
		   data_Length[i++] = mmp1 +'0';
		   data_Length[i++] = mmp2 +'0';
		   data_Length[i++] = '\0';
			   
		   mydeleteChar(data_Length,'0',&len);//���˿�ʼ��0�ַ�

		   for(i=0;i<len;i++)
			{
			 	Str_Length[13+i] = data_Length[i];
			}
	   }
	  else if(value == 0)
	  {
			len = 0;
			Str_Length[13+len] =0x30;// '0';
			len++;
	  }
		 
		Str_Length[13+len]='\"';

		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,Str_Length,13+len+1,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		
}

void Send_Speed_Fun(void)//�и�������С��
{
	
		uint8_t len = 0;
		uint8_t i=0;
		uint8_t data_Speed[9];     // t2.txt="XX"   �ٶ� 	
		uint8_t Str_Speed[30]="main.t2.txt=\"";// t2.txt="XX" �ٶ�

	   for(i=0;i<3;i++)  //���������
	   {
			data_Speed[i] = Uart3_RxBuff[25+i];
			   
	   }
	   int16_t value = data_Speed[0] << 8 | data_Speed[1];
	   uint8_t tmp1 = 0;
	   uint8_t tmp2 = 0;
	   uint8_t tmp3 = 0;
	   uint8_t mmp1 = data_Speed[2]/10%10;//�ȼ���С��λ
	   uint8_t mmp2 = data_Speed[2]%10;

	   i=0;
	   if(value!= 0 && data_Speed[2]!=0 )//����С��������0
	   {
		   if(value < 0)//���� 
		   {
				value = -value;//�õ�����
				data_Speed[i]='-';
				i++;
		   }
		   tmp1 = value/100;		//�����λ��ʮλ����λ
		   tmp2 = value/10%10;
		   tmp3 = value%10;

			data_Speed[i++] = tmp1 +'0';
			data_Speed[i++] = tmp2 +'0';
			data_Speed[i++] = tmp3 +'0';
			data_Speed[i++] = '.';
			data_Speed[i++] = mmp1 +'0';
			data_Speed[i++] = mmp2 +'0';
		  	data_Speed[i++] = '\0';
			mydeleteChar(data_Speed,'0',&len);//���˿�ʼ��0�ַ�

	  }
	   else if(value==0 && data_Speed[2]!=0)//����Ϊ0 С����Ϊ0
	   {
			
			data_Speed[0] = '0';
			data_Speed[1] = '.';
			data_Speed[2] = mmp1 +'0';
			data_Speed[3] = mmp2 +'0';
			len = 4;
			
	   }
	   else//����0
	   {
			data_Speed[0] = '0';
			len = 1;

	   }
	 	for(i=0;i<len;i++)
		{
		 	Str_Speed[13+i] = data_Speed[i];
		}
		
		Str_Speed[13+len]='\"';

		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,Str_Speed,13+len+1,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
}

void Send_Level_Fun(void)//����ת������
{
		uint8_t i=0;
		uint8_t len = 0;
		uint8_t data_Level[9];
		uint8_t tmp[3];
		uint8_t Str_Tor1[20]="carSet.n0.val=";//ת�䵲λ
		uint8_t Str_Tor2[20]="carSet.h0.val=";//
		uint8_t Str_Sped1[20]="carSet.n1.val=";//�ٶȵ�λ
		uint8_t Str_Sped2[20]="carSet.h1.val=";
		uint8_t Str_MLamp1[25]="lightingSet.n0.val=";//����Ƶ�λ
		uint8_t Str_MLamp2[25]="lightingSet.h0.val=";//����Ƶ�λ
		uint8_t Str_SLamp1[25]="lightingSet.n1.val=";//����Ƶ�λ
		uint8_t Str_SLamp2[25]="lightingSet.h1.val=";//����Ƶ�λ
		uint8_t record = 0;
	
	   for(i=0;i<5;i++)  			//���������
	   {
			data_Level[i] = Uart3_RxBuff[28+i];
		//	printf("~~%d\n",data_Level[0]);
			   
	   }
		
		Str_Sped1[14]=  data_Level[1]+'0';//�ٶȵ�λ
		Str_Sped2[14]=  data_Level[1]+'0';//�ٶȵ�λ
		 
		SPEED_LEVEL_REV = data_Level[1];/////////
		 
		uint8_t tmp1= data_Level[2]/11;
		uint8_t tmp2= data_Level[3]/11;
		
		Str_MLamp1[19]=  tmp1+'0';//����Ƶ�λ
		Str_MLamp2[19]=  tmp1+'0';//����Ƶ�λ
		Str_SLamp1[19]=  tmp2+'0';//�����
		Str_SLamp2[19]=  tmp2+'0';//�����
		record = data_Level[4];

		SPEED_LEVEL_REV = data_Level[0];//����ת�����ȣ�ҡ������תָ����Ҫ����
		if(data_Level[0] != 0 &&data_Level[0]!=0 )//ת������
	    {
	    	//printf("~2~%d\n",data_Level[0]);
			tmp[0] = data_Level[0]/100;
		    tmp[1]= data_Level[0]/10%10;
		    tmp[2]= data_Level[0]%10;

			
			tmp[0] = tmp[0] +'0';
			tmp[1] = tmp[1] +'0';
			tmp[2] = tmp[2] +'0';
			len=3;
			mydeleteChar(tmp,'0',&len);//���˿�ʼ��0�ַ�

			for(i=0;i<len;i++)
			{
			 	Str_Tor1[14+i] = tmp[i];
				Str_Tor2[14+i] = tmp[i];
			//	printf("-----%s,",&tmp[i]);
			}
	   }
	   else
	   {
			len = 0;
			Str_Tor1[14+len] = '0';
			Str_Tor2[14+len] = '0';
			len++;
	   }


		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,Str_Tor1,14+len,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_Delay(20);
		HAL_UART_Transmit(&huart1,Str_Tor2,14+len,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_Delay(20);
		
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,Str_Sped1,15,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_Delay(20);
		HAL_UART_Transmit(&huart1,Str_Sped2,15,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_Delay(20);
		
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,Str_MLamp1,20,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_Delay(20);
		HAL_UART_Transmit(&huart1,Str_MLamp2,20,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_Delay(20);

		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,Str_SLamp1,20,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
	    HAL_Delay(20);
		HAL_UART_Transmit(&huart1,Str_SLamp2,20,0xFFFF);//���͸�������
		HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
	    HAL_Delay(20);

		

		if(record != Flag_Record )				//���ϴε�״̬�Ƚϣ���״̬��ͬ������¼��״̬
		{
			switch (record)
			{	//��¼�� 
				case 0:	HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
						HAL_UART_Transmit(&huart1, Str_RecordOff1, 9, 0xFFFF);//���͸�������
						HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
						HAL_Delay(20);
						HAL_UART_Transmit(&huart1, Str_RecordOff2, 11, 0xFFFF);//���͸�������
						HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
						HAL_Delay(20);
						HAL_UART_Transmit(&huart1, Str_RecordOff3, 10, 0xFFFF);//���͸�������
						HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
						HAL_Delay(20);
						Flag_Record =0;
						break;
				//��¼��
				case 1:	HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
						HAL_UART_Transmit(&huart1, Str_RecordOn1, 11, 0xFFFF);//���͸�������
						HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
						HAL_Delay(20);
						HAL_UART_Transmit(&huart1, Str_RecordOn2, 9, 0xFFFF);//���͸�������
						HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������
						HAL_Delay(20);
						HAL_UART_Transmit(&huart1, Str_RecordOn3, 10, 0xFFFF);//���͸�������
						HAL_UART_Transmit(&huart1,END_CMD,3,0xFFFF);//���͸�������					
						HAL_Delay(20);
						Flag_Record =1;
						break;

				default:
						break;
			}
		}
		
		
}

uint8_t check_crc(uint8_t a[], uint8_t len)        //�������У��ֵ
{
	uint8_t i;
	uint8_t crc;
	crc = a[0];
	for(i=1;i<len-1;i++)
	{
		crc = crc ^ a[i];
	}	
	//printf("crc=0x%x\n",crc);
//	printf("a[len-1]=0x%x\n", a[len -1]);
	if(crc == a[len -1])
	   return TRUE;
	else
	   return FALSE;
}


#if 0
uint8_t check_sum_crc(uint8_t *a,uint8_t len)//�������ݺ͵�У��ֵ
{
	uint8_t i;
	uint8_t crc;
	crc = *(++a);
	for(i=1;i<len-2;i++)
	{
		crc = crc + *(++a);
	}	
	if(*(++a) == crc)
		return TRUE;
	else
		return FALSE;
	//printf("sum crc = 0x%x\n",crc);
}
#endif

void mydeleteChar(uint8_t *str, char c, uint8_t *len) //ɾ��ǰ������'0'
{
	uint8_t *p;
	p = str;
	*len = 0;
	uint8_t flag = 1;
	(*len) ++;
	if(*p == '-')
	{
		*str++ = *p++;
		(*len)++;

	}
	while(*p)
	{
		if(flag == 1)
		{
			if(*p != c && flag == 1)
			{
				*str++ = *p;
				flag = 0;
			}
		}
		else
		{
			*str++ = *p;
			(*len)++;
			flag = 0;
		}
		p++;
		
	}
	*str = '\0';
}
