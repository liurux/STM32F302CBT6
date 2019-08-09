/***
 *                     _ooOoo_
 *                    o8888888o
 *                    88" . "88
 *                    (| -_- |)
 *                     O\ = /O
 *                 ____/`---'\____
 *               .   ' \\| |// `.
                  / \\||| : |||// \
                / _||||| -:- |||||- \
 *                | | \\\ - /// | |
 *              | \_| ''\---/'' | |
 *               \ .-\__ `-` ___/-. /
 *            ___`. .' /--.--\ `. . __
 *         ."" '< `.___\_<|>_/___.' >'"".
 *        | | : `- \`.;`\ _ /`;.`/ - ` : | |
 *         \  \ `-. \_ __\ /__ _/ .-` / /
 *  ======`-.____`-.___\_____/___.-`____.-'======
 *                     `=---='
 *
 * .............................................
 *          ���汣��             ����BUG
 */



#include "joystick.h"
#include "bsp.h"

#define CMD_LEN_7  7
//#define PRINT_FLAG       //printf�������޸ĵ�����1��ӡ //usart.c���޸�


//ҡ�˷���
#define JOY_U     1  //��
#define JOY_D     2   //��
#define JOY_L     3   //��
#define JOY_R     4   //��
#define JOY_S     5   //�м�


#define SEND_TIMES 1         //ҡ������ʹ���
#define FRONT 1
#define BACK 2


uint8_t send_times = 0;

uint32_t Joy2_Stop_Val = 0;       //����ĳҡ�� ֹͣʱ���ֵ

uint8_t Speed_Flag = 0;        //������һ�ε��ٶ�ֵ

//��¼ң�е�״̬��־�����ڱ���ÿ��ҡ�˵Ĳ���״̬
uint8_t Flag_JOY1 = 0;       
uint8_t Flag_JOY2 = 0;       //
uint8_t Flag_JOY3 = 0;       //
uint8_t Flag_JOY4 = 0;       //

uint8_t flag_ad_status_joy2 = 0;


//��¼��һ��ҡ��״̬�������ж�ҡ�˷����Ƿ�ı�
uint8_t status_JOY1 = 0; 
uint8_t status_JOY2 = 0; 
uint8_t status_JOY3 = 0; 
uint8_t status_JOY4 = 0;             

//����ҡ�˲���ָ����������ڼ���ҡ�˲������͵�ָ�����
uint8_t Count_Joy1 = 0;
uint8_t Count_Joy2 = 0;
uint8_t Count_Joy3 = 0;
uint8_t Count_Joy4 = 0;



/***************************************************************************
MCUҪ���͸�PC���ٶ�
�Ǹ���ǰ�����˵�ҡ�˲���ʱADC��ѹֵ���Է��嵵��
���嵵ADC��ѹֵ��ƥ�䵽��Ӧ��ͬ���ٶȣ��ٶ�Ҳ���嵵��
��ƥ��ĵ�ǰ�ٶ�ֵ���浽�ٶ������С�

MCU���͸�С���ٶȵ�ָ���ʽΪ82 A2 00 2F XX YY CRC   //7�ֽ�
XX YYΪ��λֵ����ʾ���ֽڵ�����//���ֽ� ���ֽ�
����500ʱ��С��ֹͣ�˶�
����500�� С��ǰ����
           ǰ�����嵵,ֵԽ���ٶ�Խ��(755��죬����755ʱPC����755����)
           
С��500 ��С������
           ���˷��嵵���ٶ�ԽС�ٶ�Խ��(0���)

**************************************************************************/

//��ǰ�ٶ�ֵ
uint8_t SPEED_VALUE[2] = {0x0,0x0};

//#define OLD_JOYSTICK//�ɵ�ң�ж���

//��ҡ���õģ�Ŀǰ����
//��ҡ�˵İ�װ�������ǰ�෴�����Ե�λ���ݻ���
//�����ٶȵĵ�λֵ
uint8_t SPEED_B1_LEVEL[2] = {0x01,0x90};//400
uint8_t SPEED_B2_LEVEL[2] = {0x01,0x5E};//350
uint8_t SPEED_B3_LEVEL[2] = {0x01,0x2C};//300
uint8_t SPEED_B4_LEVEL[2] = {0x00,0xC8};//200
uint8_t SPEED_B5_LEVEL[2] = {0x00,0x96};//150
uint8_t SPEED_B6_LEVEL[2] = {0x00,0x50};//80
uint8_t SPEED_B7_LEVEL[2] = {0x00,0x1E};//30
uint8_t SPEED_B8_LEVEL[2] ={0x00,0x00};//0

//ǰ���ٶȵĵ�λֵ���ٶȷ��嵵���ͣ�ֵԽ���ٶ�Խ��
//�ٶ�ֵΪ01 F4��500����С��ֹͣ�˶�
uint8_t SPEED_1_LEVEL[2] = {0x02,0x27};//551----
uint8_t SPEED_2_LEVEL[2] ={0x02,0x5A};//602 ----
uint8_t SPEED_3_LEVEL[2] = {0x02,0x80};//640-----
uint8_t SPEED_4_LEVEL[2] = {0x02,0xA8};//680-----                 //
uint8_t SPEED_5_LEVEL[2] = {0x02,0xBC};//700 //755-----      //�ٶ����ֵ 755������768�ᵱ��755
uint8_t SPEED_6_LEVEL[2] = {0x02,0xD0};//720
uint8_t SPEED_7_LEVEL[2] = {0x02,0xE4};//740
uint8_t SPEED_8_LEVEL[2] = {0x03,0x00};//768


//ҡ��ǰ������ʱADC�������λֵ�������жϴﵽ�ĸ���λ�ٶ�
//uint32_t SPEED_LEVEL[5] = {2380,2800,3200,3600,4000};
uint32_t SPEED_LEVEL[8] = {2380,2500,2800,3200,3400,3600,3800,4000};
//ҡ�˺��˲���ʱADC���嵵λֵ�������жϴﵽ�ĸ���λ�ٶ�
//uint32_t SPEED_LEVEL_B[5] = {50,340,630,920,1210};
uint32_t SPEED_LEVEL_B[8] = {20,50,100,200,340,630,920,1210};





//��ӡҡ�˷���debug��
uint8_t huanhang[3]="\r\n";
uint8_t joy4_u_str[7]="adc4-u";
uint8_t joy4_d_str[7]="adc4-d";
uint8_t joy4_l_str[7]="adc4-l";
uint8_t joy4_r_str[7]="adc4-r";
uint8_t joy4_s_str[7]="adc4-s";

uint8_t joy3_u_str[7]="adc3-u";
uint8_t joy3_d_str[7]="adc3-d";
uint8_t joy3_l_str[7]="adc3-l";
uint8_t joy3_r_str[7]="adc3-r";
uint8_t joy3_s_str[7]="adc3-s";

uint8_t joy2_u_str[7]="adc2-u";
uint8_t joy2_d_str[7]="adc2-d";
uint8_t joy2_l_str[7]="adc2-l";
uint8_t joy2_r_str[7]="adc2-r";
uint8_t joy2_s_str[7]="adc2-s";

uint8_t joy1_u_str[7]="adc1-u";
uint8_t joy1_d_str[7]="adc1-d";
uint8_t joy1_l_str[7]="adc1-l";
uint8_t joy1_r_str[7]="adc1-r";
uint8_t joy1_s_str[7]="adc1-s";

/****************************************************
********MCU���͸���������С����
/ǰ��/����/ֹͣ/��ת/��ת/Э���ַ���**************
******************************************************/
//С��ǰ��
uint8_t Str_Front1[10]="gm0.vid=0";
uint8_t Str_Front2[10]="gm1.vid=1";
uint8_t Str_Front3[9]="gm0.en=1";
uint8_t Str_Front4[9]="gm0.en=1";				    
uint8_t Str_Front5[9]="gm1.en=1";

//С������
uint8_t Str_Back1[10]="gm0.vid=2";
uint8_t Str_Back2[10]="gm1.vid=3";
uint8_t Str_Back3[9]="gm1.en=1";
uint8_t Str_Back4[9]="gm0.en=1";
uint8_t Str_Back5[9]="gm1.en=1";

//С���˶�ֹͣ
uint8_t Str_Stop1[9]="gm0.en=0";
uint8_t Str_Stop2[9]="gm1.en=0";

//С��ת��ֹͣ
uint8_t Str_TurnStop[15]="main.p3.aph=0";

//С����ת
uint8_t Str_TurnLeft1[20]="main.p3.pic=32";
uint8_t Str_TurnLeft2[20]="main.p3.aph=127";

//С����ת
uint8_t Str_TurnRight1[20]="main.p3.pic=33";
uint8_t Str_TurnRight2[20]="main.p3.aph=127";



//����ҡ������䱶�;۽�
//       �䱶+
//        |
//  �۽�--    -- �۽�+
//	      |
//       �䱶-
uint8_t Focus_Stp_Cmd[7]={0x82,0xA2,0x00,0x24,0x00,0x00,0x00};//�۽�ֹͣ
uint8_t Focus_Add_Cmd[7]={0x82,0xA2,0x00,0x24,0x00,0x01,0x00};//�۽�+
uint8_t Focus_Dec_Cmd[7]={0x82,0xA2,0x00,0x24,0x00,0x02,0x00};//�۽�-

uint8_t Zoom_Stp_Cmd[7]= {0x82,0xA2,0x00,0x25,0x00,0x00,0x00};//�䱶ֹͣ
uint8_t Zoom_Add_Cmd[7]={0x82,0xA2,0x00,0x25,0x00,0x01,0x00};//�䱶+
uint8_t Zoom_Dec_Cmd[7]={0x82,0xA2,0x00,0x25,0x00,0x02,0x00};//�䱶-



//����ҡ�������ͷ�ͻ�ͷ
//        ��ͷ����
//            |
//  ��ͷ��ת--      -- ��ͷ˳ת
//	          |
//        ��ͷ�¸�
uint8_t Lens_Stp_Cmd[7]={0x82,0x0A2,0x00,0x04,0x00,0x00,0x00};//��ͷ�����¸�ֹͣ����
uint8_t Lens_Up_Cmd[7]={0x82,0x0A2,0x00,0x04,0x00,0x01,0x00};//��ͷ����(���)����
uint8_t Lens_Dw_Cmd[7]={0x82,0x0A2,0x00,0x04,0x00,0x02,0x00};//��ͷ�¸����Ұڣ�����

uint8_t Head_Stp_Cmd[7]={0x82,0xA2,0x00,0x03,0x00,0x00,0x00};//��ͷֹͣ����
uint8_t Head_Lef_Cmd[7]={0x82,0xA2,0x00,0x03,0x00,0x01,0x00};//��ͷ����
uint8_t Head_Rgt_Cmd[7]={0x82,0xA2,0x00,0x03,0x00,0x02,0x00};//��ͷ����




//����ҡ��������߳���ƽ̨
//       ƽ̨��
//        |
//  ����--    -- ����
//	      |
//       ƽ̨��
uint8_t Plat_Stp_Cmd[7]={0x82,0xA2,0x00,0x2A,0x00,0x00,0x00};//ƽֹ̨ͣ
uint8_t Plat_Up_Cmd[7]={0x82,0xA2,0x00,0x2A,0x00,0x01,0x00};//ƽ̨��
uint8_t Plat_Dw_Cmd[7]={0x82,0xA2,0x00,0x2A,0x00,0x02,0x00};//ƽ̨��

uint8_t Line_Stp_Cmd[7]={0x82,0xA1,0x00,0x2B,0x00,0x00,0x00};//���߳�ֹͣ
uint8_t Line_Pull_Cmd[7]={0x82,0xA1,0x00,0x2B,0x00,0x01,0x00};//���߳�����
uint8_t Line_Put_Cmd[7]={0x82,0xA1,0x00,0x2B,0x00,0x02,0x00};//���߳�����



//����ҡ���������С������
//       ǰ��
//        |
//  ��ת--    -- ��ת
//	      |
//      ����
uint8_t Speed_Stp_Cmd[7]={0x82,0xA2,0x00,0x30,0x01,0xF4,0x00};//�ٶ�ֹͣ����
//ǰ������ 82 A2 00 2F XX YY CRC
//XX YY��ҡ�˲ٿص������趨�ٶ�ͬʱǰ�������
uint8_t Speed_Cmd[7]={0x82,0xA2,00,0x2F,0x00,0x00,0x00};//ǰ���������� 

uint8_t Torque_S_Cmd[7]= {0x82,0xA2,0x00,0x30,0x01,0xF4,0x00};//ת��ֹͣ����
uint8_t Torque_L_Cmd[7]={0x82,0xA2,0x00,0x30,0x00,0x00,0x00};//��ת����
uint8_t Torque_R_Cmd[7]={0x82,0xA2,0x00,0x30,0x02,0x00,0x00};//��ת����

//�ٶȵ�λ������͸�PC�ģ�82 A2 00 38 00 XX CRC��ҡ��ǰ������ʱͬʱҲ�ı䵲λ
uint8_t Speed_Level_Cmd[7]={0x82, 0xA2 , 0x00 , 0x38 , 0x00 , 0x00 , 0x00};


/**************************************************
************************����ҡ��ADC**********************
**********************************************************
*********************************************************/

void HandleAdcData(void)
{

	Get_Joystick_Staus();   //��ȡң��״̬
	
	Send_1_Place_Line();	
	Send_2_Speed_Tor();         
	Send_3_Zoom_Focus();
	Send_4_Angle_Turn();

}


void Get_Joystick_Staus(void)//��ȡң��״̬
{
	uint8_t i;
	//uint8_t adc_high[8];
	//uint8_t adc_low[8];
	//uint8_t format=0xFF;

	for(i=0;i<8;i++)
	{
		//adc_high[i] = ADC_Data[i]>>8;				//adc0�ĸ�8λ����
	   //adc_low[i] = ADC_Data[i]&0xFF;				 //adc�ĵ�8λ����
//		HAL_UART_Transmit(&huart1, &adc_high[i], 1, 0xFFFF);//��ӡ��ѹֵ������1
//		HAL_UART_Transmit(&huart1, &adc_low[i], 1, 0xFFFF);
//		HAL_UART_Transmit(&huart1,huanhang,2,0xFFFF);
		#ifdef PRINT_FLAG
			printf("%ld, ",ADC_Data[i]);
		#endif
	}
	#ifdef PRINT_FLAG
		HAL_Delay(1000);
		printf("\r\n");
	#endif

	get_adc_status();

}



void get_adc_status(void)
{
	//uint8_t i=0;
	if(ADC_Data[0] > 1500 && ADC_Data[1] > 3000)         //ҡ��1����ҡ��2
	{
		Flag_JOY1 = JOY_U;	
	}
	else if(ADC_Data[0] > 1800 && ADC_Data[1] < 800)
	{
		Flag_JOY1 = JOY_D;	
	}
	else if(ADC_Data[0] > 3300 && ADC_Data[1] > 1300)
	{
		Flag_JOY1 = JOY_L;	
	}
	else if(ADC_Data[0] < 1000 && (ADC_Data[1] >1700 && ADC_Data[1] < 2800))
	{
		Flag_JOY1 = JOY_R;	
	}
	else if((ADC_Data[0] >1800 &&ADC_Data[0] < 2500 )&& (ADC_Data[1] > 1800 &&ADC_Data[1] < 2500 ))
	{
		Flag_JOY1 = JOY_S;	
	}


	if(ADC_Data[2] > 1500 && ADC_Data[3] > 2400 )         //ҡ��2ǰ������
	{
		#ifdef PRINT_FLAG
			printf("\nFFFFF\n");
		#endif
		Flag_JOY2 = JOY_U;
//		flag_ad_status_joy2 = JOY_U;

	
		
	}
	else if(ADC_Data[2] > 1800   && ADC_Data[3] < 1500 ) //Ҫ����ǰ��
	{
		Flag_JOY2 = JOY_D;
	//	flag_ad_status_joy2 = JOY_D;

		#ifdef PRINT_FLAG
			printf("\nBBBBBBBBB\n");
		#endif

	}
	else if(ADC_Data[2] > 3300 && ADC_Data[3] > 1300 )
	{
		Flag_JOY2 = JOY_L;
	//	flag_ad_status_joy2 = JOY_L;
	}
	else if(ADC_Data[2] < 1000 && (ADC_Data[3] >1700 && ADC_Data[3] < 2800))
	{
		Flag_JOY2= JOY_R;
	//	flag_ad_status_joy2 = JOY_R;
	}
	else if((ADC_Data[2] >1800 &&ADC_Data[2] < 2500 )&& (ADC_Data[3] > 1800 &&ADC_Data[3] < 2500 ))
	{
		Flag_JOY2 = JOY_S;
	//	flag_ad_status_joy2 = JOY_S;
		Joy2_Stop_Val = ADC_Data[3];//����ֹͣʱ���ֵ
	}



	if(ADC_Data[4] > 1500 && ADC_Data[5] > 3000)        //ҡ��3����ҡ��1
	{
		Flag_JOY3 = JOY_U;

	}
	else if(ADC_Data[4] > 1800 && ADC_Data[5] < 800)
	{
		Flag_JOY3 = JOY_D;		

	}
	else if(ADC_Data[4] > 3300 && ADC_Data[5] > 1800)
	{
		Flag_JOY3 = JOY_L;

	}
	else if(ADC_Data[4] < 1000 && (ADC_Data[5] >1700 && ADC_Data[5] < 2800))
	{
		Flag_JOY3 = JOY_R;

	}
	else if((ADC_Data[4] >1800 &&ADC_Data[4] < 2500 )&& (ADC_Data[5] > 1800 &&ADC_Data[5] < 2500 ))
	{
		Flag_JOY3 = JOY_S;	
	
	}


//	if(flag_ad_status_joy3 == 0 && flag_ad_status_joy1 == 0)                 //ҡ��4����ҡ��3
//	{
		if(ADC_Data[6] > 1500 && ADC_Data[7] > 3000)
			Flag_JOY4 = JOY_U;
		else if(ADC_Data[6] > 1800 && ADC_Data[7] < 800)
			Flag_JOY4 = JOY_D;
		else if(ADC_Data[6] < 1000 && ADC_Data[7] >1700)
			Flag_JOY4 = JOY_R;
		else if((ADC_Data[6] >1800 &&ADC_Data[6] < 2500 )&& (ADC_Data[7] > 1800 &&ADC_Data[7] < 2300 ))
			Flag_JOY4 = JOY_S;
		else if(ADC_Data[6] > 3000 && ADC_Data[7] < 2300 )
			Flag_JOY4 = JOY_L;

//	}

}

/********************************************************************************
*********************************************************************************
*******************����ҡ�˲��� ���͸������������´�����С��״̬��ʾ
*********************************************************************************
**********************************************************************************/
	void send_front_cmd(void)//����ǰ�������������
	{
		send_turnstop_cmd();
	
		HAL_UART_Transmit(&huart1, Str_Front1, 9, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Front2, 9, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Front3, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		//HAL_UART_Transmit(&huart1, Str_Front4, 8, 0xFFFF);
			//HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			//HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Front5, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
	void send_back_cmd(void)//���ͺ��������������
	{
		send_turnstop_cmd();
	
	
		HAL_UART_Transmit(&huart1, Str_Back1, 9, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Back2, 9, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Back3, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Back4, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Back5, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
	void send_stop_cmd(void)//����ֹͣ�����������
	{
		send_turnstop_cmd();//ת��ֹͣ
		
		HAL_UART_Transmit(&huart1, Str_Stop1, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Stop2, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
	void send_turnstop_cmd(void)//����ת��ֹͣ�����������
	{
		HAL_UART_Transmit(&huart1, Str_TurnStop, 13, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
	void send_turnleft_cmd(void)//������ת������
	{
		send_turnstop_cmd();
		HAL_UART_Transmit(&huart1, Str_TurnLeft1, 14, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_TurnLeft2, 15, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
	void send_turnright_cmd(void)//������ת�����������
	{
		send_turnstop_cmd();
		HAL_UART_Transmit(&huart1, Str_TurnRight1, 14, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_TurnRight2, 15, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
/*******************************************************************************

����ҡ�˿��� ƽ̨���������߳��շ���

********************************************************************************/

void Send_3_Zoom_Focus(void)//ҡ��4�䱶�;۽�
{
		switch(Flag_JOY3)
		{
			case JOY_U:
						status_JOY3 = JOY_U;                      //����ÿ�ε�״̬
                        
						if(Count_Joy3 < SEND_TIMES)    //����1��
						{
							count_crc( Zoom_Dec_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Zoom_Dec_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
										
						}
						else
							Count_Joy3 = 0;
						Count_Joy3++;						

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy3_d_str);
						#endif 
							
						break;
						
			case JOY_D:
						status_JOY3 = JOY_D;
										
						if(Count_Joy3 < SEND_TIMES)    //����ֻ����1��
						{
							count_crc( Zoom_Add_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Zoom_Add_Cmd, CMD_LEN_7, 0xFFFF);//�������ݸ�����3(PC)
							//HAL_Delay(20);
										
						}
						else
							Count_Joy3 = 0;
						Count_Joy3++;						

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy3_u_str);
						#endif 
						break;
						
			case JOY_L:
						status_JOY3= JOY_L;
						if(Count_Joy3 < SEND_TIMES)    //����1��
						{
							count_crc( Focus_Dec_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Focus_Dec_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
											
						}
						else
							Count_Joy3 = 0;
						Count_Joy3++;						

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy3_r_str);
						#endif 
						break;
						
			case JOY_R:
						status_JOY3 = JOY_R;
					if(Count_Joy3 < SEND_TIMES)    //����1��
						{
							count_crc( Focus_Add_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Focus_Add_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
										
						}
						else
							Count_Joy3 = 0;
						Count_Joy3++;					

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy3_l_str);
						#endif
						break;
						
			case JOY_S://ֹͣ
						//�ж���һ�ε�״̬�����ҡ�˴ӿ���״̬�ظ�������������ֹͣ����
						if(status_JOY3 ==JOY_U || status_JOY3 == JOY_D )
						{
							count_crc( Zoom_Stp_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Zoom_Stp_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
						}
						else if( status_JOY3== JOY_L ||status_JOY3 == JOY_R)
						{
							count_crc(Focus_Stp_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Focus_Stp_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
						}
						status_JOY3 = JOY_S;
						Count_Joy3 = 0;						

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy3_s_str);
						#endif 
						
						break;
						
			default:
						break;
		}
}


/*******************************************************************************

����ҡ�˿��� ƽ̨���������߳��շ���

********************************************************************************/
void Send_1_Place_Line(void)//ҡ��2 ƽ̨���������߳��շ���
{
		switch(Flag_JOY1)
		{
			case JOY_U:
						status_JOY1 = JOY_U;

						//ƽ̨��
						if(Count_Joy1 < SEND_TIMES)    //����1��
						{
							count_crc( Plat_Dw_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Plat_Dw_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);	
									
						}
						else 
							Count_Joy1 = 0;
						Count_Joy1++;
						
						#ifdef PRINT_FLAG
							printf("%s\r\n",joy1_d_str);
						#endif
							
						break;	
						
			case JOY_D:
						status_JOY1 = JOY_D;

						//ƽ̨��
						if(Count_Joy1 < SEND_TIMES)    //����1��
						{
							count_crc( Plat_Up_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Plat_Up_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
										
						}
						else 
							Count_Joy1 = 0;
						Count_Joy1++;						

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy1_u_str);
						#endif
						
						break;

						
			case JOY_L:
						status_JOY1= JOY_L;

						//����
						if(Count_Joy1 < SEND_TIMES)    //����1��
						{
							count_crc( Line_Put_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Line_Put_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
											
						}
						else 
							Count_Joy1 = 0;
						Count_Joy1++;
											 
						#ifdef PRINT_FLAG
							printf("%s\r\n",joy1_r_str);
						#endif
						break;
						
			case JOY_R:
						status_JOY1 = JOY_R;

						//����
						if(Count_Joy1 < SEND_TIMES)    //����1��
						{
							count_crc( Line_Pull_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Line_Pull_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
										
						}
						else 
							Count_Joy1 = 0;
						Count_Joy1++;						

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy1_l_str);
						#endif	
						
						break;
						
			case JOY_S:
						if(status_JOY1 ==JOY_U || status_JOY1 == JOY_D ) //ƽֹ̨ͣ
						{
							count_crc( Plat_Stp_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Plat_Stp_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
						}
						else if(status_JOY1 == JOY_L ||status_JOY1 == JOY_R)//���߳�ֹͣ
						{
							count_crc( Line_Stp_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Line_Stp_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
						}
						
						status_JOY1 = JOY_S;
						Count_Joy1 = 0;						

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy1_s_str);
						#endif
						break;
						
			default:
						break;
		}
}

/*******************************************************************************

����ҡ�˿��� ��������ͷ��������ת��

********************************************************************************/
void Send_4_Angle_Turn(void)      //ң��3 ��������ͷ��������ת��
{
		switch(Flag_JOY4)
		{
			case JOY_U:
						status_JOY4 = JOY_U;

						//��ͷ��ת
						if(Count_Joy4 < SEND_TIMES)	  //����1��
						{
							count_crc( Head_Rgt_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Head_Rgt_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
									
						}
						else
							Count_Joy4 = 0;
						Count_Joy4++;

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy4_r_str);
						#endif
						break;
			
			case JOY_D:
						status_JOY4 = JOY_D;

						//��ͷ��ת
						if(Count_Joy4 < SEND_TIMES)   //����1��
						{
	
							count_crc( Head_Lef_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Head_Lef_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
										
						}
						else
							Count_Joy4 = 0;
						Count_Joy4++;
						
						#ifdef PRINT_FLAG
							printf("%s\r\n",joy4_l_str);
						#endif

			
					
						break;
				
			case JOY_L:
						status_JOY4 = JOY_L;

						//��ͷ�¸�
						if(Count_Joy4 < SEND_TIMES)    //����1��
						{
							count_crc( Lens_Dw_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Lens_Dw_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
										
						}
						else
							Count_Joy4 = 0;
						Count_Joy4++;
						
						#ifdef PRINT_FLAG
							printf("%s\r\n",joy4_d_str);
						#endif
					
						break;
						
			
			case JOY_R:
						status_JOY4 = JOY_R;

						//��ͷ����
						if(Count_Joy4 < SEND_TIMES)    //����1��
						{
							count_crc( Lens_Up_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Lens_Up_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
											
						}
						else
							Count_Joy4 = 0;
						Count_Joy4++;
						
						#ifdef PRINT_FLAG
							printf("%s\r\n",joy4_u_str);
						#endif

						
						break;
						
			case JOY_S://ֹͣ
						
						if(status_JOY4 ==JOY_U || status_JOY4 == JOY_D )
						{
							count_crc( Head_Stp_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Head_Stp_Cmd, CMD_LEN_7, 0xFFFF);
						
							//HAL_Delay(50);
						}
						else if(status_JOY4 == JOY_L ||status_JOY4 == JOY_R)
						{
							count_crc( Lens_Stp_Cmd, CMD_LEN_7);//��ͷֹͣ
							HAL_UART_Transmit(&huart3, Lens_Stp_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
						}
						status_JOY4 = JOY_S;
						Count_Joy4 = 0;

						#ifdef PRINT_FLAG
							printf("%s\r\n",joy4_s_str);
						#endif

						break;
						
			default:
					break;
		}
}



/*******************************************************************************

���� ҡ�˿���С��ǰ��,����,��ת,��ת

********************************************************************************/
void Send_2_Speed_Tor(void)         //ң��1��������		,����1(������)�������������3(PC)
{
		switch(Flag_JOY2)
		{
			case JOY_U:	                
						status_JOY2 = JOY_U;
						
						//���ˣ�����ֵ��Speed_Cmd[4]��Speed_Cmd[5]����
						if(Count_Joy2 < SEND_TIMES)    //����1��
						{
							 speed_value_input(BACK);	 //�ٶȸ�ֵ
						
							 #ifdef PRINT_FLAG
								printf("Speed_Cmd[4]:0x%x,Speed_Cmd[5]:0x%x\r\n",Speed_Cmd[4],Speed_Cmd[5]);
							 #endif
							 
							 count_crc(Speed_Cmd, CMD_LEN_7) ;//���������crcУ��ֵ
							 HAL_UART_Transmit(&huart3,Speed_Cmd,CMD_LEN_7,0xFFFF);
							 send_front_cmd();	  //���������������״̬
										 
							 #ifdef PRINT_FLAG
								printf("enter %s\r\n",joy2_u_str);
							 #endif
										 
						}
						 else
							 Count_Joy2 =0;
						 Count_Joy2++;

										
							#ifdef PRINT_FLAG
								printf("%s\r\n",joy2_d_str);
							#endif


						break;
										
				
			case JOY_D:	
						status_JOY2 = JOY_D;		
		               //�³�ǰ��,ֵ��Speed_Cmd[4]��Speed_Cmd[5]����
					   if(Count_Joy2 < SEND_TIMES)	  //����1��
					   {
							speed_value_input(FRONT);	//�ٶȸ�ֵ

							#ifdef PRINT_FLAG
								printf("Speed_Cmd[4]:0x%x,Speed_Cmd[5]:0x%x\r\n",Speed_Cmd[4],Speed_Cmd[5]);
							#endif
							count_crc(Speed_Cmd, CMD_LEN_7) ;//���������crcУ��ֵ
							HAL_UART_Transmit(&huart3,Speed_Cmd,CMD_LEN_7,0xFFFF);
							send_front_cmd();	 //���������������״̬
							
							#ifdef PRINT_FLAG
								printf("enter %s\r\n",joy2_u_str);
							#endif
							
					   }
						else
							Count_Joy2 =0;
						Count_Joy2++;
						
							#ifdef PRINT_FLAG
								printf("%s\r\n",joy2_u_str);
							#endif

						break;

				
			case JOY_L:	
						status_JOY2 = JOY_L;
						
						//��ת
						if(Count_Joy2 < SEND_TIMES)	  //����1��
						{
							Torque_R_Cmd[5] = SPEED_LEVEL_REV;
							count_crc(Torque_R_Cmd, CMD_LEN_7) ;//���������crcУ��ֵ
							HAL_UART_Transmit(&huart3,Torque_R_Cmd,CMD_LEN_7,0xFFFF);
							//HAL_Delay(50);
							send_turnright_cmd();    //���������������״̬

										
						}
						else
							Count_Joy2 =0;
						Count_Joy2++;
						
						#ifdef PRINT_FLAG
							printf("%s\r\n",joy2_r_str);
						#endif
						break;
									
							
			case JOY_R: 
						status_JOY2 = JOY_R;

						//��ת
						if(Count_Joy2 < SEND_TIMES)	  //����1��
						{
							Torque_L_Cmd[5] = SPEED_LEVEL_REV;
							count_crc(Torque_L_Cmd, CMD_LEN_7) ;//���������crcУ��ֵ
							HAL_UART_Transmit(&huart3,Torque_L_Cmd,CMD_LEN_7,0xFFFF);
							//HAL_Delay(50);
							send_turnleft_cmd();    //���������������״̬

										
						}
						else
							Count_Joy2 =0;
						Count_Joy2++;

						#ifdef PRINT_FLAG
								printf("%s\r\n",joy2_l_str);
						#endif
						break;


			case JOY_S:	//С��ֹͣ
						if(status_JOY2 == JOY_L || status_JOY2 ==JOY_R )
						{
							count_crc(Torque_S_Cmd, CMD_LEN_7) ;
							HAL_UART_Transmit(&huart3,Torque_S_Cmd,CMD_LEN_7,0xFFFF);
							//HAL_Delay(50);
							send_stop_cmd();    //���������������״̬
						}
						else if(status_JOY2 == JOY_U || status_JOY2 ==JOY_D )
						{
							count_crc(Speed_Stp_Cmd, CMD_LEN_7) ;
							HAL_UART_Transmit(&huart3,Speed_Stp_Cmd,CMD_LEN_7,0xFFFF);
							//HAL_Delay(50);
							send_stop_cmd();    //���������������״̬
							printf("u or d stop\r\n");
						
						}
						
						#ifdef PRINT_FLAG
								printf("%s\r\n",joy2_s_str);
						#endif
									
						status_JOY2 = JOY_S;
						Count_Joy2 =0;
						break;

						
			default:	
						break;
							
		}
}


/******************************************************************************
�ٶ����ֵ�����������ٶȵ�λ��Speed_Cmd[4] Speed_Cmd[5]��ֵ
ǰ�����˸���������ֵ����
755~500 ǰ�� ֵԽ�� ǰ��Խ��  ȡ��8����λ
500~0 ���� ֵԽС ����Խ�� ȡ��8����λ
***********************************************************************/
void speed_value_input(uint8_t direction)
{
	uint8_t speed_level = SPEED_LEVEL_REV; //ת���ٶȵ�λ
	speed_level = 7;
	if(direction == FRONT)//ǰ��
	{
		switch(speed_level)
		{
			case 1:	
			case 2:
					Speed_Cmd[4] = SPEED_1_LEVEL[0];
					Speed_Cmd[5] = SPEED_1_LEVEL[1];break;
			case 3:
					Speed_Cmd[4] = SPEED_2_LEVEL[0];
					Speed_Cmd[5] = SPEED_2_LEVEL[1];break;
			case 4:			
					Speed_Cmd[4] = SPEED_3_LEVEL[0];
					Speed_Cmd[5] = SPEED_3_LEVEL[1];break;
			case 5:			
					Speed_Cmd[4] = SPEED_4_LEVEL[0];
					Speed_Cmd[5] = SPEED_4_LEVEL[1];break;
							
			case 6:	Speed_Cmd[4] = SPEED_5_LEVEL[0];
					Speed_Cmd[5] = SPEED_5_LEVEL[1];break;
			case 7:	
					Speed_Cmd[4] = SPEED_6_LEVEL[0];
					Speed_Cmd[5] = SPEED_6_LEVEL[1];break;
			case 8:	
					Speed_Cmd[4] = SPEED_7_LEVEL[0];
					Speed_Cmd[5] = SPEED_7_LEVEL[1];break;										
			case 9:
					Speed_Cmd[4] = SPEED_8_LEVEL[0];
					Speed_Cmd[5] = SPEED_8_LEVEL[1];break;
			default:
					break;
		}
	}
	else if(direction == BACK)//����
	{
		switch(speed_level)
		{
			case 1:	
			case 2:
					Speed_Cmd[4] = SPEED_B1_LEVEL[0];
					Speed_Cmd[5] = SPEED_B1_LEVEL[1];break;
			case 3:
					Speed_Cmd[4] = SPEED_B2_LEVEL[0];
					Speed_Cmd[5] = SPEED_B2_LEVEL[1];break;
			case 4:			
					Speed_Cmd[4] = SPEED_B3_LEVEL[0];
					Speed_Cmd[5] = SPEED_B3_LEVEL[1];break;
			case 5:			
					Speed_Cmd[4] = SPEED_B4_LEVEL[0];
					Speed_Cmd[5] = SPEED_B4_LEVEL[1];break;
							
			case 6:	Speed_Cmd[4] = SPEED_B5_LEVEL[0];
					Speed_Cmd[5] = SPEED_B5_LEVEL[1];break;
			case 7:	
					Speed_Cmd[4] = SPEED_B6_LEVEL[0];
					Speed_Cmd[5] = SPEED_B6_LEVEL[1];break;
			case 8:	
					Speed_Cmd[4] = SPEED_B7_LEVEL[0];
					Speed_Cmd[5] = SPEED_B7_LEVEL[1];break;										
			case 9:
					Speed_Cmd[4] = SPEED_B8_LEVEL[0];
					Speed_Cmd[5] = SPEED_B8_LEVEL[1];break;
			default:
					break;
		}
	}
}

void count_sum_crc(uint8_t *a,uint8_t len)//����crc = �ڶ��ֽ� + �����ڶ��ֽ�
{
	uint8_t i;
	uint8_t crc;
	crc = *(++a);
	for(i=1;i<len-2;i++)
	{
		crc = crc + *(++a);
	}	
	*(++a) = crc;
	//printf("sum crc = 0x%x\n",crc);
}

void count_crc(uint8_t *a,uint8_t len)        //����crc��������ĩβ����У��ֵ //���У��ֵ
{
	uint8_t i;
	uint8_t crc;
	crc = *a;
	for(i=1;i<len-1;i++)
	{
		crc = crc ^ *(++a);
	}	
	*(++a) = crc;
	//printf("new crc = 0x%x\n",crc);
}

