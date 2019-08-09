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
 *          佛祖保佑             永无BUG
 */



#include "joystick.h"
#include "bsp.h"

#define CMD_LEN_7  7
//#define PRINT_FLAG       //printf函数已修改到串口1打印 //usart.c中修改


//摇杆方向
#define JOY_U     1  //上
#define JOY_D     2   //下
#define JOY_L     3   //左
#define JOY_R     4   //右
#define JOY_S     5   //中间


#define SEND_TIMES 1         //摇杆命令发送次数
#define FRONT 1
#define BACK 2


uint8_t send_times = 0;

uint32_t Joy2_Stop_Val = 0;       //保存某摇杆 停止时候的值

uint8_t Speed_Flag = 0;        //保存上一次的速度值

//记录遥感的状态标志，用于保存每次摇杆的操作状态
uint8_t Flag_JOY1 = 0;       
uint8_t Flag_JOY2 = 0;       //
uint8_t Flag_JOY3 = 0;       //
uint8_t Flag_JOY4 = 0;       //

uint8_t flag_ad_status_joy2 = 0;


//记录上一次摇杆状态，用于判断摇杆方向是否改变
uint8_t status_JOY1 = 0; 
uint8_t status_JOY2 = 0; 
uint8_t status_JOY3 = 0; 
uint8_t status_JOY4 = 0;             

//发送摇杆操作指令次数，用于计算摇杆操作后发送的指令次数
uint8_t Count_Joy1 = 0;
uint8_t Count_Joy2 = 0;
uint8_t Count_Joy3 = 0;
uint8_t Count_Joy4 = 0;



/***************************************************************************
MCU要发送给PC的速度
是根据前进后退的摇杆操作时ADC电压值可以分五档，
由五档ADC电压值，匹配到相应不同的速度（速度也分五档）
将匹配的当前速度值保存到速度命令中。

MCU发送给小车速度的指令格式为82 A2 00 2F XX YY CRC   //7字节
XX YY为挡位值，表示两字节的数据//高字节 低字节
等于500时，小车停止运动
大于500， 小车前进，
           前进分五档,值越大速度越快(755最快，超过755时PC当成755处理)
           
小于500 ，小车后退
           后退分五档，速度越小速度越快(0最快)

**************************************************************************/

//当前速度值
uint8_t SPEED_VALUE[2] = {0x0,0x0};

//#define OLD_JOYSTICK//旧的遥感定义

//新摇杆用的，目前采用
//新摇杆的安装方向和以前相反，所以挡位数据互换
//后退速度的挡位值
uint8_t SPEED_B1_LEVEL[2] = {0x01,0x90};//400
uint8_t SPEED_B2_LEVEL[2] = {0x01,0x5E};//350
uint8_t SPEED_B3_LEVEL[2] = {0x01,0x2C};//300
uint8_t SPEED_B4_LEVEL[2] = {0x00,0xC8};//200
uint8_t SPEED_B5_LEVEL[2] = {0x00,0x96};//150
uint8_t SPEED_B6_LEVEL[2] = {0x00,0x50};//80
uint8_t SPEED_B7_LEVEL[2] = {0x00,0x1E};//30
uint8_t SPEED_B8_LEVEL[2] ={0x00,0x00};//0

//前进速度的挡位值，速度分五档发送，值越大，速度越快
//速度值为01 F4（500），小车停止运动
uint8_t SPEED_1_LEVEL[2] = {0x02,0x27};//551----
uint8_t SPEED_2_LEVEL[2] ={0x02,0x5A};//602 ----
uint8_t SPEED_3_LEVEL[2] = {0x02,0x80};//640-----
uint8_t SPEED_4_LEVEL[2] = {0x02,0xA8};//680-----                 //
uint8_t SPEED_5_LEVEL[2] = {0x02,0xBC};//700 //755-----      //速度最大值 755，发送768会当成755
uint8_t SPEED_6_LEVEL[2] = {0x02,0xD0};//720
uint8_t SPEED_7_LEVEL[2] = {0x02,0xE4};//740
uint8_t SPEED_8_LEVEL[2] = {0x03,0x00};//768


//摇杆前进操作时ADC的五个档位值，用于判断达到哪个挡位速度
//uint32_t SPEED_LEVEL[5] = {2380,2800,3200,3600,4000};
uint32_t SPEED_LEVEL[8] = {2380,2500,2800,3200,3400,3600,3800,4000};
//摇杆后退操作时ADC的五档位值，用于判断达到哪个挡位速度
//uint32_t SPEED_LEVEL_B[5] = {50,340,630,920,1210};
uint32_t SPEED_LEVEL_B[8] = {20,50,100,200,340,630,920,1210};





//打印摇杆方向，debug用
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
********MCU发送给串口屏的小车的
/前进/后退/停止/左转/右转/协议字符串**************
******************************************************/
//小车前进
uint8_t Str_Front1[10]="gm0.vid=0";
uint8_t Str_Front2[10]="gm1.vid=1";
uint8_t Str_Front3[9]="gm0.en=1";
uint8_t Str_Front4[9]="gm0.en=1";				    
uint8_t Str_Front5[9]="gm1.en=1";

//小车后退
uint8_t Str_Back1[10]="gm0.vid=2";
uint8_t Str_Back2[10]="gm1.vid=3";
uint8_t Str_Back3[9]="gm1.en=1";
uint8_t Str_Back4[9]="gm0.en=1";
uint8_t Str_Back5[9]="gm1.en=1";

//小车运动停止
uint8_t Str_Stop1[9]="gm0.en=0";
uint8_t Str_Stop2[9]="gm1.en=0";

//小车转向停止
uint8_t Str_TurnStop[15]="main.p3.aph=0";

//小车左转
uint8_t Str_TurnLeft1[20]="main.p3.pic=32";
uint8_t Str_TurnLeft2[20]="main.p3.aph=127";

//小车右转
uint8_t Str_TurnRight1[20]="main.p3.pic=33";
uint8_t Str_TurnRight2[20]="main.p3.aph=127";



//左上摇杆命令：变倍和聚焦
//       变倍+
//        |
//  聚焦--    -- 聚焦+
//	      |
//       变倍-
uint8_t Focus_Stp_Cmd[7]={0x82,0xA2,0x00,0x24,0x00,0x00,0x00};//聚焦停止
uint8_t Focus_Add_Cmd[7]={0x82,0xA2,0x00,0x24,0x00,0x01,0x00};//聚焦+
uint8_t Focus_Dec_Cmd[7]={0x82,0xA2,0x00,0x24,0x00,0x02,0x00};//聚焦-

uint8_t Zoom_Stp_Cmd[7]= {0x82,0xA2,0x00,0x25,0x00,0x00,0x00};//变倍停止
uint8_t Zoom_Add_Cmd[7]={0x82,0xA2,0x00,0x25,0x00,0x01,0x00};//变倍+
uint8_t Zoom_Dec_Cmd[7]={0x82,0xA2,0x00,0x25,0x00,0x02,0x00};//变倍-



//左下摇杆命令：镜头和机头
//        镜头上仰
//            |
//  机头逆转--      -- 机头顺转
//	          |
//        镜头下俯
uint8_t Lens_Stp_Cmd[7]={0x82,0x0A2,0x00,0x04,0x00,0x00,0x00};//镜头上仰下俯停止命令
uint8_t Lens_Up_Cmd[7]={0x82,0x0A2,0x00,0x04,0x00,0x01,0x00};//镜头上仰(左摆)命令
uint8_t Lens_Dw_Cmd[7]={0x82,0x0A2,0x00,0x04,0x00,0x02,0x00};//镜头下俯（右摆）命令

uint8_t Head_Stp_Cmd[7]={0x82,0xA2,0x00,0x03,0x00,0x00,0x00};//机头停止命令
uint8_t Head_Lef_Cmd[7]={0x82,0xA2,0x00,0x03,0x00,0x01,0x00};//机头上旋
uint8_t Head_Rgt_Cmd[7]={0x82,0xA2,0x00,0x03,0x00,0x02,0x00};//机头下旋




//右上摇杆命令：收线车和平台
//       平台升
//        |
//  收线--    -- 防线
//	      |
//       平台降
uint8_t Plat_Stp_Cmd[7]={0x82,0xA2,0x00,0x2A,0x00,0x00,0x00};//平台停止
uint8_t Plat_Up_Cmd[7]={0x82,0xA2,0x00,0x2A,0x00,0x01,0x00};//平台升
uint8_t Plat_Dw_Cmd[7]={0x82,0xA2,0x00,0x2A,0x00,0x02,0x00};//平台降

uint8_t Line_Stp_Cmd[7]={0x82,0xA1,0x00,0x2B,0x00,0x00,0x00};//收线车停止
uint8_t Line_Pull_Cmd[7]={0x82,0xA1,0x00,0x2B,0x00,0x01,0x00};//收线车收线
uint8_t Line_Put_Cmd[7]={0x82,0xA1,0x00,0x2B,0x00,0x02,0x00};//收线车放线



//右下摇杆命令：控制小车方向
//       前进
//        |
//  左转--    -- 右转
//	      |
//      后退
uint8_t Speed_Stp_Cmd[7]={0x82,0xA2,0x00,0x30,0x01,0xF4,0x00};//速度停止命令
//前进后退 82 A2 00 2F XX YY CRC
//XX YY由摇杆操控的力度设定速度同时前进或后退
uint8_t Speed_Cmd[7]={0x82,0xA2,00,0x2F,0x00,0x00,0x00};//前进后退命令 

uint8_t Torque_S_Cmd[7]= {0x82,0xA2,0x00,0x30,0x01,0xF4,0x00};//转向停止命令
uint8_t Torque_L_Cmd[7]={0x82,0xA2,0x00,0x30,0x00,0x00,0x00};//左转命令
uint8_t Torque_R_Cmd[7]={0x82,0xA2,0x00,0x30,0x02,0x00,0x00};//右转命令

//速度挡位命令，发送给PC的，82 A2 00 38 00 XX CRC，摇杆前进后退时同时也改变挡位
uint8_t Speed_Level_Cmd[7]={0x82, 0xA2 , 0x00 , 0x38 , 0x00 , 0x00 , 0x00};


/**************************************************
************************处理摇杆ADC**********************
**********************************************************
*********************************************************/

void HandleAdcData(void)
{

	Get_Joystick_Staus();   //获取遥感状态
	
	Send_1_Place_Line();	
	Send_2_Speed_Tor();         
	Send_3_Zoom_Focus();
	Send_4_Angle_Turn();

}


void Get_Joystick_Staus(void)//获取遥感状态
{
	uint8_t i;
	//uint8_t adc_high[8];
	//uint8_t adc_low[8];
	//uint8_t format=0xFF;

	for(i=0;i<8;i++)
	{
		//adc_high[i] = ADC_Data[i]>>8;				//adc0的高8位数据
	   //adc_low[i] = ADC_Data[i]&0xFF;				 //adc的低8位数据
//		HAL_UART_Transmit(&huart1, &adc_high[i], 1, 0xFFFF);//打印电压值到串口1
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
	if(ADC_Data[0] > 1500 && ADC_Data[1] > 3000)         //摇杆1换成摇杆2
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


	if(ADC_Data[2] > 1500 && ADC_Data[3] > 2400 )         //摇杆2前进后退
	{
		#ifdef PRINT_FLAG
			printf("\nFFFFF\n");
		#endif
		Flag_JOY2 = JOY_U;
//		flag_ad_status_joy2 = JOY_U;

	
		
	}
	else if(ADC_Data[2] > 1800   && ADC_Data[3] < 1500 ) //要换成前进
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
		Joy2_Stop_Val = ADC_Data[3];//保存停止时候的值
	}



	if(ADC_Data[4] > 1500 && ADC_Data[5] > 3000)        //摇杆3换成摇杆1
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


//	if(flag_ad_status_joy3 == 0 && flag_ad_status_joy1 == 0)                 //摇杆4换到摇杆3
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
*******************处理摇杆操作 发送给串口屏，更新串口屏小车状态显示
*********************************************************************************
**********************************************************************************/
	void send_front_cmd(void)//发送前进命令给串口屏
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
	
	void send_back_cmd(void)//发送后退命令给串口屏
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
	
	void send_stop_cmd(void)//发送停止命令给串口屏
	{
		send_turnstop_cmd();//转向停止
		
		HAL_UART_Transmit(&huart1, Str_Stop1, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_Stop2, 8, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
	void send_turnstop_cmd(void)//发送转向停止命令给串口屏
	{
		HAL_UART_Transmit(&huart1, Str_TurnStop, 13, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
	void send_turnleft_cmd(void)//发送左转串口屏
	{
		send_turnstop_cmd();
		HAL_UART_Transmit(&huart1, Str_TurnLeft1, 14, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
		HAL_UART_Transmit(&huart1, Str_TurnLeft2, 15, 0xFFFF);
			HAL_UART_Transmit(&huart1, END_CMD, 3, 0xFFFF);
			HAL_Delay(50);
	}
	
	void send_turnright_cmd(void)//发送右转命令给串口屏
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

左上摇杆控制 平台升降和收线车收放线

********************************************************************************/

void Send_3_Zoom_Focus(void)//摇杆4变倍和聚焦
{
		switch(Flag_JOY3)
		{
			case JOY_U:
						status_JOY3 = JOY_U;                      //保存每次的状态
                        
						if(Count_Joy3 < SEND_TIMES)    //发送1次
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
										
						if(Count_Joy3 < SEND_TIMES)    //命令只发送1次
						{
							count_crc( Zoom_Add_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Zoom_Add_Cmd, CMD_LEN_7, 0xFFFF);//发送数据给串口3(PC)
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
						if(Count_Joy3 < SEND_TIMES)    //发送1次
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
					if(Count_Joy3 < SEND_TIMES)    //发送1次
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
						
			case JOY_S://停止
						//判断上一次的状态，如果摇杆从控制状态回复到不动，则发送停止命令
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

右上摇杆控制 平台升降和收线车收放线

********************************************************************************/
void Send_1_Place_Line(void)//摇杆2 平台升降和收线车收放线
{
		switch(Flag_JOY1)
		{
			case JOY_U:
						status_JOY1 = JOY_U;

						//平台降
						if(Count_Joy1 < SEND_TIMES)    //发送1次
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

						//平台升
						if(Count_Joy1 < SEND_TIMES)    //发送1次
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

						//放线
						if(Count_Joy1 < SEND_TIMES)    //发送1次
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

						//收线
						if(Count_Joy1 < SEND_TIMES)    //发送1次
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
						if(status_JOY1 ==JOY_U || status_JOY1 == JOY_D ) //平台停止
						{
							count_crc( Plat_Stp_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Plat_Stp_Cmd, CMD_LEN_7, 0xFFFF);
							//HAL_Delay(50);
						}
						else if(status_JOY1 == JOY_L ||status_JOY1 == JOY_R)//收线车停止
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

左下摇杆控制 控制摄像头上仰和逆转等

********************************************************************************/
void Send_4_Angle_Turn(void)      //遥感3 控制摄像头上仰和逆转等
{
		switch(Flag_JOY4)
		{
			case JOY_U:
						status_JOY4 = JOY_U;

						//机头右转
						if(Count_Joy4 < SEND_TIMES)	  //发送1次
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

						//机头左转
						if(Count_Joy4 < SEND_TIMES)   //发送1次
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

						//镜头下俯
						if(Count_Joy4 < SEND_TIMES)    //发送1次
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

						//镜头上仰
						if(Count_Joy4 < SEND_TIMES)    //发送1次
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
						
			case JOY_S://停止
						
						if(status_JOY4 ==JOY_U || status_JOY4 == JOY_D )
						{
							count_crc( Head_Stp_Cmd, CMD_LEN_7);
							HAL_UART_Transmit(&huart3, Head_Stp_Cmd, CMD_LEN_7, 0xFFFF);
						
							//HAL_Delay(50);
						}
						else if(status_JOY4 == JOY_L ||status_JOY4 == JOY_R)
						{
							count_crc( Lens_Stp_Cmd, CMD_LEN_7);//镜头停止
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

右下 摇杆控制小车前进,后退,左转,右转

********************************************************************************/
void Send_2_Speed_Tor(void)         //遥感1发送数据		,串口1(串口屏)发送命令给串口3(PC)
{
		switch(Flag_JOY2)
		{
			case JOY_U:	                
						status_JOY2 = JOY_U;
						
						//后退，后退值由Speed_Cmd[4]和Speed_Cmd[5]决定
						if(Count_Joy2 < SEND_TIMES)    //发送1次
						{
							 speed_value_input(BACK);	 //速度赋值
						
							 #ifdef PRINT_FLAG
								printf("Speed_Cmd[4]:0x%x,Speed_Cmd[5]:0x%x\r\n",Speed_Cmd[4],Speed_Cmd[5]);
							 #endif
							 
							 count_crc(Speed_Cmd, CMD_LEN_7) ;//给命令加入crc校验值
							 HAL_UART_Transmit(&huart3,Speed_Cmd,CMD_LEN_7,0xFFFF);
							 send_front_cmd();	  //发送命令给串口屏状态
										 
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
		               //下车前进,值由Speed_Cmd[4]和Speed_Cmd[5]决定
					   if(Count_Joy2 < SEND_TIMES)	  //发送1次
					   {
							speed_value_input(FRONT);	//速度赋值

							#ifdef PRINT_FLAG
								printf("Speed_Cmd[4]:0x%x,Speed_Cmd[5]:0x%x\r\n",Speed_Cmd[4],Speed_Cmd[5]);
							#endif
							count_crc(Speed_Cmd, CMD_LEN_7) ;//给命令加入crc校验值
							HAL_UART_Transmit(&huart3,Speed_Cmd,CMD_LEN_7,0xFFFF);
							send_front_cmd();	 //发送命令给串口屏状态
							
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
						
						//右转
						if(Count_Joy2 < SEND_TIMES)	  //发送1次
						{
							Torque_R_Cmd[5] = SPEED_LEVEL_REV;
							count_crc(Torque_R_Cmd, CMD_LEN_7) ;//给命令加入crc校验值
							HAL_UART_Transmit(&huart3,Torque_R_Cmd,CMD_LEN_7,0xFFFF);
							//HAL_Delay(50);
							send_turnright_cmd();    //发送命令给串口屏状态

										
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

						//左转
						if(Count_Joy2 < SEND_TIMES)	  //发送1次
						{
							Torque_L_Cmd[5] = SPEED_LEVEL_REV;
							count_crc(Torque_L_Cmd, CMD_LEN_7) ;//给命令加入crc校验值
							HAL_UART_Transmit(&huart3,Torque_L_Cmd,CMD_LEN_7,0xFFFF);
							//HAL_Delay(50);
							send_turnleft_cmd();    //发送命令给串口屏状态

										
						}
						else
							Count_Joy2 =0;
						Count_Joy2++;

						#ifdef PRINT_FLAG
								printf("%s\r\n",joy2_l_str);
						#endif
						break;


			case JOY_S:	//小车停止
						if(status_JOY2 == JOY_L || status_JOY2 ==JOY_R )
						{
							count_crc(Torque_S_Cmd, CMD_LEN_7) ;
							HAL_UART_Transmit(&huart3,Torque_S_Cmd,CMD_LEN_7,0xFFFF);
							//HAL_Delay(50);
							send_stop_cmd();    //发送命令给串口屏状态
						}
						else if(status_JOY2 == JOY_U || status_JOY2 ==JOY_D )
						{
							count_crc(Speed_Stp_Cmd, CMD_LEN_7) ;
							HAL_UART_Transmit(&huart3,Speed_Stp_Cmd,CMD_LEN_7,0xFFFF);
							//HAL_Delay(50);
							send_stop_cmd();    //发送命令给串口屏状态
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
速度命令赋值函数，根据速度挡位给Speed_Cmd[4] Speed_Cmd[5]赋值
前进后退根据这两个值决定
755~500 前进 值越大 前进越快  取了8个挡位
500~0 后退 值越小 后退越快 取了8个挡位
***********************************************************************/
void speed_value_input(uint8_t direction)
{
	uint8_t speed_level = SPEED_LEVEL_REV; //转存速度挡位
	speed_level = 7;
	if(direction == FRONT)//前进
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
	else if(direction == BACK)//后退
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

void count_sum_crc(uint8_t *a,uint8_t len)//计算crc = 第二字节 + 倒数第二字节
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

void count_crc(uint8_t *a,uint8_t len)        //计算crc并在数组末尾加入校验值 //异或校验值
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

