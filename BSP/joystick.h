#ifndef __JOYSTICK_H_
#define __JOYSTICK_H_

#include "stm32f3xx_hal.h"
#include "adc.h"
#include "usart.h"

extern uint8_t END_CMD[3];
extern uint8_t TORQU_LEVEL;          // 保存串口屏发送的转矩挡位52 00 13 14 02 XX CRC,   XX为TORQU_RECV
extern uint8_t TORQU_VALUE;
extern uint8_t END_CMD[3];

void speed_value_input(uint8_t direction);
void HandleAdcData(void);
void Get_Joystick_Staus(void);//获取遥感状态
void get_adc_status(void);

void HandleAdcData(void);

void Send_2_Speed_Tor(void);         //遥感1发送数据		,串口1(串口屏)发送命令给串口3(PC)

void Send_4_Angle_Turn(void);      //遥感3 控制摄像头上仰和逆转等

void Send_1_Place_Line(void);//摇杆2 平台升降和收线车收放线

void Send_3_Zoom_Focus(void);//摇杆4变倍和聚焦

void send_front_cmd(void);
void send_back_cmd(void);
void send_stop_cmd(void);
void send_turnstop_cmd(void);
void send_turnleft_cmd(void);
void sned_turnright_cmd(void);

void count_sum_crc(uint8_t *a,uint8_t len);//计算crc = 第二字节 + 倒数第二字节
void count_crc(uint8_t *a,uint8_t len);        //计算crc并在数组末尾加入校验值 //异或校验值


#endif
