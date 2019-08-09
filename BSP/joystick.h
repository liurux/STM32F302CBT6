#ifndef __JOYSTICK_H_
#define __JOYSTICK_H_

#include "stm32f3xx_hal.h"
#include "adc.h"
#include "usart.h"

extern uint8_t END_CMD[3];
extern uint8_t TORQU_LEVEL;          // ���洮�������͵�ת�ص�λ52 00 13 14 02 XX CRC,   XXΪTORQU_RECV
extern uint8_t TORQU_VALUE;
extern uint8_t END_CMD[3];

void speed_value_input(uint8_t direction);
void HandleAdcData(void);
void Get_Joystick_Staus(void);//��ȡң��״̬
void get_adc_status(void);

void HandleAdcData(void);

void Send_2_Speed_Tor(void);         //ң��1��������		,����1(������)�������������3(PC)

void Send_4_Angle_Turn(void);      //ң��3 ��������ͷ��������ת��

void Send_1_Place_Line(void);//ҡ��2 ƽ̨���������߳��շ���

void Send_3_Zoom_Focus(void);//ҡ��4�䱶�;۽�

void send_front_cmd(void);
void send_back_cmd(void);
void send_stop_cmd(void);
void send_turnstop_cmd(void);
void send_turnleft_cmd(void);
void sned_turnright_cmd(void);

void count_sum_crc(uint8_t *a,uint8_t len);//����crc = �ڶ��ֽ� + �����ڶ��ֽ�
void count_crc(uint8_t *a,uint8_t len);        //����crc��������ĩβ����У��ֵ //���У��ֵ


#endif
