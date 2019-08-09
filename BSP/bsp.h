#ifndef __BSP_H_
#define __BSP_H_

#include "stm32f3xx_hal.h"
#include "adc.h"
#include "usart.h"



extern uint8_t Uart1RX_Data,Uart3RX_Data;
extern uint32_t ADC_Data[9];
extern uint8_t SPEED_LEVEL_REV;

//extern uint8_t END_CMD[3]={0xFF,0xFF,0xFF};
//extern uint8_t SPEED_LEVEL=0;          //保存串口屏发送的速度挡位52 00 13 14 01 XX CRC,   XX为SPEED_RECV
//extern uint8_t TORQU_LEVEL=0;          // 保存串口屏发送的转矩挡位52 00 13 14 02 XX CRC,   XX为TORQU_RECV
//extern uint8_t SPEED_VALUE=0;
//extern uint8_t TORQU_VALUE=0;

void Usart1_Data_Recv(void);          //串口1接收从串口屏发送的数据
void Usart3_Data_Recv(void);


uint8_t check_crc(uint8_t a[], uint8_t len);
void HandleUsart1Screen(void);
//uint8_t check_sum_crc(uint8_t *a,uint8_t len);
void HandleUsart3Pc(void);
void Send_Press_Fun(void);
void Send_FrnAgle_Fun(void);
void Send_SideAgle_Fun(void);
void Send_Height_Fun(void);
void Send_Length_Fun(void);

void Send_Speed_Fun(void);
void Send_Level_Fun(void);


void mydeleteChar(uint8_t *str, char c, uint8_t *len);


extern void ADC_DataLinkInit(void);
extern void USART_IT_Start(void);



#endif



