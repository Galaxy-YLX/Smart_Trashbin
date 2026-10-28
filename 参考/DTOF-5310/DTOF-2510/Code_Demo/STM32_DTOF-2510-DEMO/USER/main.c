//******************************************************************************//	 
// File           : STM32_DTOF-2510
// Generated Date : 2026-03-10 10:30:51
// Autor          : 大蜗牛
// Script Mark    : ['434']
// Arguments      : STM32_DTOF-2510 Demo
// Version        : V1.0
//Copyright(C)    : CXTC  2026-2036
//All rights reserved
//******************************************************************************//		  

#include "stm32f10x.h"
#include "delay.h"
#include "bsp_usart.h"
#include <stdio.h>

extern uint8_t data_buff[5]; //备份buf Backup buf

int main()
{
	
	SystemInit();	
	delay_init();
	
	USART1_init(115200);  //接PC的串口 PA9(TX) PA10(RX)  Connect to PC's serial port
	USART2_init(921600);  //接测距模块PA2(R) PA3(T)    Connect the distance measurement modules PA2(RX) and PA3(TX)
  delay_ms(100);
	printf("Distance:%dmm, Confidence:%d\r\n");


while(1) {
		//接收到数据就做打印处理 Upon receiving the data, print it out
    if(data_ready) {
        uint16_t dist = distance_value;
        uint16_t conf = confidence_value;
        data_ready = 0;
        printf("Distance:%dmm, Confidence:%d\r\n", dist, conf);
    }
		//*数据直接可以调用distance_value测距数值、confidence_value置信度数值来嵌入自己系统   ******//
}
}

