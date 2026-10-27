/**
 ****************************************************************************************************
 * @file        usart2.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.1
 * @date        2025-06-25
 * @brief       串口初始化代码(一般是串口1)，支持printf
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20250625
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __USART2_H
#define __USART2_H

#include "stdio.h"
#include "stdbool.h"
#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 引脚 和 串口 定义 
 * 默认是针对USART1的.
 * 注意: 通过修改这几个宏定义,可以支持USART1~UART5任意一个串口.
 */
#define USART2_TX_GPIO_PORT                  GPIOA
#define USART2_TX_GPIO_PIN                   GPIO_PIN_2
#define USART2_TX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define USART2_RX_GPIO_PORT                  GPIOA
#define USART2_RX_GPIO_PIN                   GPIO_PIN_3
#define USART2_RX_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define USART2_UX                            USART2
#define USART2_UX_IRQn                       USART2_IRQn
#define USART2_UX_IRQHandler                 USART2_IRQHandler
#define USART2_UX_CLK_ENABLE()               do{ __HAL_RCC_USART2_CLK_ENABLE(); }while(0)  /* USART2 时钟使能 */

#define RS485_RE_GPIO_PORT                  GPIOD
#define RS485_RE_GPIO_PIN                   GPIO_PIN_7
#define RS485_RE_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)   /* PD口时钟使能 */


/******************************************************************************************/
/* 控制RS485_RE脚, 控制RS485发送/接收状态
 * RS485_RE = 0, 进入接收模式
 * RS485_RE = 1, 进入发送模式
 */
#define RS485_RE(x)   do{ x ? \
                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_SET) : \
                          HAL_GPIO_WritePin(RS485_RE_GPIO_PORT, RS485_RE_GPIO_PIN, GPIO_PIN_RESET); \
                      }while(0)

                      
#define RX_BUFFER_SIZE   128

extern __IO uint8_t g_rx_cmd[RX_BUFFER_SIZE];     /* 存放接收到的指令 */
extern __IO uint8_t g_rx_count;                   /* 接收到的数据长度 */
extern __IO bool g_rx_frame_flag;                 /* 接收一帧数据标志位 */
extern __IO uint8_t g_rx_len;                     /* 接收到的数据长度 */
extern UART_HandleTypeDef g_uart2_handle;         /* HAL UART句柄 */




void usart2_init(uint32_t bound);                 /* 串口初始化函数 */
void usart2_send_cmd(uint8_t *data, uint8_t len); /* 串口发送发送多个字节数据 */



#endif


