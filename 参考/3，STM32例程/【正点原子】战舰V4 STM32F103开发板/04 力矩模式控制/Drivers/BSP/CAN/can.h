/**
 ****************************************************************************************************
 * @file        can.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-24
 * @brief       CAN 驱动代码
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
 * V1.0 20200424
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __CAN_H
#define __CAN_H

#include "./SYSTEM/sys/sys.h"
#include <stdbool.h>

/******************************************************************************************/
/* CAN 引脚 定义 */

#define CAN_RX_GPIO_PORT                GPIOA
#define CAN_RX_GPIO_PIN                 GPIO_PIN_11
#define CAN_RX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

#define CAN_TX_GPIO_PORT                GPIOA
#define CAN_TX_GPIO_PIN                 GPIO_PIN_12
#define CAN_TX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PA口时钟使能 */

/******************************************************************************************/
#define CAN_RECV_BUF_LEN               256
/* CAN接收数据相关结构体 */
typedef struct 
{
    uint8_t  buf[CAN_RECV_BUF_LEN];     /* 数据缓冲区 */
    uint16_t index;                     /* 当前写入索引 */
    __IO bool frame_done;               /* 一帧完成标志 */
} can_frame_t;

extern  can_frame_t g_can_frame;        /* 全局CAN帧接收管理变量 */
/* CAN接收RX0中断使能 */
#define CAN_RX0_INT_ENABLE      1       /* 0,不使能; 1,使能; */

/* 对外接口函数 */
uint8_t can_receive_msg(uint32_t id, uint8_t *buf);             /* CAN接收数据, 查询 */
uint8_t can_send_msg(uint32_t id, uint8_t *msg, uint8_t len);   /* CAN发送数据 */
uint8_t can_init(uint32_t tsjw,uint32_t tbs2,uint32_t tbs1,uint16_t brp,uint32_t mode); /* CAN初始化 */
uint8_t can_send_long_msg(uint32_t id, uint8_t *data, uint16_t len);
#endif

















