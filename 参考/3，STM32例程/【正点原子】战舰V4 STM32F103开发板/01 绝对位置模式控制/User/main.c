/**
 ****************************************************************************************************
 * @file        main.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-06-25
 * @brief       步进电机驱动器 实验
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
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LED/led.h"
#include "./BSP/SMD/smd.h"
#include "./BSP/SMD/usart2.h"
#include "string.h"
#include "./BSP/SMD/process_frame.h"
#include "./BSP/KEY/key.h"
#include "./BSP/CAN/can.h"


/**
 * @brief    处理应答
 * @param    over_time : 超时时间（单位ms）
 * @retval   0：处理成功，1：等待应答超时
 */
uint8_t handle_ack(uint32_t over_time)
{
    SERIAL_FRAME g_serial_frame;
    
    while(g_rx_frame_flag == false && g_can_frame.frame_done == false)
    {
        delay_ms(1);

        if (--over_time == 0)
        {
            return 1;
        }
    }
    
    if(g_rx_frame_flag == true)
    {
        for(uint8_t i = 0; i < g_rx_len; i++)   /* 打印接收到的数据 */
        {
            printf("%x ", g_rx_cmd[i]);
        }
        printf("\r\n");
        
        serial_frame_process((uint8_t *)g_rx_cmd, g_rx_len, &g_serial_frame);
        
        g_rx_frame_flag = false;                /* 为下次接收做准备 */
    }
    if(g_can_frame.frame_done == true)
    {
        for(uint8_t i = 0; i <  g_can_frame.index; i++)
        {
            printf("%x ", g_can_frame.buf[i]);
        }
        printf("\r\n");
        
        serial_frame_process((uint8_t *)g_can_frame.buf, g_can_frame.index, &g_serial_frame);
        
        g_can_frame.frame_done = false;         /* 为下次接收做准备 */
        g_can_frame.index = 0;
    }
    
    return 0;
}


int main(void)
{
    uint8_t t = 0, key = 0;
    
    HAL_Init();                                 /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* 设置时钟,72M */
    delay_init(72);                             /* 初始化延时函数 */
    usart_init(115200);                         /* 串口1初始化波特率为115200 */
    led_init();                                 /* 初始化LED */
    key_init();                                 /* 初始化KEY */
    usart2_init(115200);                        /* 串口2初始化波特率为115200 */
    can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_NORMAL); /* CAN初始化, 回环模式, 波特率500Kbps */
    
    while(1)
    {
        key = key_scan(0);
        if(key == KEY0_PRES)
        {
            /***********************************************************
            * 绝对位置模式控制
            * 电机地址          ：1
            * 方向              ：0，顺时针
            * 加速度            ：200，加速度
            * 速度              ：6000 RPM，实际速度跟负载大小有关
            * 脉冲数            ：10240000，即200圈
            **********************************************************/
            smd_pos_mode(1, 0, 200, 6000, 10240000);
            handle_ack(50);            /* 处理应答 */
        }
        else if(key == KEY1_PRES)
        {
            smd_read_motor_sta(1);     /* 读取电机运行状态 */
            handle_ack(50);            /* 处理应答 */
            
            smd_read_rotate_speed(1);  /* 读取电机实时转速 */
            handle_ack(50);            /* 处理应答 */
        }
        else if(key == KEY2_PRES)      /* 请在电机运动完成后，等1秒左右到位，再按 */
        {
            smd_read_pos(1);           /* 读取电机实时位置 */
            handle_ack(50);            /* 处理应答 */
            
            smd_read_pos_error(1);     /* 读取电机位置误差 */
            handle_ack(50);            /* 处理应答 */
        }
        else if(key == WKUP_PRES)      /* 请在电机运动完成后，再按 */
        {
            /* 如果没有将当前位置角度清零，在绝对位置模式控制下，当前位置角度值到设定的值后，按KEY0重复设置，电机将不会继续运动 */
            smd_angle_to_zero(1);      /* 将当前位置角度清零（请在电机运动完成后，再清零） */
            handle_ack(50);            /* 处理应答 */
        }
        
        t++;
        if((t % 50) == 0)
        {
            t = 0;
            LED0_TOGGLE();  /* LED0 亮 */
        }
        
        delay_ms(10);
    }
}

