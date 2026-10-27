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
#include "./BSP/PULSE_WIDTH/atim.h"


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
    int16_t angle = 30;
    
    HAL_Init();                                 /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* 设置时钟,72M */
    delay_init(72);                             /* 初始化延时函数 */
    usart_init(115200);                         /* 串口1初始化波特率为115200 */
    led_init();                                 /* 初始化LED */
    key_init();                                 /* 初始化KEY */
    usart2_init(115200);                        /* 串口2初始化波特率为115200 */
    can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_NORMAL); /* CAN初始化, 回环模式, 波特率500Kbps */
    
    /***********************************************************
    * 脉宽速度模式
    * 电机地址              ：1
    * 高电平脉宽最长长度     ：3300 us，切忌不能设置为0
    * 高电平脉宽最小长度     ：300 us，切忌不能设置为0
    * 脉宽最长长度对应的位置 ：6000 rpm
    * 脉宽最小长度对应的位置 ：0 rpm
    * 
    * 这里自定义PWM的占空比和电流关系如下：
      PWM的频率设置为100Hz(即周期为10ms)
      高电平时间   占空比   电流
      0.3ms       3%       0 rpm
      0.6ms       6%       600 rpm
      0.9ms       9%       1200 rpm
      1.2ms       12%      1800 rpm
      1.5ms       15%      2400 rpm
      1.8ms       18%      3000 rpm
      2.1ms       21%      3600 rpm
      2.4ms       24%      4200 rpm
      2.7ms       27%      4800 rpm
      3.0ms       30%      5400 rpm
      3.3ms       33%      6000 rpm
    **********************************************************/
    smd_pulse_width_speed_mode(1, 3300, 300, 6000, 0);  /* 脉宽力矩模式 */
    handle_ack(50);                                     /* 处理应答 */
    
    atim_timx_pwm_chy_init(1000 - 1, 720 - 1);          /* 100Khz的计数频率,100hz的PWM频率 */
    
    while(1)
    {
        key = key_scan(0);
        if(key == KEY0_PRES)
        {
            angle += 30;
            if (angle > 330)
            {
                angle = 330;
            }
            printf("占空比为 %.1f %% \r\n", (float)angle/10.0);
            
            /* 修改比较值控制占空比 */
            __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY, angle);
        }
        else if(key == KEY1_PRES)
        {
            angle -= 30;
            if (angle < 30)
            {
                angle = 30;
            }
            printf("占空比为 %.1f %% \r\n", (float)angle/10.0);
            
            /* 修改比较值控制占空比 */
            __HAL_TIM_SET_COMPARE(&g_timx_pwm_chy_handle, ATIM_TIMX_PWM_CHY, angle);
        }
        else if(key == KEY2_PRES)
        {
            smd_read_rotate_speed(1);                   /* 读取电机实时转速 */
            handle_ack(50);                             /* 处理应答 */
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

