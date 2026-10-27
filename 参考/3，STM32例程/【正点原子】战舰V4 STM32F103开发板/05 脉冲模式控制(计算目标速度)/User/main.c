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
#include "./BSP/PULSE/atim.h"
#include "./BSP/PULSE/en_dir.h"


/* 步进电机基本参数 */
#define BASE_STEPS_PER_REV  200  /* 1.8°步进角的步进电机每旋转一圈对应200步（360/1.8） */

/**
 * @brief   计算达到目标转速所需的脉冲频率（Hz）
 * @param   step       : 细分数（1-256）
 * @param   rpm        : 目标转速（转/分钟）
 * @retval  脉冲频率（Hz），如果输入无效返回-1
 */
float calculate_frequency_from_rpm(uint16_t step, float rpm)
{
    if (step < 1 || step > 256 || rpm <= 0)
    {
        return -1;  /* 无效输入 */
    }
    
    /* 公式： 频率 = (RPM * 每圈总步数) / 60 */
    return (rpm * BASE_STEPS_PER_REV * step) / 60.0f;
}

/**
 * @brief   计算当前脉冲频率对应的实际转速（RPM）
 * @param   frequency  : 脉冲频率（Hz）
 * @param   step       : 细分数（1-256）
 * @retval  实际转速（转/分钟），如果输入无效返回-1
 */
float calculate_rpm_from_frequency(float frequency, uint16_t step)
{
    if (step < 1 || step > 256 || frequency <= 0)
    {
        return -1;  /* 无效输入 */
    }
    
    /* 公式: RPM = (频率 * 60) / (每圈总步数) */
    return (frequency * 60.0f) / (BASE_STEPS_PER_REV * step);
}

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
    en_dir_init();                              /* 初始化EN和DIR引脚 */
    can_init(CAN_SJW_1TQ, CAN_BS2_8TQ, CAN_BS1_9TQ, 4, CAN_MODE_NORMAL); /* CAN初始化, 回环模式, 波特率500Kbps */
    
    /* 假设脉冲频率10khz，16细分，计算目标速度 */
    float actual_rpm = calculate_rpm_from_frequency(10000, 16);   /* 10khz的PWM频率，16细分 */
    printf("在 16 细分下， 10K Hz 脉冲对应转速: %.1f RPM\r\n", actual_rpm);
    
    /***********************************************************
    * 脉冲模式
    * 电机地址          ：1
    * 说明：EN引脚用于使能脉冲模式控制，STP引脚用于输入脉冲，DIR用于控制电机旋转方向
    **********************************************************/
    smd_pulse_mode(1);                          /* 脉冲模式 */
    handle_ack(50);                             /* 处理应答 */
    
    smd_set_step(1, 16);                        /* 设置细分 */
    handle_ack(50);                             /* 处理应答 */
    
    smd_set_en_level(1, 1);                     /* 设置EN引脚高电平有效 */
    handle_ack(50);                             /* 处理应答 */
    EN(1);                                      /* 设置EN引脚高电平 */
    
    smd_set_dir_level(1, 0);                    /* 设置DIR引脚高电平正转 */
    handle_ack(50);                             /* 处理应答 */
    DIR(1);                                     /* 设置DIR引脚高电平 */
    
    while(1)
    {
        key = key_scan(0);
        if(key == KEY0_PRES)
        {
            smd_angle_to_zero(1);       /* 将当前位置角度清零 */
            handle_ack(50);             /* 处理应答 */
            
            atim_timx_npwm_chy_init(100 - 1, 72 - 1);  /* 1Mhz的计数频率,10khz的PWM频率 */
            atim_timx_npwm_chy_set(32000);             /* 设置输出脉冲数（16细分下，一圈脉冲数为16 * BASE_STEPS_PER_REV） */
            printf("输出32000个脉冲，16细分下，即转10圈\r\n");
        }
        else if(key == KEY1_PRES)
        {
            EN_TOGGLE();                /* 翻转EN引脚电平 */
        }
        else if(key == KEY2_PRES)
        {
            DIR_TOGGLE();               /* 翻转DIR引脚电平 */
        }
        else if(key == WKUP_PRES)
        {
            smd_read_motor_sta(1);      /* 读取电机运行状态 */
            handle_ack(50);             /* 处理应答 */
            
            smd_read_rotate_speed(1);   /* 读取电机实时转速 */
            handle_ack(50);             /* 处理应答 */
            
            smd_read_pos(1);            /* 读取电机实时位置 */
            handle_ack(50);             /* 处理应答 */
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


