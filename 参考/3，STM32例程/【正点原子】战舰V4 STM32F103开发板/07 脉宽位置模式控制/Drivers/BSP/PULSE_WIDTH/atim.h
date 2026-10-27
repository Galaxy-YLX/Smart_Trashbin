/**
 ****************************************************************************************************
 * @file        atim.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2020-04-21
 * @brief       高级定时器 驱动代码
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
 * V1.0 20211217
 * 第一次发布
 *
 ****************************************************************************************************
 */

#ifndef __ATIM_H
#define __ATIM_H

#include "./SYSTEM/sys/sys.h"


extern TIM_HandleTypeDef g_timx_pwm_chy_handle;     /* 定时器x句柄 */

/******************************************************************************************/
/* 高级定时器 定义 */

/* TIMX 输出指定个数PWM 定义 
 * 这里输出的PWM通过PC6(TIM8_CH1)输出
 * 默认使用的是TIM8_CH1
 * 注意: 通过修改这几个宏定义, 可以支持TIM1/TIM8定时器, 任意一个IO口输出指定个数的PWM
 */
#define ATIM_TIMX_PWM_CHY_GPIO_PORT            GPIOC
#define ATIM_TIMX_PWM_CHY_GPIO_PIN             GPIO_PIN_6
#define ATIM_TIMX_PWM_CHY_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)  /* PC口时钟使能 */

#define ATIM_TIMX_PWM                          TIM8
#define ATIM_TIMX_PWM_CHY                      TIM_CHANNEL_1                                 /* 通道Y,  1<= Y <=4 */
#define ATIM_TIMX_PWM_CHY_CCRX                 TIM8->CCR1                                    /* 通道Y的输出比较寄存器 */
#define ATIM_TIMX_PWM_CHY_CLK_ENABLE()         do{ __HAL_RCC_TIM8_CLK_ENABLE(); }while(0)    /* TIM8 时钟使能 */

/******************************************************************************************/

void atim_timx_pwm_chy_init(uint16_t arr, uint16_t psc);   /* 高级定时器 PWM初始化函数 */


#endif

















