/**
 ****************************************************************************************************
 * @file        io_start_stop.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-10-10
 * @brief       IO启停 驱动代码
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
 * V1.0 20251010
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/IO_START_STOP/io_start_stop.h"


/**
 * @brief       初始化IO启停相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void io_start_stop_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    IO_START_STOP_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = IO_START_STOP_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(IO_START_STOP_GPIO_PORT, &gpio_init_struct);  /* 初始化LED1引脚 */

    IO_START_STOP(0);   /* 默认输出低电平 */
}
