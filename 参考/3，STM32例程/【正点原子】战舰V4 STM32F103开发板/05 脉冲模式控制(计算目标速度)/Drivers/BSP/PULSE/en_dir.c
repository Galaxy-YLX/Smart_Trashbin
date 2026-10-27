/**
 ****************************************************************************************************
 * @file        en_dir.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-07-17
 * @brief       en和dir引脚 驱动代码
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
 * V1.0 20250717
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/PULSE/en_dir.h"


/**
 * @brief       初始化en和dir引脚, 并使能时钟
 * @param       无
 * @retval      无
 */
void en_dir_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    EN_GPIO_CLK_ENABLE();
    DIR_GPIO_CLK_ENABLE();

    gpio_init_struct.Pin = EN_GPIO_PIN;                     /* EN引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(EN_GPIO_PORT, &gpio_init_struct);         /* 初始化LED0引脚 */

    gpio_init_struct.Pin = DIR_GPIO_PIN;                    /* DIR引脚 */
    HAL_GPIO_Init(DIR_GPIO_PORT, &gpio_init_struct);        /* 初始化DIR引脚 */
    
    EN(1);                                                  /* EN置1 */
    DIR(1);                                                 /* DIR置1 */
}
