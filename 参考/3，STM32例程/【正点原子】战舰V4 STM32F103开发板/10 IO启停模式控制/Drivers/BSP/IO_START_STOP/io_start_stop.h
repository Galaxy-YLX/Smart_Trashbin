/**
 ****************************************************************************************************
 * @file        io_start_stop.h
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

#ifndef _IO_START_STOP_H
#define _IO_START_STOP_H
#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 引脚 定义 */

#define IO_START_STOP_GPIO_PORT                  GPIOA
#define IO_START_STOP_GPIO_PIN                   GPIO_PIN_8
#define IO_START_STOP_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

/******************************************************************************************/
/* LED端口定义 */
#define IO_START_STOP(x)   do{ x ? \
                      HAL_GPIO_WritePin(IO_START_STOP_GPIO_PORT, IO_START_STOP_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(IO_START_STOP_GPIO_PORT, IO_START_STOP_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)


/* 取反定义 */
#define IO_START_STOP_TOGGLE()   do{ HAL_GPIO_TogglePin(IO_START_STOP_GPIO_PORT, IO_START_STOP_GPIO_PIN); }while(0)

/******************************************************************************************/
/* 外部接口函数*/
void io_start_stop_init(void);    /* IO启停初始化 */

#endif
