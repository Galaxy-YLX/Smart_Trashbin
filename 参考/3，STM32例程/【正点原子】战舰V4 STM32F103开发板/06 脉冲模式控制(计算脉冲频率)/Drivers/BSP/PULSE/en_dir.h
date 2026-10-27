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

#ifndef _EN_DIR_H
#define _EN_DIR_H
#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* 引脚 定义 */

//#define EN_GPIO_PORT                  GPIOA
//#define EN_GPIO_PIN                   GPIO_PIN_2
//#define EN_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define EN_GPIO_PORT                  GPIOG
#define EN_GPIO_PIN                   GPIO_PIN_7
#define EN_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)

#define DIR_GPIO_PORT                 GPIOC
#define DIR_GPIO_PIN                  GPIO_PIN_8
#define DIR_GPIO_CLK_ENABLE()         do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

/******************************************************************************************/
/* 端口定义 */
#define EN(x)   do{ x ? \
                      HAL_GPIO_WritePin(EN_GPIO_PORT, EN_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(EN_GPIO_PORT, EN_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)

#define DIR(x)   do{ x ? \
                      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_SET) : \
                      HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, GPIO_PIN_RESET); \
                  }while(0)

/* 取反定义 */
#define EN_TOGGLE()   do{ HAL_GPIO_TogglePin(EN_GPIO_PORT, EN_GPIO_PIN); }while(0)
#define DIR_TOGGLE()  do{ HAL_GPIO_TogglePin(DIR_GPIO_PORT, DIR_GPIO_PIN); }while(0)
/******************************************************************************************/
/* 外部接口函数*/
void en_dir_init(void);         /* 初始化 */

#endif
