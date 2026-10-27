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
#include "string.h"
#include "./BSP/LED/led.h"


typedef union
{
    float    f_val;     /* 单精度浮点数 (4字节) */
    uint32_t u32_val;   /* 32位无符号整数 (4字节) */
    uint8_t  bytes[4];  /* 字节数组，用于观察内存布局 */
} converter_t;

int main(void)
{
    uint8_t t = 0;
    
    HAL_Init();                                 /* 初始化HAL库 */
    sys_stm32_clock_init(RCC_PLL_MUL9);         /* 设置时钟,72M */
    delay_init(72);                             /* 初始化延时函数 */
    usart_init(115200);                         /* 串口1初始化波特率为115200 */
    led_init();                                 /* 初始化LED */
    
    converter_t u;                              /* 定义联合体 */
    
    u.u32_val = 0x40970A3D;                     /* 将十六进制值 0x40970A3D 写入联合体 */
    printf("Hex: 0x%08X\r\n", u.u32_val);       /* 打印u32_val的值 */
    printf("Float: %.6f\r\n", u.f_val);         /* 打印f_val浮点数小数点后6位 */

    u.f_val = 600.0f;                           /* 将浮点数 写入联合体 */
    printf("Float: %.6f\r\n", u.f_val);         /* 打印f_val浮点数小数点后6位 */
    printf("Hex: 0x%08X\r\n", u.u32_val);       /* 打印u32_val的值 */
    
    while(1)
    {
        t++;
        if((t % 50) == 0)
        {
            t = 0;
            LED0_TOGGLE();  /* LED0 亮 */
        }
        
        delay_ms(10);
    }
}

