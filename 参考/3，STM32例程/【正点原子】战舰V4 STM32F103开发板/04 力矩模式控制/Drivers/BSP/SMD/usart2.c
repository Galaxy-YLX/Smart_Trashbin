/**
 ****************************************************************************************************
 * @file        usart2.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.1
 * @date        2025-06-25
 * @brief       串口驱动代码
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
 * V1.0 20250625
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "./BSP/SMD/usart2.h"



__IO uint8_t g_rx_cmd[RX_BUFFER_SIZE];      /* 存放接收到的指令 */
__IO uint8_t g_rx_len;                      /* 接收到的数据长度 */
__IO bool g_rx_frame_flag  = false;         /* 接收一帧数据标志位 */

UART_HandleTypeDef g_uart2_handle;          /* UART句柄 */
DMA_HandleTypeDef g_dma_usart2_rx;          /* DMA句柄串口接收 */
DMA_HandleTypeDef g_dma_usart2_tx;          /* DMA句柄串口发送 */

/**
 * @brief       串口2初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart2_init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_init_struct;

    RS485_RE_GPIO_CLK_ENABLE();                              /* 使能 RS485_RE 脚时钟 */
    USART2_TX_GPIO_CLK_ENABLE();                             /* 使能串口TX脚时钟 */
    USART2_RX_GPIO_CLK_ENABLE();                             /* 使能串口RX脚时钟 */
    USART2_UX_CLK_ENABLE();                                  /* 使能串口时钟 */

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    
    gpio_init_struct.Pin = USART2_TX_GPIO_PIN;               /* 串口发送引脚号 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;                 /* 复用推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                     /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;           /* IO速度设置为高速 */
    HAL_GPIO_Init(USART2_TX_GPIO_PORT, &gpio_init_struct);
            
    gpio_init_struct.Pin = USART2_RX_GPIO_PIN;               /* 串口RX脚 模式设置 */
    gpio_init_struct.Mode = GPIO_MODE_AF_INPUT;    
    HAL_GPIO_Init(USART2_RX_GPIO_PORT, &gpio_init_struct);   /* 串口RX脚 必须设置成输入模式 */

    gpio_init_struct.Pin = RS485_RE_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485_RE_GPIO_PORT, &gpio_init_struct);    /* RS485_RE 脚 模式设置 */
    
   
    /* USART2_RX DMA Init */
    g_dma_usart2_rx.Instance = DMA1_Channel6;
    g_dma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_dma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_dma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_dma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_dma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_dma_usart2_rx.Init.Mode = DMA_NORMAL;
    g_dma_usart2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    HAL_DMA_Init(&g_dma_usart2_rx);

    __HAL_LINKDMA(&g_uart2_handle, hdmarx, g_dma_usart2_rx);

    /* USART2_TX DMA Init */
    g_dma_usart2_tx.Instance = DMA1_Channel7;
    g_dma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    g_dma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_dma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    g_dma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_dma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_dma_usart2_tx.Init.Mode = DMA_NORMAL;
    g_dma_usart2_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
    HAL_DMA_Init(&g_dma_usart2_tx);

    __HAL_LINKDMA(&g_uart2_handle, hdmatx, g_dma_usart2_tx);


    /*UART 初始化设置*/
    g_uart2_handle.Instance = USART2_UX;                        /* USART2_UX */
    g_uart2_handle.Init.BaudRate = baudrate;                    /* 波特率 */
    g_uart2_handle.Init.WordLength = UART_WORDLENGTH_8B;        /* 字长为8位数据格式 */
    g_uart2_handle.Init.StopBits = UART_STOPBITS_1;             /* 一个停止位 */
    g_uart2_handle.Init.Parity = UART_PARITY_NONE;              /* 无奇偶校验位 */
    g_uart2_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* 无硬件流控 */
    g_uart2_handle.Init.Mode = UART_MODE_TX_RX;                 /* 收发模式 */
    g_uart2_handle.Init.OverSampling = UART_OVERSAMPLING_16;    /* 16倍过采样 */
    HAL_UART_Init(&g_uart2_handle);                             /* HAL_UART_Init()会使能UART1 */

    HAL_NVIC_EnableIRQ(USART2_UX_IRQn);                         /* 使能USART1中断通道 */
    HAL_NVIC_SetPriority(USART2_UX_IRQn, 1, 1);                 /* 组2，最低优先级:抢占优先级1，子优先级1 */


    __HAL_UART_CLEAR_IDLEFLAG(&g_uart2_handle);                 /* 清除IDLE标志 */
    __HAL_UART_ENABLE_IT(&g_uart2_handle, UART_IT_IDLE);        /* 使能串UART1 IDLE中断 */
    
    HAL_UART_Receive_DMA(&g_uart2_handle, (uint8_t *)g_rx_cmd, RX_BUFFER_SIZE); /* 开启DMA接收模式 */

     RS485_RE(0); /* 默认为接收模式 */
}



/**
 * @brief       串口发送发送多个字节数据
 * @param       *cmd: 数据指针
 * @param       len: 长度
 * @retval      无
 */
void usart2_send_cmd(uint8_t *data, uint8_t len)
{
    RS485_RE(1);
    HAL_UART_Transmit_DMA(&g_uart2_handle, data, len);
    uint32_t timeout = 100000;                  /* 可适当调整 */
    while (timeout--)
    {
        if (__HAL_DMA_GET_FLAG(&g_dma_usart2_tx, DMA_FLAG_TC7))
        {
            __HAL_DMA_CLEAR_FLAG(&g_dma_usart2_tx, DMA_FLAG_TC7);
            RS485_RE(0);                        /* 进入接收模式 */
            HAL_UART_DMAStop(&g_uart2_handle);  /* 停止DMA */
            HAL_UART_Receive_DMA(&g_uart2_handle, (uint8_t *)g_rx_cmd, RX_BUFFER_SIZE);
            break;
        }
    }
}

/**
 * @brief       串口2中断服务函数
 * @param       无
 * @retval      无
 */
void USART2_UX_IRQHandler(void)
{
    if(__HAL_UART_GET_FLAG(&g_uart2_handle, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&g_uart2_handle);     /* 清除IDLE标志 */

        HAL_UART_DMAStop(&g_uart2_handle);              /* 停止DMA */

        g_rx_len = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&g_dma_usart2_rx);
         
        g_rx_frame_flag = true;                         /* 置位一帧命令接收完毕标志位 */
        
        HAL_UART_Receive_DMA(&g_uart2_handle, (uint8_t *)g_rx_cmd, RX_BUFFER_SIZE);
    }
}

