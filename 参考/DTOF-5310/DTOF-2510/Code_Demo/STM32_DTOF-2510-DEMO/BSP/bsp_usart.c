#include "bsp_usart.h"
#include <stdlib.h>   // for atof, atoi
#include <string.h>   // for memcpy

#define DISTANCE_MIN     20      // 最小有效距离 (mm)
#define DISTANCE_MAX     10000   // 最大有效距离 (mm)   // 修改为10000
#define CONFIDENCE_MAX   1000    // 最大置信度          // 修改为1000

volatile uint16_t distance_value = 0;       // 解析得到的距离值(20-10000)单位mm
volatile uint16_t confidence_value = 0;      // 解析得到的置信度（0-1000） // 类型改为uint16_t
volatile uint8_t data_ready = 0;            // 接收完成标志

// USART1初始化 - 与PC通信，输出调试信息
void USART1_init(u32 baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
    GPIO_Init(GPIOA, &GPIO_InitStructure);    

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
      
    USART_InitStructure.USART_BaudRate = baudrate;        
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
    USART_InitStructure.USART_StopBits = USART_StopBits_1;     
    USART_InitStructure.USART_Parity = USART_Parity_No;         
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure); 
    
    USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);   
    
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_Cmd(USART1, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                       // USART1中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;               // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;                      // 子优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
    NVIC_Init(&NVIC_InitStructure);
}

// 使用串口2接收传感器数据，避免与调试串口冲突
void USART2_init(u32 baudrate) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;        
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;           
    NVIC_Init(&NVIC_InitStruct);
}

void USART1_Send_U8(uint8_t ch)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, ch);
}

void USART1_Send_ArrayU8(uint8_t *BufferPtr)
{
    while (*BufferPtr)
    {
        USART1_Send_U8(*BufferPtr++);
    }
}

void Processing_Data(uint8_t RXdata) {
    static uint8_t recv_buf[16] = {0}; // 接收缓冲区
    static uint8_t index = 0;           // 当前缓冲区位置
    static uint8_t parsing = 0;         // 解析状态
    static uint8_t comma_pos = 0;       // 第一个逗号的位置

    // 防溢出：缓冲区满则重置
    if (index >= sizeof(recv_buf)) {
        index = 0;
        parsing = 0;
        comma_pos = 0;
        return;
    }

    // 存储接收字节
    recv_buf[index++] = RXdata;

    switch (parsing) {
        case 0:  // 等待有效数据开始（数字、小数点）
            if ((RXdata >= '0' && RXdata <= '9') || RXdata == '.') {
                parsing = 1;  // 进入距离解析状态
            } else {
                // 无效字符（如空格、换行等），重置
                index = 0;
                parsing = 0;
                comma_pos = 0;
            }
            break;

        case 1:  // 解析距离值，直到遇到第一个逗号
            if (RXdata == ',') {
                parsing = 2;               // 进入置信度解析状态
                comma_pos = index - 1;      // 记录逗号位置（从0开始）
            }
            // 其他字符继续累积
            break;

        case 2:  // 解析置信度，直到遇到帧结束符（换行符）
            if (RXdata == '\n' || RXdata == '\r') {  // 以换行作为结束标志
                // ---------- 提取距离字符串 ----------
                uint8_t dist_len = comma_pos;  // 距离字符数
                float dist_float = 0.0f;
                if (dist_len > 0 && dist_len < sizeof(recv_buf)) {
                    char dist_str[12] = {0};
                    memcpy(dist_str, recv_buf, dist_len);
                    dist_str[dist_len] = '\0';
                    dist_float = atof(dist_str);
                }
                // 四舍五入取整
                distance_value = (uint16_t)(dist_float + 0.5f);

                // ---------- 提取置信度字符串 ----------
                uint8_t conf_start = comma_pos + 1;
                int conf_len = index - conf_start - 1;  // 减去刚存入的结束符
                uint16_t conf_temp = 0;                  // 使用uint16_t接收
                if (conf_len > 0 && conf_len < 10) {
                    char conf_str[10] = {0};
                    memcpy(conf_str, &recv_buf[conf_start], conf_len);
                    conf_str[conf_len] = '\0';
                    conf_temp = (uint16_t)atoi(conf_str);
                }
                confidence_value = conf_temp;

                // 数据有效性检查
                if (distance_value < DISTANCE_MIN ||
                    distance_value > DISTANCE_MAX ||
                    confidence_value > CONFIDENCE_MAX) {
                    distance_value = 0;
                    confidence_value = 0;
                }

                data_ready = 1;  // 设置数据就绪标志

                // 重置状态机
                index = 0;
                parsing = 0;
                comma_pos = 0;
            }
            // 注意：若数据以逗号结尾（如原格式），此处不会触发，因为逗号不会进入此分支。
            // 如需兼容以逗号结尾的格式，可在条件中增加 RXdata == ','，但根据新要求已去除末尾逗号，故不再添加。
            break;

        default:
            // 未知状态，重置
            parsing = 0;
            index = 0;
            comma_pos = 0;
            break;
    }
}

// 串口中断服务函数
void USART2_IRQHandler(void)
{
    uint8_t Rx1_Temp = 0;
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        Rx1_Temp = USART_ReceiveData(USART2);
        Processing_Data(Rx1_Temp);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

// 重定向printf到串口
int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return (ch);
}