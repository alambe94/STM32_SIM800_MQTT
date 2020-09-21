/** standard includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/** ST includes */
#include "usart.h"

/** app includes */
#include "sim800_uart.h"
#include "sim800_mqtt.h"

#define USE_UART_RX_DMA 0

/** uart used for comm with sim800 */
UART_HandleTypeDef *SIM800_UART = &huart3;

/** rx ring buffer data reception from sim800 */
#define RB_STORAGE_SIZE 1500
static uint8_t RB_Storage[RB_STORAGE_SIZE];
static uint32_t RB_Read_Index;
static volatile uint32_t RB_Write_Index;
static volatile uint8_t RB_Full_Flag;

/**
 * @brief Init uart used for sim800
 */
void SIM800_UART_Init(void)
{
    /** configured in cube @see usart.c*/

#if (USE_UART_RX_DMA == 1)
    /** start uart data reception */
    HAL_UART_Receive_DMA(SIM800_UART, RB_Storage, RB_STORAGE_SIZE);
#else
    HAL_UART_Receive_IT(SIM800_UART, &RB_Storage[0], 1);
#endif

    /** enable idle interrupt */
    __HAL_UART_ENABLE_IT(SIM800_UART, UART_IT_IDLE);
}

/**
 * @brief restart uart
 */
void SIM800_UART_Restart(void)
{
#if (USE_UART_RX_DMA == 1)
    __HAL_UART_DISABLE_IT(SIM800_UART, UART_IT_IDLE);
    HAL_UART_DMAStop(SIM800_UART);
    HAL_UART_DeInit(SIM800_UART);

    HAL_UART_Init(SIM800_UART);
    HAL_UART_Receive_DMA(SIM800_UART, RB_Storage, RB_STORAGE_SIZE);
    __HAL_UART_ENABLE_IT(SIM800_UART, UART_IT_IDLE);
#endif
}

/**
 * @brief flush ring buffer
 */
static void RB_Flush(void)
{
    RB_Read_Index = RB_Write_Index;
}

/**
 * @brief check if ring buffer is full
 * @retval return 1 if ring buffer is full
 * @note if using DMA, since data is written by dma, RB_Full_Flag can be not set at proper place. So this function might not be useful in this context.
 */
static uint8_t RB_Is_Full(void)
{
    return RB_Full_Flag;
}

/**
 * @brief check number of chars in ring buffer
 * @retval return number of chars in ring buffer
 */
static uint32_t RB_Get_Count(void)
{
#if (USE_UART_RX_DMA == 1)
    /** data is written to buffer via uart DMA in background*/
    /** need to update Write_Index manually */
    RB_Write_Index = RB_STORAGE_SIZE - SIM800_UART->hdmarx->Instance->NDTR;

    //RB_Full_Flag = (RB_Write_Index == RB_Read_Index);
#endif

    if (RB_Is_Full())
        return RB_STORAGE_SIZE;
    if (RB_Write_Index >= RB_Read_Index)
        return (RB_Write_Index - RB_Read_Index);
    return (RB_STORAGE_SIZE - (RB_Read_Index - RB_Write_Index));
}

/**
 * @brief check if buffer is empty
 * @retval return 1 if ring buffer is empty
 */
static uint8_t RB_Is_Empty(void)
{
    return (RB_Get_Count() == 0);
}

/**
 * @brief return a char from from ring buffer
 * @retval char return
 */
static int RB_Get_Char(void)
{
    if (RB_Is_Empty())
    {
        /** exception */
        return -1;
    }

    uint8_t temp = RB_Storage[RB_Read_Index++];
    RB_Full_Flag = 0;

    if (RB_Read_Index == RB_STORAGE_SIZE)
        RB_Read_Index = 0;

    return temp;
}

/**
 * @brief peek a char from from ring buffer
 * @retval char return
 */
static int RB_Peek_Char(void)
{
    if (RB_Is_Empty())
    {
        /** exception */
        return -1;
    }

    uint8_t temp = RB_Storage[RB_Read_Index];

    return temp;
}

/**
 * @brief return number of requested char from ring buffer
 *        if timeout occurs return only available chars
 * @param buff buffer where to retrieve
 * @param cnt  number of char to retrieve
 * @param timeout max wait time in milliseconds
 */
static uint32_t RB_Get_Chars(char *buff, uint32_t cnt, uint32_t timeout)
{
    uint32_t tick_now = HAL_GetTick();
    uint32_t tick_timeout = tick_now + timeout;
    uint32_t count = cnt;

    while (RB_Get_Count() < cnt && (tick_now <= tick_timeout))
    {
        tick_now = HAL_GetTick();
    }

    if (tick_now > tick_timeout)
    {
        /** get bytes available within timeout */
        count = RB_Get_Count();
    }

    for (uint32_t i = 0; i < count; i++)
    {
        buff[i] = RB_Get_Char();
    }

    return count;
}

/**
 * @brief send character
 * @param data char to be sent
 */
void SIM800_UART_Send_Char(char data)
{
    SIM800_UART->Instance->DR = data;
    while (__HAL_UART_GET_FLAG(SIM800_UART, UART_FLAG_TC) == 0)
        ;
}

/**
 * @brief send bytes buffer
 * @param data input buffer
 * @param number of chars to send
 **/
void SIM800_UART_Send_Bytes(char *data, uint32_t count)
{
    while (count--)
    {
        SIM800_UART_Send_Char(*data);
        data++;
    }
}

/**
 * @brief send bytes buffer using dma
 * @param data input buffer
 * @param number of chars to send
 **/
void SIM800_UART_Send_Bytes_DMA(char *data, uint32_t count)
{
    HAL_UART_Transmit_DMA(SIM800_UART, (uint8_t *)data, count);
}

/**
 * @brief send null terminated string
 * @param str string buffer
 **/
void SIM800_UART_Send_String(char *str)
{
    while (*str)
    {
        SIM800_UART_Send_Char(*str++);
    }
}

/**
 * @brief wrapper printf around SIM800 uart
 * @param fmt formatted string
 **/
void SIM800_UART_Printf(const char *fmt, ...)
{
    static char buffer[512];
    va_list args;
    va_start(args, fmt);
    uint32_t len = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    SIM800_UART_Send_Bytes(buffer, len);
}

/**
 * @brief get characters count in rx buffer
 * @param timeout
 */
uint32_t SIM800_UART_Get_Count(void)
{
    return RB_Get_Count();
}

/**
 * @brief get character
 * @retval return -1 if nochar is available
 */
int SIM800_UART_Get_Char(void)
{
    return RB_Get_Char();
}

/**
 * @brief peek character
 * @retval return -1 if nochar is available
 */
int SIM800_UART_Peek_Char(void)
{
    return RB_Peek_Char();
}

/**
 * @brief get chars
 * @param buffer
 * @param count
 * @param timeout
 * @retval number chars received
 */
uint32_t SIM800_UART_Get_Chars(char *buffer, uint32_t count, uint32_t timeout)
{
    return RB_Get_Chars(buffer, count, timeout);
}

/**
 * @brief get chars upto '\r' or max upto timeout or buff_size
 * @param buffer
 * @param buff_size max count that can be written to buffer before '\r' is found
 * @param timeout maximum number of millisecond to wait before '\r' is found
 * @retval number chars received
 */
uint32_t SIM800_UART_Get_Line(char *buffer, uint32_t buff_size, uint32_t timeout)
{
    uint32_t tick_now = HAL_GetTick();
    uint32_t tick_timeout = tick_now + timeout;
    uint32_t rx_chars_cnt = 0;

    while (tick_now <= tick_timeout && rx_chars_cnt < buff_size)
    {
        tick_now = HAL_GetTick();
        int rx_char = SIM800_UART_Get_Char();
        if (rx_char != -1)
        {
            /** carriage return found */
            if (rx_char == '\r')
            {
                SIM800_UART_Get_Char(); /** remove '\n' */
                buffer[rx_chars_cnt] = '\0';
                break;
            }

            if (rx_char != '\n') /** ignore '\n' if any */
            {
                buffer[rx_chars_cnt++] = rx_char;
            }
        }
    }

    return rx_chars_cnt;
}

/**
 * @brief flus sim800 rx buffer
 */
void SIM800_UART_Flush_RX(void)
{
    RB_Flush();
}

/**
 * @brief uart RX ISR, check for idle interrupt and update ring buffer
 *        called from @see USART3_IRQHandler in stm32f4xx_it.c
 **/
void SIM800_UART_RX_ISR(void)
{
    if (__HAL_UART_GET_FLAG(SIM800_UART, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(SIM800_UART);

        /** start sim800 rx process */
        extern void SIM800_RX_Ready_Callback(void);
        SIM800_RX_Ready_Callback();
    }
}

/**
 * @brief called when uart tx dma finishes sending
 *        called from @see HAL_UART_TxCpltCallback in stm32f4xx_it.c
 **/
void SIM800_UART_TX_CMPLT_ISR(void)
{
    extern void SIM800_TX_Complete_Callback(void);
    SIM800_TX_Complete_Callback();
}

/**
 * @brief called when uart rx is complete
 *        called from @see HAL_UART_RxCpltCallback in stm32f4xx_it.c
 **/
void SIM800_UART_RX_CMPLT_ISR(void)
{
#if (USE_UART_RX_DMA == 0)
    RB_Write_Index++;
    if (RB_Write_Index == RB_STORAGE_SIZE)
    {
        RB_Write_Index = 0;
    }
    /** start another reception */
    HAL_UART_Receive_IT(SIM800_UART, (RB_Storage + RB_Write_Index), 1);
#endif
}
