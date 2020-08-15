/** standard includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/** ST includes */
#include "usart.h"

/** app includes */
#include "sim800_mqtt.h"

/** uart used from comm with sim800 */
UART_HandleTypeDef *SIM800_UART = &huart3;

/** rx ring buffer data reception from sim800 */
#define RB_STORAGE_SIZE 1024
static uint8_t RB_Storage[RB_STORAGE_SIZE];
static uint8_t RB_Read_Index;
static uint8_t RB_Write_Index;
static uint8_t RB_Full_Flag;

/**
 * @brief flush ring buffer
 */
static void RB_Flush()
{
    RB_Read_Index = RB_Write_Index;
}

/**
 * @brief check if ring buffer is full
 * @retval return 1 if ring buffer is full
 * @note since data is written by dma, RB_Full_Flag can be not set at proper place. So this function is not useful in this context.
 */
static uint8_t RB_Is_Full()
{
    return RB_Full_Flag;
}

/**
 * @brief check number of chars in ring buffer
 * @retval return number of chars in ring buffer
 */
static uint32_t RB_Get_Count()
{
    if (RB_Full_Flag)
        return RB_STORAGE_SIZE;
    if (RB_Write_Index >= RB_Read_Index)
        return (RB_Write_Index - RB_Read_Index);
    return (RB_STORAGE_SIZE - (RB_Read_Index - RB_Write_Index));
}

/**
 * @brief check if buffer is empty
 * @retval return 1 if ring buffer is empty
 */
static uint8_t RB_Is_Empty()
{
    return (RB_Get_Count() == 0);
}

/**
 * @brief return a char from from ring buffer
 * @retval char return
 */
static int RB_Get_Char()
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

    while (RB_Get_Count() < cnt && (tick_now < tick_timeout))
    {
        tick_now = HAL_GetTick();
    }

    if (tick_now >= tick_timeout)
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
static void SIM800_UART_Send_Char(char data)
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
 * @brief send null terminated string
 * @param data input buffer
 * @param number of chars to send
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
 * @brief get character
 * @param timeout
 */
int SIM800_UART_Get_Char(uint32_t timeout)
{
    return RB_Get_Char();
}

/**
 * @brief get chars
 * @param timeout
 * @retval number chars received
 */
uint32_t SIM800_UART_Get_Chars(char *buffer, uint32_t count, uint32_t timeout)
{
    return RB_Get_Chars(buffer, count, timeout);
}

/**
 * @brief get chars upto '\r' or max upto timeout
 * @param timeout
 * @retval number chars received
 */
uint32_t SIM800_UART_Get_Line(char *buffer, uint32_t timeout)
{
    uint32_t tick_now = HAL_GetTick();
    uint32_t tick_timeout = tick_now + timeout;
    uint16_t rx_chars_cnt = 0;
    int rx_char;

    while (tick_now < tick_timeout)
    {
        tick_now = HAL_GetTick();
        rx_char = SIM800_UART_Get_Char(1);
        if (rx_char != -1)
        {
            /** carriage return found */
            if (rx_char == '\r')
            {
                SIM800_UART_Get_Char(1); /** remove '\n' */
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
 * @brief Init peripheral ised by sim800
 */
uint8_t SIM800_Init(void)
{
    /** uart used for comm is configured in cube @see usart.c */

    uint8_t sim800_result = 0;

    /** start uart data reception */
    HAL_UART_Receive_DMA(SIM800_UART, RB_Storage, RB_STORAGE_SIZE);

    /** enable idle interrupt */
    __HAL_UART_ENABLE_IT(SIM800_UART, UART_IT_IDLE);

    HAL_GPIO_WritePin(RST_SIM800_GPIO_Port, RST_SIM800_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(RST_SIM800_GPIO_Port, RST_SIM800_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);

    /** send dummy, so sim800 can auto adjust its baud */
    SIM800_UART_Send_String("AT\r\n");
    HAL_Delay(100);

    /** disable echo */
    SIM800_UART_Send_String("ATE0\r\n");
    HAL_Delay(100);

    uint8_t lines = 10;
    while (lines--)
    {
        sim800_result = SIM800_Check_Response("SMS Ready", 2000);
        if (sim800_result)
        {
            break;
        }
    }

    RB_Flush();

    return sim800_result;
}

/**
 * @brief flus sim800 rx buffer
 */
void SIM800_Flush_RX()
{
    RB_Flush();
}

/**
 * @brief return the received chars from sim800
 * @param buff buffer where to retrieve
 * @param cnt  number of char to retrieve
 * @param timeout max wait time in milliseconds
 * @retval number chars in response
 */
uint32_t SIM800_Get_Response(char *buff, uint32_t timeout)
{
    SIM800_UART_Get_Line(buff, timeout); /** ignore first '\r' */
    return SIM800_UART_Get_Line(buff, timeout);
}

/**
 * @brief check for expected response
 * @param buff expected response
 * @param alt_buff alternate response or other substring
 * @param count max chars to receive
 * @param timeout max wait time in milliseconds
 * @retval return 1 if success else 0
 */
uint8_t SIM800_Check_Response(char *buff, uint32_t timeout)
{
    char reply[32] = "";
    char *res;

    SIM800_UART_Get_Line(reply, timeout); /** ignore first '\r' */
    SIM800_UART_Get_Line(reply, timeout);

    res = strstr(reply, buff);

    return (res != NULL);
}

/**
 * @brief open tcp connection, connect to mqtt broker
 * @param sim_apn simcard apn such "www" for vodafone and "airtelgprs.com" for airtel
 * @param broker broker mqtt address
 * @param port   broker mqtt port
 * @param protocol_name "MQTT" or "MQIsdp" etc
 * @param protocol_version 3 for 3.1.0 or 4 for 3.1.1
 * @param flags TODO add bitfields for user name, password and other flags
 * @param keep_alive kepp alive interval in seconds
 * @param my_id clien id can be arbitary
 * @param user_name user name at broker service
 * @param password password for broker service
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Connect(char *sim_apn,
                            char *broker,
                            uint32_t port,
                            char *protocol_name,
                            uint8_t protocol_version,
                            uint8_t flags,
                            uint32_t keep_alive,
                            char *my_id,
                            char *user_name,
                            char *password)
{
    char sim800_reply[32];
    char conn_ack[4];
    uint8_t sim800_result;

    /**************************************************************** TCP connection start *********************************************************************/

    SIM800_UART_Send_String("AT\r\n");
    sim800_result = SIM800_Check_Response("OK", 1000); /** expected reply "OK" within 1 second "*/

    SIM800_UART_Send_String("AT+CGATT?\r\n");                 /** GPRS Service’s status */
    sim800_result = SIM800_Check_Response("+CGATT: 1", 1000); /** expected reply  "+CGATT: 1" and "OK" within 1 second "*/
    sim800_result = SIM800_Check_Response("OK", 3000);        /** expected reply "OK" within 1 second "*/

    if (sim800_result)
    {
        /** assemble sim apn */
        SIM800_UART_Printf("AT+CSTT=\"%s\"\r\n", sim_apn);
        sim800_result = SIM800_Check_Response("OK", 3000); /** expected reply "OK" within 1 second "*/
    }

    if (sim800_result)
    {
        SIM800_UART_Send_String("AT+CIICR\r\n");           /** Bring up wireless connection (GPRS or CSD) */
        sim800_result = SIM800_Check_Response("OK", 3000); /** expected reply "OK" within 10 second */
    }

    if (sim800_result)
    {
        SIM800_UART_Send_String("AT+CIFSR\r\n"); /** Get local IP address */
        SIM800_Get_Response(sim800_reply, 3000); /** expected reply "xxx.xxx.xxx.xxx" */
    }

    if (sim800_result)
    {
        /** assemble server ip and port */
        SIM800_UART_Printf("AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n",
                           broker,
                           port);
        sim800_result = SIM800_Check_Response("OK", 5000); /** expected reply "OK" and "CONNECT OK" within 10 second */
        sim800_result = SIM800_Check_Response("CONNECT OK", 5000);
    }
    /**************************************************************** TCP connection end *********************************************************************/

    /*************************************************************** MQTT connection start *******************************************************************/

    if (sim800_result)
    {
        SIM800_UART_Send_String("AT+CIPSEND\r\n"); /** Send data to remote server after promt mark ">" */
        sim800_result = SIM800_Check_Response(">", 3000);
    }

    if (sim800_result)
    {
        SIM800_UART_Send_Char(0x10); /** MQTT connect fixed header */

        uint8_t protocol_name_len = strlen(protocol_name);
        uint8_t my_id_len = strlen(my_id);
        uint8_t user_name_len = strlen(user_name);
        uint8_t password_len = strlen(password);

        uint32_t packet_len = 2 + protocol_name_len + 2 + my_id_len + 2 + user_name_len + 2 + password_len + 4;

        do
        {
            uint8_t len = packet_len % 128;
            packet_len = packet_len / 128;
            if (packet_len > 0)
            {
                len |= 128;
            }
            SIM800_UART_Send_Char(len);
        } while (packet_len > 0);

        SIM800_UART_Send_Char(protocol_name_len >> 8);
        SIM800_UART_Send_Char(protocol_name_len & 0xFF);
        SIM800_UART_Send_String(protocol_name);

        SIM800_UART_Send_Char(protocol_version);

        SIM800_UART_Send_Char(flags);

        SIM800_UART_Send_Char(keep_alive >> 8);
        SIM800_UART_Send_Char(keep_alive & 0xFF);

        SIM800_UART_Send_Char(my_id_len >> 8);
        SIM800_UART_Send_Char(my_id_len & 0xFF);
        SIM800_UART_Send_String(my_id);

        SIM800_UART_Send_Char(user_name_len >> 8);
        SIM800_UART_Send_Char(user_name_len & 0xFF);
        SIM800_UART_Send_String(user_name);

        SIM800_UART_Send_Char(password_len >> 8);
        SIM800_UART_Send_Char(password_len & 0xFF);
        SIM800_UART_Send_String(password);

        SIM800_UART_Send_Char(0x1A); /**  CTRL+Z (0x1A) to send */
        sim800_result = SIM800_Check_Response("SEND OK", 5000);

        SIM800_UART_Get_Chars(conn_ack, 4, 5000);

        if (conn_ack[0] == 0x20 && conn_ack[1] == 0x02 && conn_ack[2] == 0x00 && conn_ack[3] == 0x00)
        {
            sim800_result = 1;
        }
        else
        {
            sim800_result = 0;
        }
    }
    /*************************************************************** MQTT connection end *********************************************************************/

    return sim800_result;
}

/**
 * @brief publish message to a topic
 * @param topic topic to which message will be published
 * @param mesaage message to published
 * @param mesaage_len message length
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Publish(char *topic, char *mesaage, uint32_t mesaage_len)
{
    uint8_t sim800_result;
    char pub_ack[4];

    SIM800_UART_Send_String("AT\r\n");
    sim800_result = SIM800_Check_Response("OK", 1000); /** expected reply "OK" within 1 second "*/

    if (sim800_result)
    {
        SIM800_UART_Send_String("AT+CIPSEND\r\n"); /** Send data to remote server after promoting mark ">" */
        sim800_result = SIM800_Check_Response(">", 3000);
    }

    if (sim800_result)
    {
        uint8_t topic_len = strlen(topic);

        SIM800_UART_Send_Char(0x30); /** MQTT publish fixed header */

        uint32_t packet_len = 2 + topic_len + mesaage_len;

        do
        {
            uint8_t len = packet_len % 128;
            packet_len = packet_len / 128;
            if (packet_len > 0)
            {
                len |= 128;
            }
            SIM800_UART_Send_Char(len);
        } while (packet_len > 0);

        SIM800_UART_Send_Char(topic_len >> 8);
        SIM800_UART_Send_Char(topic_len & 0xFF);

        SIM800_UART_Send_String(topic);
        SIM800_UART_Send_Bytes(mesaage, mesaage_len);

        SIM800_UART_Send_Char(0x1A); /**  CTRL+Z (0x1A) to send */
        sim800_result = SIM800_Check_Response("SEND OK", 5000);
    }

    return sim800_result;
}
#if 0
/**
 * @brief subscribe to a topic
 * @param topic topic to be subscribe to
 * @param packet_id message ID can be arbitrary? TODO
 * @param qos 0, 1 or 2
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos)
{
    uint8_t sim800_result;

    SIM800_UART_Send_String("AT\r\n");
    sim800_result = SIM800_Check_Response("OK\r\n", NULL, 6, 1000); /** expected reply "OK\r\n" within 1 second "*/

    if (sim800_result)
    {
        SIM800_UART_Send_String("AT+CIPSEND\r\n"); /** Send data to remote server after promoting mark ">" */
        sim800_result = SIM800_Check_Response(">", NULL, 1, 3000);
    }

    if (sim800_result)
    {
        uint8_t topic_len = strlen(topic);

        uint32_t packet_len = 2 + 2 + topic_len + 1;

        SIM800_UART_Send_Char(0x82); /** MQTT subscribe fixed header */

        do
        {
            uint8_t len = packet_len % 128;
            packet_len = packet_len / 128;
            if (packet_len > 0)
            {
                len |= 128;
            }
            SIM800_UART_Send_Char(len);
        } while (packet_len > 0);

        SIM800_UART_Send_Char(packet_id >> 8);
        SIM800_UART_Send_Char(packet_id & 0xFF);

        SIM800_UART_Send_Char(topic_len >> 8);
        SIM800_UART_Send_Char(topic_len & 0xFF);

        SIM800_UART_Send_String(topic);

        SIM800_UART_Send_Char(qos);

        SIM800_UART_Send_Char(0x1A); /**  CTRL+Z (0x1A) to send */

        sim800_result = SIM800_Check_Response(">", NULL, 1, 3000); /** TODO */
    }
    return sim800_result;
}
#endif
/**
 * @brief called when message is received TODO
 * @param topic topic on which message is received
 * @param message received message
 */
void SIM800_MQTT_Received_Callback(char *topic, char *message)
{
}

/**
 * @brief mqtt processing loop TODO
 */
void SIM800_MQTT_Loop()
{
    while (RB_Get_Count())
    {
    }
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

        /** data is written to buffer via uart DMA in background*/
        /** need to update Write_Index manually */
        RB_Write_Index = RB_STORAGE_SIZE - SIM800_UART->hdmarx->Instance->NDTR;
    }
}
