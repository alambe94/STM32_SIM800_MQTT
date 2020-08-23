/** standard includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>

/** ST includes */
#include "main.h"
#include "tim.h"

/** app includes */
#include "sim800_mqtt.h"
#include "sim800_uart.h"

enum SIM800_Response_t
{
    SIM800_RESP_NONE,
    SIM800_RESP_ANY,
    SIM800_RESP_OK,
    SIM800_RESP_SMS_READY,
    SIM800_RESP_GPRS_READY,
    SIM800_RESP_SHUT_OK,
    SIM800_RESP_IP,
    SIM800_RESP_CONNECT,

    SIM800_RESP_MQTT_CONNACK,
    SIM800_RESP_MQTT_PUBACK,
    SIM800_RESP_MQTT_SUBACK,
    SIM800_RESP_MQTT_PINGACK,
} SIM800_Expected_Response,
    SIM800_Received_Response;

enum SIM800_State_t
{
    SIM800_IDLE,
    SIM800_RESETING,
    SIM800_RESET_OK,
    SIM800_TCP_CONNECTING,
    SIM800_TCP_CONNECTED, /** after this modem is in transparent mode */
    SIM800_MQTT_CONNECTING,
    SIM800_MQTT_CONNECTED,

    SIM800_MQTT_TRANSMITTING, /** indicates uart tx is busy */
    SIM800_MQTT_RECEIVING,

} SIM800_State;

typedef enum SIM800_Status_t
{
    SIM800_SUCCESS,
    SIM800_BUSY,
    SIM800_FAILED
} SIM800_Status_t;

/**
  * @brief  return ch occurrence in string
  */
static uint32_t CH_In_STR(char ch, char *str)
{
    uint32_t cnt = 0;
    while (*str)
    {
        if (*str++ == ch)
        {
            ch++;
        }
    }

    return cnt;
}

/**
  * @brief  one of the unused gpio pin interrupt is used to handle SIM800 background task
  *         it will be software triggered by sim800 uart idle interrupt
  * @note   set NVIC to 4 bit preemption 0 bit for sub priority
  */
void SIM800_RX_Task_Init(void)
{
    HAL_NVIC_SetPriority((IRQn_Type)EXTI1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)EXTI1_IRQn);
}

/**
  * @brief  one of the timer is used to generate periodic interrupt at 10ms (adjustable)
  * @note
  */
void SIM800_TX_Task_Init(void)
{
    /** configured in cube @see tim.c */
}

/**
  * @brief trigger sim800 rx task interrupt
  */
void SIM800_RX_Task_Trigger(void)
{
    HAL_NVIC_SetPendingIRQ(EXTI1_IRQn);
}

/**
 * @brief Init peripheral used by sim800
 */
void SIM800_Init(void)
{
    /** uart used for comm is configured in cube @see usart.c */
    SIM800_UART_Init();

    SIM800_RX_Task_Init();

    SIM800_TX_Task_Init();

    HAL_TIM_Base_Start_IT(&htim14);
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
 * @brief reset sim800
 */
SIM800_Status_t SIM800_Reset(void)
{
    if (SIM800_State == SIM800_IDLE)
    {
        SIM800_State = SIM800_RESETING;
        return 1;
    }

    return 0;
}
static SIM800_Status_t _SIM800_Reset(void)
{
    static SIM800_Status_t sim800_result = SIM800_BUSY;

    static uint8_t reset_step = 0;
    static uint8_t retry = 0;

    static uint32_t next_delay = 0;
    static uint32_t loop_ticks = 0;

    if (HAL_GetTick() - loop_ticks > next_delay)
    {
        loop_ticks = HAL_GetTick();

        switch (reset_step)
        {
        case 0:
            HAL_GPIO_WritePin(RST_SIM800_GPIO_Port, RST_SIM800_Pin, GPIO_PIN_RESET);
            sim800_result = SIM800_BUSY;
            reset_step++;
            next_delay = 1000;
            break;

        case 1:
            HAL_GPIO_WritePin(RST_SIM800_GPIO_Port, RST_SIM800_Pin, GPIO_PIN_SET);
            reset_step++;
            next_delay = 5000;
            break;

        case 2:
            /** send dummy, so sim800 can auto adjust its baud */
            SIM800_UART_Send_String("AT\r\n");
            SIM800_Expected_Response = SIM800_RESP_NONE;
            reset_step++;
            next_delay = 100;
            break;

        case 3:
            /** disable echo */
            SIM800_UART_Send_String("ATE0\r\n");
            SIM800_Expected_Response = SIM800_RESP_NONE;
            reset_step++;
            next_delay = 1000;
            break;

        case 4:
            /** disable echo */
            SIM800_UART_Flush_RX();
            reset_step++;
            next_delay = 100;
            break;

        case 5:
            /** wait SMS Ready flag */
            if (SIM800_Received_Response == SIM800_RESP_SMS_READY)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                reset_step++;
                next_delay = 100;
                retry = 0;
            }
            {
                next_delay = 2000;
                SIM800_Expected_Response = SIM800_RESP_SMS_READY;
                retry++;
                if (retry == 10)
                {
                    sim800_result = SIM800_FAILED; /** failed */
                }
            }
            break;

        case 6:
            /** wait GPRS Service’s status */
            if (SIM800_Received_Response == SIM800_RESP_GPRS_READY)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                reset_step++;
                retry = 0;
                next_delay = 1000;
            }
            {
                SIM800_UART_Send_String("AT+CGATT?\r\n");
                SIM800_Expected_Response = SIM800_RESP_GPRS_READY;
                next_delay = 3000;
                retry++;
                if (retry == 20)
                {
                    sim800_result = SIM800_FAILED; /** failed */
                }
            }
            break;

        case 7:
            SIM800_UART_Flush_RX();
            reset_step = 0;
            retry = 0;
            sim800_result = SIM800_SUCCESS;
            break;

        default:
            break;
        }
    }

    return sim800_result;
}

/**
 * @brief open tcp connection to mqtt broker
 * @param sim_apn simcard apn such "www" for vodafone and "airtelgprs.com" for airtel
 * @param broker broker mqtt address
 * @param port   broker mqtt port
 * @retval return 1 if success else 0
 */
struct TCP_Data_t
{
    char _SIM_APN[32];
    char _Broker[32];
    uint16_t _Port;
} _TCP_Data;
uint8_t SIM800_TCP_Connect(char *sim_apn, char *broker, uint16_t port)
{
    if (SIM800_State >= SIM800_RESET_OK)
    {
        strncpy(_TCP_Data._SIM_APN, sim_apn, sizeof(_TCP_Data._SIM_APN));
        strncpy(_TCP_Data._Broker, broker, sizeof(_TCP_Data._Broker));
        _TCP_Data._Port = port;

        SIM800_State = SIM800_TCP_CONNECTING;

        return 1;
    }

    return 0;
}
static uint8_t _SIM800_TCP_Connect()
{
    static SIM800_Status_t sim800_result = SIM800_BUSY;

    static uint8_t tcp_step = 0;

    static uint32_t next_delay = 0;
    static uint32_t loop_ticks = 0;

    if (HAL_GetTick() - loop_ticks > next_delay)
    {
        loop_ticks = HAL_GetTick();

        switch (tcp_step)
        {
        case 0:
            SIM800_UART_Send_String("AT+CIPSHUT\r\n");
            SIM800_Expected_Response = SIM800_RESP_SHUT_OK;
            sim800_result = SIM800_BUSY;
            next_delay = 10;
            tcp_step++;
            break;

        case 1:
            /** check response of previous cmd (AT+CIPSHUT) */
            if (SIM800_Received_Response == SIM800_RESP_SHUT_OK)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                SIM800_UART_Send_String("AT+CIPMODE=1\r\n");
                SIM800_Expected_Response = SIM800_RESP_OK;
                next_delay = 10;
                tcp_step++;
            }
            else
            {
                sim800_result = SIM800_FAILED;
            }
            break;

        case 2:
            /** check response of previous cmd (AT+CIPMODE=1) */
            if (SIM800_Received_Response == SIM800_RESP_OK)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                /** assemble sim apn */
                SIM800_UART_Printf("AT+CSTT=\"%s\",\"\",\"\"\r\n", _TCP_Data._SIM_APN);
                SIM800_Expected_Response = SIM800_RESP_OK;
                next_delay = 3000;
                tcp_step++;
            }
            else
            {
                sim800_result = SIM800_FAILED;
            }
            break;

        case 3:
            /** check response of previous cmd (AT+CSTT=) */
            if (SIM800_Received_Response == SIM800_RESP_OK)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                /** Bring up wireless connection (GPRS or CSD) */
                SIM800_UART_Send_String("AT+CIICR\r\n");
                SIM800_Expected_Response = SIM800_RESP_OK;
                next_delay = 100;
                tcp_step++;
            }
            else
            {
                sim800_result = SIM800_FAILED;
            }
            break;

        case 4:
            /** check response of previous cmd (AT+CIICR) */
            if (SIM800_Received_Response == SIM800_RESP_OK)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                /** Get local IP address */
                SIM800_UART_Send_String("AT+CIFSR\r\n");
                SIM800_Expected_Response = SIM800_RESP_IP;
                next_delay = 100;
                tcp_step++;
            }
            else
            {
                sim800_result = SIM800_FAILED;
            }
            break;

        case 5:
            /** check response of previous cmd (AT+CIFSR) */
            if (SIM800_Received_Response == SIM800_RESP_IP)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                /** assemble server ip and port */
                SIM800_UART_Printf("AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n",
                                   _TCP_Data._Broker,
                                   _TCP_Data._Port);
                SIM800_Expected_Response = SIM800_RESP_CONNECT;
                next_delay = 100;
                tcp_step++;
            }
            else
            {
                sim800_result = SIM800_FAILED;
            }
            break;

        case 6:
            /** check response of previous cmd (AT+CIPSTART) */
            if (SIM800_Received_Response == SIM800_RESP_CONNECT)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                sim800_result = SIM800_SUCCESS;
                next_delay = 100;
                tcp_step = 0;
            }
            else
            {
                sim800_result = SIM800_FAILED;
            }
            break;

        default:
            break;
        }
    }

    return sim800_result;
}

/**
 * @brief send connect packet to broker
 * @param topic topic to which message will be published
 * @param mesaage message to published
 * @param mesaage_len message length
 * @retval return 1 if success else 0
 */
struct CONN_Data_t
{
    char _Protocol_Name[32];
    char _My_ID[32];
    char _User_Name[32];
    char _Password[32];
    uint8_t _Protocol_Version;
    uint8_t _CONN_Flags;
    uint16_t _Keep_Alive;
} _CONN_Data;
uint8_t SIM800_MQTT_Connect(char *protocol_name,
                            uint8_t protocol_version,
                            uint8_t flags,
                            uint16_t keep_alive,
                            char *my_id,
                            char *user_name,
                            char *password)
{
    if (SIM800_State >= SIM800_TCP_CONNECTED)
    {
        strncpy(_CONN_Data._Protocol_Name, protocol_name, sizeof(_CONN_Data._Protocol_Name));
        strncpy(_CONN_Data._My_ID, my_id, sizeof(_CONN_Data._My_ID));
        strncpy(_CONN_Data._User_Name, user_name, sizeof(_CONN_Data._User_Name));
        strncpy(_CONN_Data._Password, password, sizeof(_CONN_Data._Password));
        _CONN_Data._Protocol_Version = protocol_version;
        _CONN_Data._CONN_Flags = flags;
        _CONN_Data._Keep_Alive = keep_alive;

        SIM800_State = SIM800_TCP_CONNECTING;

        return 1;
    }

    return 0;
}
uint8_t _SIM800_MQTT_Connect()
{
    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(0x10); /** MQTT connect fixed header */

    uint8_t protocol_name_len = strlen(_CONN_Data._Protocol_Name);
    uint8_t my_id_len = strlen(_CONN_Data._My_ID);
    uint8_t user_name_len = strlen(_CONN_Data._User_Name);
    uint8_t password_len = strlen(_CONN_Data._Password);

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
    SIM800_UART_Send_String(_CONN_Data._Protocol_Name);

    SIM800_UART_Send_Char(_CONN_Data._Protocol_Version);

    SIM800_UART_Send_Char(_CONN_Data._CONN_Flags);

    SIM800_UART_Send_Char(_CONN_Data._Keep_Alive >> 8);
    SIM800_UART_Send_Char(_CONN_Data._Keep_Alive & 0xFF);

    SIM800_UART_Send_Char(my_id_len >> 8);
    SIM800_UART_Send_Char(my_id_len & 0xFF);
    SIM800_UART_Send_String(_CONN_Data._My_ID);

    SIM800_UART_Send_Char(user_name_len >> 8);
    SIM800_UART_Send_Char(user_name_len & 0xFF);
    SIM800_UART_Send_String(_CONN_Data._User_Name);

    SIM800_UART_Send_Char(password_len >> 8);
    SIM800_UART_Send_Char(password_len & 0xFF);
    SIM800_UART_Send_String(_CONN_Data._Password);

    SIM800_Expected_Response = SIM800_RESP_MQTT_CONNACK;

    SIM800_State = SIM800_MQTT_RECEIVING; /** indicates uart tx is done */

    return 1;
}

/**
 * @brief return 1 if sim800 is connected to broker
 */
uint8_t SIM800_Is_Connected()
{
    return (SIM800_State >= SIM800_MQTT_CONNECTED);
}

/**
 * @brief send disconnect packet
 */
void SIM800_MQTT_Disconnect(void)
{
    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(0xD0); /** MQTT disconnect */
    SIM800_UART_Send_Char(0x00);

    SIM800_State = SIM800_TCP_CONNECTED;
}

/**
 * @brief send ping packet
 */
uint8_t SIM800_MQTT_Ping(void)
{
    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(0xC0); /** MQTT ping */
    SIM800_UART_Send_Char(0x00);

    SIM800_Expected_Response = SIM800_RESP_MQTT_PINGACK;

    SIM800_State = SIM800_MQTT_RECEIVING; /** indicates uart tx is done */

    return 1;
}

/**
 * @brief publish message to a topic
 * @param topic topic to which message will be published
 * @param mesaage message to published
 * @param mesaage_len message length
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Publish(char *topic,
                            char *mesaage,
                            uint32_t mesaage_len,
                            uint8_t dup,
                            uint8_t qos,
                            uint8_t retain,
                            uint16_t mesaage_id)
{
    uint8_t topic_len = strlen(topic);

    uint8_t pub = 0x30 | (dup << 3) | (qos << 1) | retain;

    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(pub); /** MQTT publish fixed header */

    uint32_t packet_len = 2 + topic_len + mesaage_len;

    if (qos)
    {
        packet_len += 2;
    }

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

    if (qos)
    {
        SIM800_UART_Send_Char(mesaage_id >> 8);
        SIM800_UART_Send_Char(mesaage_id & 0xFF);
    }

    if (mesaage_len > 64)
    {
        /** non blocking, SIM800_MQTT_TRANSMITTING will be cleared in uart tx dma isr */
        SIM800_UART_Send_Bytes_DMA(mesaage, mesaage_len);
    }
    else
    {
        /** blocking */
        SIM800_UART_Send_Bytes(mesaage, mesaage_len);
        SIM800_State = SIM800_MQTT_RECEIVING; /** indicates uart tx is done */
    }

    SIM800_Expected_Response = SIM800_RESP_MQTT_PUBACK;

    return 1;
}

/**
 * @brief subscribe to a topic
 * @param topic topic to be subscribe to
 * @param packet_id message ID can be arbitrary? TODO
 * @param qos 0, 1
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos)
{
    uint8_t topic_len = strlen(topic);

    uint32_t packet_len = 2 + 2 + topic_len + 1;

    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

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

    SIM800_Expected_Response = SIM800_RESP_MQTT_SUBACK;

    SIM800_State = SIM800_MQTT_RECEIVING; /** indicates uart tx is done */

    return 1;
}

/******************** callbacks **************************/

void SIM800_Reset_complete_Callback(SIM800_Status_t status)
{
}

void SIM800_TCP_CONN_complete_Callback(SIM800_Status_t status)
{
}

/**
 * @brief called when message is received TODO
 * @param topic topic on which message is received
 * @param message received message
 */
void SIM800_MQTT_Received_Callback(char *topic, char *message, uint8_t dup, uint8_t qos, uint8_t message_id)
{
}

/**
 * @brief called when sim800 modem using uart dma mode, @see SIM800_UART_TX_CMPLT_ISR
 * @note only applicable if tx dma is used
 */
void SIM800_MQTT_TX_Complete_Callback(void)
{
    if (SIM800_State == SIM800_MQTT_TRANSMITTING)
    {
        SIM800_State = SIM800_MQTT_RECEIVING;
    }
}

/**
 * @brief called when CONNACK is received
 * @param code return code fro broker
 */
void SIM800_MQTT_CONNACK_Callback(uint16_t code)
{
    if (code == 0)
    {
        SIM800_State = SIM800_MQTT_CONNECTED;
    }
    else
    {
        SIM800_State = SIM800_IDLE;
    }
}

/**
 * @brief called when PUBACK is received
 * @param mesaage_id message on which ack is received
 */
void SIM800_MQTT_PUBACK_Callback(uint16_t mesaage_id)
{
}

/**
 * @brief called when SUBACK is received
 * @param packet_id packet on which ack is received
 * @param qos of topic
 */
void SIM800_MQTT_SUBACK_Callback(uint16_t packet_id, uint8_t qos)
{
}

/**
 * @brief called when ping response is received
 */
void SIM800_MQTT_Ping_Callback()
{
}

/************************* ISR ***************************/

/**
 * @brief this sim800 rx task, called when sim800 timer expires
 *        called from @see HAL_UART_TxCpltCallback in stm32f4xx_it.c
 **/
void SIM800_TIM_CMPLT_ISR(void)
{
    SIM800_Status_t sim800_result = SIM800_BUSY;

    switch (SIM800_State)
    {
    case SIM800_IDLE:
        break;

    case SIM800_RESET_OK:
        break;

    case SIM800_RESETING:
        sim800_result = _SIM800_Reset();
        if (sim800_result == SIM800_SUCCESS || sim800_result == SIM800_FAILED)
        {
            SIM800_Reset_complete_Callback(sim800_result);
        }
        else if (sim800_result == SIM800_BUSY)
        {
        }
        break;

    case SIM800_TCP_CONNECTING:
        _SIM800_TCP_Connect();
        if (sim800_result == SIM800_SUCCESS || sim800_result == SIM800_FAILED)
        {
            SIM800_TCP_CONN_complete_Callback(sim800_result);
        }
        else if (sim800_result == SIM800_BUSY)
        {
        }
        break;

    case SIM800_TCP_CONNECTED:
        break;

    case SIM800_MQTT_CONNECTING:
        break;

    case SIM800_MQTT_CONNECTED:
    case SIM800_MQTT_TRANSMITTING:
    case SIM800_MQTT_RECEIVING:
        break;
    }
}

/**
  * @brief this sim800 rx task, triggered by software in sim800 uart idle isr
  */
void EXTI1_IRQHandler(void)
{
    if (SIM800_State >= SIM800_TCP_CONNECTED)
    {
        /** in transparent mode */
        char rx_chars[32] = "";

        switch (SIM800_Expected_Response)
        {
        case SIM800_RESP_MQTT_CONNACK:
            SIM800_UART_Get_Chars(rx_chars, 4, 5);
            {
                if (rx_chars[0] == 0x20 && rx_chars[1] == 0x02)
                {
                    SIM800_Received_Response = SIM800_RESP_MQTT_CONNACK;
                    SIM800_Expected_Response = SIM800_RESP_NONE;
                    SIM800_MQTT_CONNACK_Callback(rx_chars[2] << 8 | rx_chars[3]);
                }
            }
            break;

        case SIM800_RESP_MQTT_PUBACK:
            SIM800_UART_Get_Chars(rx_chars, 4, 5);
            if (rx_chars[0] == 0x40 && rx_chars[1] == 0x02)
            {
                SIM800_Received_Response = SIM800_RESP_MQTT_PUBACK;
                SIM800_Expected_Response = SIM800_RESP_NONE;
                SIM800_MQTT_PUBACK_Callback(rx_chars[2] << 8 | rx_chars[3]);
            }
            break;

        case SIM800_RESP_MQTT_SUBACK:
            SIM800_UART_Get_Chars(rx_chars, 5, 5);
            if (rx_chars[0] == 0x90 && rx_chars[1] == 0x02)
            {
                SIM800_Received_Response = SIM800_RESP_MQTT_PUBACK;
                SIM800_Expected_Response = SIM800_RESP_NONE;
                SIM800_MQTT_SUBACK_Callback(rx_chars[2] << 8 | rx_chars[3], rx_chars[4]);
            }
            break;

        case SIM800_RESP_MQTT_PINGACK:
            SIM800_UART_Get_Chars(rx_chars, 2, 5);
            if (rx_chars[0] == 0xD0 && rx_chars[1] == 0x00)
            {
                SIM800_Received_Response = SIM800_RESP_MQTT_PUBACK;
                SIM800_Expected_Response = SIM800_RESP_NONE;
                SIM800_MQTT_Ping_Callback();
            }
            break;

        case SIM800_RESP_NONE:

            SIM800_UART_Get_Chars(rx_chars, 1, 5);

            if ((rx_chars[0] & 0x30) == 0x30)
            {
                uint8_t qos = (rx_chars[0] >> 1) & 0x03;
                uint8_t dup = (rx_chars[0] >> 3) & 0x01;

                uint32_t multiplier = 1;
                uint32_t total_len = 0;
                uint32_t mesg_len = 0;
                uint16_t topic_len = 0;
                uint16_t mesaage_id = 0;

                char topic[32] = "";
                char msg[32] = "";

                do
                {
                    SIM800_UART_Get_Chars(rx_chars, 1, 5);
                    total_len += (rx_chars[0] & 127) * multiplier;
                    multiplier *= 128;
                    if (multiplier > 128 * 128 * 128)
                    {
                        break;
                    }
                } while ((rx_chars[0] & 128) != 0);

                SIM800_UART_Get_Chars(rx_chars, 2, 5);
                topic_len = (rx_chars[1] << 8) | rx_chars[0];

                mesg_len = total_len - topic_len;

                SIM800_UART_Get_Chars(topic, topic_len, 5);

                if (qos)
                {
                    SIM800_UART_Get_Chars(rx_chars, 2, 5);
                    mesaage_id = (rx_chars[1] << 8) | rx_chars[0];
                    mesg_len -= 2;
                }

                SIM800_UART_Get_Chars(msg, mesg_len, 5);

                SIM800_MQTT_Received_Callback(topic, msg, dup, qos, mesaage_id);
            }
            break;

        case SIM800_RESP_ANY:
        case SIM800_RESP_OK:
        case SIM800_RESP_SMS_READY:
        case SIM800_RESP_GPRS_READY:
        case SIM800_RESP_SHUT_OK:
        case SIM800_RESP_IP:
        case SIM800_RESP_CONNECT:
            break;
        }
    }
    else
    {
        /** in AT mode */
        char line[32] = "";

        switch (SIM800_Expected_Response)
        {
        case SIM800_RESP_NONE:
            break;

        case SIM800_RESP_ANY:
            break;

        case SIM800_RESP_OK:
            if (SIM800_Check_Response("OK", 5))
            {
                SIM800_Received_Response = SIM800_RESP_OK;
                SIM800_Expected_Response = SIM800_RESP_NONE;
            }
            break;

        case SIM800_RESP_SMS_READY:
            if (SIM800_Check_Response("SMS Ready", 5))
            {
                SIM800_Received_Response = SIM800_RESP_SMS_READY;
                SIM800_Expected_Response = SIM800_RESP_NONE;
            }
            break;

        case SIM800_RESP_GPRS_READY:
            if (SIM800_Check_Response("+CGATT: 1", 5))
            {
                SIM800_Received_Response = SIM800_RESP_GPRS_READY;
                SIM800_Expected_Response = SIM800_RESP_NONE;
            }
            break;

        case SIM800_RESP_SHUT_OK:
            if (SIM800_Check_Response("SHUT OK", 5))
            {
                SIM800_Received_Response = SIM800_RESP_GPRS_READY;
                SIM800_Expected_Response = SIM800_RESP_NONE;
            }
            break;

        case SIM800_RESP_IP:
            if (SIM800_Get_Response(line, 5))
            {
                if (CH_In_STR('.', line) == 3)
                { /** ip format xxx.xxx.xxx.xxx */
                    SIM800_Received_Response = SIM800_RESP_IP;
                    SIM800_Expected_Response = SIM800_RESP_NONE;
                }
            }
            break;

        case SIM800_RESP_CONNECT:
            if (SIM800_Check_Response("OK", 5))
            {
                if (SIM800_Check_Response("CONNECT", 5))
                {
                    SIM800_Received_Response = SIM800_RESP_CONNECT;
                    SIM800_Expected_Response = SIM800_RESP_NONE;
                }
            }
            break;

        case SIM800_RESP_MQTT_CONNACK:
        case SIM800_RESP_MQTT_PUBACK:
        case SIM800_RESP_MQTT_SUBACK:
        case SIM800_RESP_MQTT_PINGACK:
            break;
        }
    }
}
