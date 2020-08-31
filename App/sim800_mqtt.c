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

struct SIM800_Response_Flags_t
{
    uint8_t SIM800_RESP_OK;
    uint8_t SIM800_RESP_SMS_READY;
    uint8_t SIM800_RESP_CALL_READY;
    uint8_t SIM800_RESP_GPRS_READY;
    uint8_t SIM800_RESP_TIME;
    uint8_t SIM800_RESP_SHUT_OK;
    uint8_t SIM800_RESP_IP;
    uint8_t SIM800_RESP_CONNECT;

    uint8_t SIM800_RESP_MQTT_CONNACK;
    uint8_t SIM800_RESP_MQTT_PUBACK;
    uint8_t SIM800_RESP_MQTT_SUBACK;
    uint8_t SIM800_RESP_MQTT_PINGACK;
} SIM800_Response_Flags;

typedef enum SIM800_Status_t
{
    SIM800_SUCCESS,
    SIM800_BUSY,
    SIM800_FAILED
} SIM800_Status_t;

struct SIM800_PUBACK_Data_t
{
    uint8_t Flag;
    uint16_t Message_ID;
} SIM800_PUBACK_Data;

/** hold sim800 state */
static enum SIM800_State_t SIM800_State;

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
            cnt++;
        }
    }

    return cnt;
}

/**
  * @brief  one of the unused gpio pin interrupt is used to handle SIM800 background task
  *         it will be software triggered by sim800 uart idle interrupt
  * @note   set NVIC to 4 bit for preemption 0 bit for sub priority, this interrupt priority < systick 
  * @warning interrupt on this GPIO can no longer be used and usage is not visible on cubeMX generator
  */
void SIM800_RX_Task_Init(void)
{
    HAL_NVIC_SetPriority((IRQn_Type)EXTI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)EXTI1_IRQn);
}

/**
  * @brief  one of the timer is used to generate periodic interrupt at 10ms (adjustable)
  * @note set NVIC to 4 bit preemption 0 bit for sub priority, this interrupt priority < systick 
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

    SIM800_State = SIM800_IDLE;

    /** timer used for tx task is configured in cube @see tim.c */
    HAL_TIM_Base_Start_IT(&htim14);
}

/**
 * @brief return the received chars from sim800
 * @param buff response destination
 * @param buff_size max count that can be written to buffer before '\r' is found
 * @param timeout max wait time in milliseconds
 * @retval number chars in response
 */
uint32_t SIM800_Get_Response(char *buff, uint32_t buff_size, uint32_t timeout)
{
    SIM800_UART_Get_Line(buff, buff_size, timeout); /** ignore first '\r' */
    return SIM800_UART_Get_Line(buff, buff_size, timeout);
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

    SIM800_UART_Get_Line(reply, sizeof(reply), timeout); /** ignore first '\r' */
    SIM800_UART_Get_Line(reply, sizeof(reply), timeout);

    res = strstr(reply, buff);

    return (res != NULL);
}

/**
 * @brief return 1 if sim800 is connected to broker
 */
uint8_t SIM800_Is_MQTT_Connected(void)
{
    return (SIM800_State >= SIM800_MQTT_RECEIVING);
}

/**
 * @brief return the state of sim800
 */
enum SIM800_State_t SIM800_Get_State(void)
{
    return SIM800_State;
}

/**
 * @brief get network time
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_Get_Time(void)
{
    if (SIM800_State < SIM800_TCP_CONNECTED)
    {
        SIM800_UART_Send_String("AT+CCLK?\r\n");
        return 1;
    }

    return 0;
}

/**
 * @brief reset sim800
 *        result callback is @see SIM800_Reset_complete_Callback
 * @param none
 * @retval return 1 if command can be executed     
 */
uint8_t SIM800_Reset(void)
{
    SIM800_State = SIM800_RESETING;
    return 1;
}
static SIM800_Status_t _SIM800_Reset(void)
{
    static SIM800_Status_t sim800_result = SIM800_BUSY;

    static uint32_t next_delay = 0;
    static uint32_t loop_ticks = 0;

    if (HAL_GetTick() - loop_ticks > next_delay)
    {
        static uint8_t reset_step = 0;
        static uint8_t retry = 0;

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
            reset_step++;
            next_delay = 100;
            break;

        case 3:
            /** disable echo */
            SIM800_UART_Send_String("ATE0\r\n");
            reset_step++;
            next_delay = 1000;
            break;

        case 4:
            SIM800_UART_Flush_RX();
            SIM800_Response_Flags.SIM800_RESP_OK = 0;
            reset_step++;
            next_delay = 100;
            break;

        case 5:
            /** wait SMS Ready flag */
            if (SIM800_Response_Flags.SIM800_RESP_SMS_READY)
            {
                SIM800_Response_Flags.SIM800_RESP_SMS_READY = 0;
                reset_step++;
                next_delay = 100;
                retry = 0;
            }
            else
            {
                next_delay = 2000;
                retry++;
                if (retry == 10)
                {
                    reset_step = 0;
                    sim800_result = SIM800_FAILED; /** failed */
                }
            }
            break;

        case 6:
            /** wait GPRS Service’s status */
            if (SIM800_Response_Flags.SIM800_RESP_GPRS_READY)
            {
                SIM800_Response_Flags.SIM800_RESP_GPRS_READY = 0;
                reset_step++;
                retry = 0;
                next_delay = 1000;
            }
            else
            {
                SIM800_UART_Send_String("AT+CGATT?\r\n");
                next_delay = 3000;
                retry++;
                if (retry == 20)
                {
                    reset_step = 0;
                    sim800_result = SIM800_FAILED; /** failed */
                }
            }
            break;

        case 7:
            /** wait for network time */
            if (SIM800_Response_Flags.SIM800_RESP_TIME)
            {
                SIM800_Response_Flags.SIM800_RESP_TIME = 0;
                reset_step++;
                retry = 0;
                next_delay = 1000;
            }
            else
            {
                SIM800_UART_Send_String("AT+CCLK?\r\n");
                next_delay = 1000;
                retry++;
                if (retry == 2)
                {
                    reset_step = 0;
                    sim800_result = SIM800_SUCCESS; /** return success even if time failed */
                }
            }
            break;

        case 8:
            SIM800_UART_Flush_RX();
            reset_step = 0;
            retry = 0;
            sim800_result = SIM800_SUCCESS;
            SIM800_Response_Flags.SIM800_RESP_OK = 0;
            break;
        }
    }

    return sim800_result;
}

/**
 * @brief open tcp connection to mqtt broker
 *        result callback is @see SIM800_TCP_CONN_complete_Callback
 * @param sim_apn simcard apn such "www" for vodafone and "airtelgprs.com" for airtel
 * @param broker broker mqtt address
 * @param port   broker mqtt port
 * @retval return 1 if command can be executed
 */
struct TCP_Data_t
{
    char _SIM_APN[32];
    char _Broker[32];
    uint16_t _Port;
} _TCP_Data;
uint8_t SIM800_TCP_Connect(char *sim_apn, char *broker, uint16_t port)
{
    if (SIM800_State == SIM800_RESET_OK)
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

    static uint32_t next_delay = 0;
    static uint32_t loop_ticks = 0;

    if (HAL_GetTick() - loop_ticks > next_delay)
    {
        static uint8_t tcp_step = 0;

        loop_ticks = HAL_GetTick();

        switch (tcp_step)
        {
        case 0:
            SIM800_UART_Send_String("AT+CIPSHUT\r\n");
            sim800_result = SIM800_BUSY;
            next_delay = 10;
            tcp_step++;
            break;

        case 1:
            /** check response of previous cmd (AT+CIPSHUT) */
            if (SIM800_Response_Flags.SIM800_RESP_SHUT_OK)
            {
                SIM800_Response_Flags.SIM800_RESP_SHUT_OK = 0;
                SIM800_UART_Send_String("AT+CIPMODE=1\r\n");
                next_delay = 10;
                tcp_step++;
            }
            else
            {
                tcp_step = 0;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 2:
            /** check response of previous cmd (AT+CIPMODE=1) */
            if (SIM800_Response_Flags.SIM800_RESP_OK)
            {
                SIM800_Response_Flags.SIM800_RESP_OK = 0;
                /** assemble sim apn */
                SIM800_UART_Printf("AT+CSTT=\"%s\",\"\",\"\"\r\n", _TCP_Data._SIM_APN);
                next_delay = 3000;
                tcp_step++;
            }
            else
            {
                tcp_step = 0;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 3:
            /** check response of previous cmd (AT+CSTT=) */
            if (SIM800_Response_Flags.SIM800_RESP_OK)
            {
                SIM800_Response_Flags.SIM800_RESP_OK = 0;
                /** Bring up wireless connection (GPRS or CSD) */
                SIM800_UART_Send_String("AT+CIICR\r\n");
                next_delay = 1000;
                tcp_step++;
            }
            else
            {
                tcp_step = 0;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 4:
            /** check response of previous cmd (AT+CIICR) */
            if (SIM800_Response_Flags.SIM800_RESP_OK)
            {
                SIM800_Response_Flags.SIM800_RESP_OK = 0;
                /** Get local IP address */
                SIM800_UART_Send_String("AT+CIFSR\r\n");
                next_delay = 100;
                tcp_step++;
            }
            else
            {
                tcp_step = 0;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 5:
            /** check response of previous cmd (AT+CIFSR) */
            if (SIM800_Response_Flags.SIM800_RESP_IP)
            {
                SIM800_Response_Flags.SIM800_RESP_IP = 0;
                /** assemble server ip and port */
                SIM800_UART_Printf("AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n",
                                   _TCP_Data._Broker,
                                   _TCP_Data._Port);
                next_delay = 1000;
                tcp_step++;
            }
            else
            {
                tcp_step = 0;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 6:
            /** check response of previous cmd (AT+CIPSTART) */
            if (SIM800_Response_Flags.SIM800_RESP_CONNECT && SIM800_Response_Flags.SIM800_RESP_OK)
            {
                SIM800_Response_Flags.SIM800_RESP_CONNECT = 0;
                SIM800_Response_Flags.SIM800_RESP_OK = 0;
                sim800_result = SIM800_SUCCESS;
                next_delay = 100;
                tcp_step = 0;
            }
            else
            {
                tcp_step = 0;
                sim800_result = SIM800_FAILED;
            }
            break;
        }
    }

    return sim800_result;
}

/**
 * @brief send connect packet to broker
 *        result callback is @see SIM800_MQTT_CONNACK_Callback 
 * @param protocol_version used mqtt version 3 for 3.1 and 4 for 3.1.1
 * @param bitfields for control flags
 * @param keep_alive keep alive interval in seconds
 * @param my_id clients unique ID across mqtt broker
 * @param user_name user name for mqtt broker
 * @param password password for mqtt broker
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_MQTT_Connect(char *protocol_name,
                            uint8_t protocol_version,
                            CONN_Flag_t flags,
                            uint16_t keep_alive,
                            char *my_id,
                            char *user_name,
                            char *password)
{
    if (SIM800_State < SIM800_TCP_CONNECTED)
    {
        return 0;
    }

    uint8_t protocol_name_len = strlen(protocol_name);
    uint8_t my_id_len = strlen(my_id);

    uint16_t user_name_len = 0;
    uint16_t password_len = 0;

    uint32_t packet_len = 2 + protocol_name_len + 1 + 1 + 2 + 2 + my_id_len;

    if (flags.Bits.User_Name && user_name != NULL)
    {
        user_name_len = strlen(user_name);
        packet_len += 2 + user_name_len;
        if (flags.Bits.Password && password != NULL)
        {
            password_len = strlen(password);
            packet_len += 2 + password_len;
        }
    }

    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(0x10); /** MQTT connect fixed header */

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

    SIM800_UART_Send_Char(flags.C_Flags);

    SIM800_UART_Send_Char(keep_alive >> 8);
    SIM800_UART_Send_Char(keep_alive & 0xFF);

    SIM800_UART_Send_Char(my_id_len >> 8);
    SIM800_UART_Send_Char(my_id_len & 0xFF);
    SIM800_UART_Send_String(my_id);

    if (flags.Bits.User_Name)
    {
        SIM800_UART_Send_Char(user_name_len >> 8);
        SIM800_UART_Send_Char(user_name_len & 0xFF);
        SIM800_UART_Send_String(user_name);

        if (flags.Bits.Password)
        {
            SIM800_UART_Send_Char(password_len >> 8);
            SIM800_UART_Send_Char(password_len & 0xFF);
            SIM800_UART_Send_String(password);
        }
    }

    SIM800_State = SIM800_MQTT_RECEIVING; /** indicates uart tx is done */

    return 1;
}

/**
 * @brief send disconnect packet
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_MQTT_Disconnect(void)
{
    if (!SIM800_Is_MQTT_Connected() && SIM800_State != SIM800_MQTT_TRANSMITTING)
    {
        return 0;
    }

    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(0xD0); /** MQTT disconnect */
    SIM800_UART_Send_Char(0x00);

    SIM800_State = SIM800_TCP_CONNECTED;

    return 1;
}

/**
 * @brief send ping packet
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_MQTT_Ping(void)
{
    if (!SIM800_Is_MQTT_Connected() && SIM800_State != SIM800_MQTT_TRANSMITTING)
    {
        return 0;
    }

    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(0xC0); /** MQTT ping */
    SIM800_UART_Send_Char(0x00);

    SIM800_State = SIM800_MQTT_RECEIVING; /** indicates uart tx is done */

    return 1;
}

/**
 * @brief publish message to a topic
 * @param topic topic to which message will be published
 * @param message message to published
 * @param message_len message length
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_MQTT_Publish(char *topic,
                            char *message,
                            uint32_t message_len,
                            uint8_t dup,
                            uint8_t qos,
                            uint8_t retain,
                            uint16_t message_id)
{
    if (!SIM800_Is_MQTT_Connected() && SIM800_State != SIM800_MQTT_TRANSMITTING)
    {
        return 0;
    }

    uint8_t topic_len = strlen(topic);

    uint8_t pub = 0x30 | ((dup & 0x01) << 3) | ((qos & 0x03) << 1) | (retain & 0x01);

    SIM800_State = SIM800_MQTT_TRANSMITTING; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(pub); /** MQTT publish fixed header */

    uint32_t packet_len = 2 + topic_len + message_len;

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
        SIM800_UART_Send_Char(message_id >> 8);
        SIM800_UART_Send_Char(message_id & 0xFF);
    }

    if (message_len > 64)
    {
        /** non blocking, SIM800_MQTT_TRANSMITTING will be cleared in uart tx dma isr */
        SIM800_UART_Send_Bytes_DMA(message, message_len);
    }
    else
    {
        /** blocking */
        SIM800_UART_Send_Bytes(message, message_len);
        SIM800_State = SIM800_MQTT_RECEIVING; /** indicates uart tx is done */
    }

    return 1;
}

/**
 * @brief subscribe to a topic
 * @param topic topic to be subscribe to
 * @param packet_id message ID can be arbitrary?
 * @param qos 0, 1
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos)
{
    if (!SIM800_Is_MQTT_Connected() && SIM800_State != SIM800_MQTT_TRANSMITTING)
    {
        return 0;
    }

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

    SIM800_State = SIM800_MQTT_RECEIVING; /** indicates uart tx is done */

    return 1;
}

/******************** callbacks **************************/
/**
 * @brief called when SIM800 reset sequence is complete
 *        callback response for @see SIM800_Reset
 * @param code reset success(GPRS is active) of failed
 */
void SIM800_Reset_complete_Callback(SIM800_Status_t status)
{
    if (status == SIM800_SUCCESS)
    {
        SIM800_State = SIM800_RESET_OK;
        APP_SIM800_Reset_CB(1);
    }
    else
    {
        // failed
        SIM800_State = SIM800_IDLE;
        APP_SIM800_Reset_CB(0);
    }
}

/**
 * @brief called when TCP connection is established
 *        callback response for @see SIM800_TCP_Connect
 * @param code TCP connection success of failed
 */
void SIM800_TCP_CONN_complete_Callback(SIM800_Status_t status)
{
    if (status == SIM800_SUCCESS)
    {
        SIM800_State = SIM800_TCP_CONNECTED;
        APP_SIM800_TCP_CONN_CB(1);
    }
    else
    {
        // failed
        SIM800_State = SIM800_RESET_OK;
        APP_SIM800_TCP_CONN_CB(0);
    }
}

/**
 * @brief called when CONNACK is received
 *        callback response for @see SIM800_MQTT_Connect
 * @param code return code from broker
 */
void SIM800_MQTT_CONNACK_Callback(uint16_t code)
{
    if (code == 0)
    {
        SIM800_State = SIM800_MQTT_RECEIVING;
        APP_SIM800_MQTT_CONN_CB(1);
    }
    else
    {
        SIM800_State = SIM800_RESET_OK;
        APP_SIM800_MQTT_CONN_CB(0);
    }
}

/**
 * @brief called when PUBACK is received
 *        callback response for @see SIM800_MQTT_Publish
 * @param message_id message on which ack is received
 */
void SIM800_MQTT_PUBACK_Callback(uint16_t message_id)
{
    APP_SIM800_MQTT_PUBACK_CB(message_id);
}

/**
 * @brief called when SUBACK is received
 *        callback response for @see SIM800_MQTT_Subscribe
 * @param packet_id packet on which ack is received
 * @param qos of topic
 */
void SIM800_MQTT_SUBACK_Callback(uint16_t packet_id, uint8_t qos)
{
    APP_SIM800_MQTT_SUBACK_CB(packet_id, qos);
}

/**
 * @brief called when ping response is received
 *        callback response for @see SIM800_MQTT_Ping
 */
void SIM800_MQTT_Ping_Callback()
{
    APP_SIM800_MQTT_Ping_CB();
}

/**
 * @brief called when network time is received
 *        callback response for @see SIM800_Get_Time
 */
void SIM800_MQTT_Date_Time_Callback(struct SIM800_Date_Time_t *dt)
{
    APP_SIM800_Date_Time_CB(dt);
}

/**
 * @brief called when message is received
 * @param topic topic on which message is received
 * @param message received message
 * @param msg_len message length
 * @param dup duplicates flag
 * @param qos qos of received message
 * @param message_id message id
 */
void SIM800_MQTT_Received_Callback(char *topic,
                                   char *message,
                                   uint32_t msg_len,
                                   uint8_t dup,
                                   uint8_t qos,
                                   uint16_t message_id)
{
    APP_SIM800_MQTT_MSG_CB(topic, message, msg_len, dup, qos, message_id);
}

/************************* ISR ***************************/
/**
 * @brief this sim800 TX TASK, called when sim800 timer expires every 10ms(adjustable)
 *        called from @see HAL_TIM_PeriodElapsedCallback in stm32f4xx_it.c
 **/
void SIM800_TIM_ISR(void)
{
    SIM800_Status_t sim800_result = SIM800_BUSY;

    switch (SIM800_State)
    {
    case SIM800_IDLE:
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

    case SIM800_RESET_OK:
        break;

    case SIM800_TCP_CONNECTING:
        sim800_result = _SIM800_TCP_Connect();
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

    case SIM800_MQTT_RECEIVING:
        if (SIM800_PUBACK_Data.Flag)
        {
            SIM800_PUBACK_Data.Flag = 0;
            SIM800_UART_Send_Char(0x40); /** PUBACK */
            SIM800_UART_Send_Char(0x02);
            SIM800_UART_Send_Char((SIM800_PUBACK_Data.Message_ID >> 8) & 0xFF);
            SIM800_UART_Send_Char(SIM800_PUBACK_Data.Message_ID & 0xFF);
        }
        break;
    case SIM800_MQTT_TRANSMITTING:
        break;
    }
}

/**
  * @brief this is sim800 RX TASK, software triggered by sim800 uart idle interrupt
  *        @see SIM800_RX_Task_Trigger & SIM800_UART_RX_ISR in sim800_uart.c
  */
void EXTI1_IRQHandler(void)
{
    while (SIM800_UART_Get_Count())
    {
        if (SIM800_State >= SIM800_TCP_CONNECTED)
        {
            /** in transparent mode */
            char rx_chars[64] = "";

            rx_chars[0] = SIM800_UART_Peek_Char();

            if (rx_chars[0] != -1)
            {
                /** published from broker received */
                if ((rx_chars[0] & 0x30) == 0x30)
                {
                    SIM800_UART_Get_Char();

                    uint8_t qos = (rx_chars[0] >> 1) & 0x03;
                    uint8_t dup = (rx_chars[0] >> 3) & 0x01;

                    uint32_t multiplier = 1;
                    uint32_t total_len = 0;
                    uint32_t msg_len = 0;
                    uint16_t topic_len = 0;
                    uint16_t message_id = 0;

                    char topic[32] = "";
                    /** @warning  large local buffer */
                    static char msg[1500] = "";

                    do
                    {
                        SIM800_UART_Get_Chars(rx_chars, 1, 0);
                        total_len += (rx_chars[0] & 127) * multiplier;
                        multiplier *= 128;
                        if (multiplier > 128 * 128 * 128)
                        {
                            break;
                        }
                    } while ((rx_chars[0] & 128) != 0);

                    SIM800_UART_Get_Chars(rx_chars, 2, 0);
                    topic_len = (rx_chars[0] << 8) | rx_chars[1];

                    msg_len = total_len - topic_len - 2;

                    if (topic_len > sizeof(topic))
                    {
                        topic_len = sizeof(topic);
                        /**  TODO handle this exception */
                    }
                    SIM800_UART_Get_Chars(topic, topic_len, 0);

                    if (qos)
                    {
                        SIM800_UART_Get_Chars(rx_chars, 2, 0);
                        message_id = (rx_chars[0] << 8) | rx_chars[1];
                        msg_len -= 2;

                        SIM800_PUBACK_Data.Flag = 1; /** indicates need to send PUBACK*/
                        SIM800_PUBACK_Data.Message_ID = message_id;
                    }

                    if (msg_len > sizeof(msg))
                    {
                        msg_len = sizeof(msg);
                        /**  TODO handle this exception */
                    }
                    SIM800_UART_Get_Chars(msg, msg_len, 0);

                    SIM800_MQTT_Received_Callback(topic, msg, msg_len, dup, qos, message_id);
                }
                else if (rx_chars[0] == 0x20)
                {
                    SIM800_UART_Get_Chars(rx_chars, 4, 0);
                    {
                        if (rx_chars[0] == 0x20 && rx_chars[1] == 0x02)
                        {
                            SIM800_Response_Flags.SIM800_RESP_MQTT_CONNACK = 1;
                            SIM800_MQTT_CONNACK_Callback(rx_chars[2] << 8 | rx_chars[3]);
                        }
                    }
                }
                else if (rx_chars[0] == 0x40)
                {
                    SIM800_UART_Get_Chars(rx_chars, 4, 0);
                    if (rx_chars[0] == 0x40 && rx_chars[1] == 0x02)
                    {
                        SIM800_Response_Flags.SIM800_RESP_MQTT_PUBACK = 1;
                        SIM800_MQTT_PUBACK_Callback(rx_chars[2] << 8 | rx_chars[3]);
                    }
                }
                else if (rx_chars[0] == 0x90)
                {
                    SIM800_UART_Get_Chars(rx_chars, 5, 0);
                    if (rx_chars[0] == 0x90 && rx_chars[1] == 0x03)
                    {
                        SIM800_Response_Flags.SIM800_RESP_MQTT_PUBACK = 1;
                        SIM800_MQTT_SUBACK_Callback(rx_chars[2] << 8 | rx_chars[3], rx_chars[4]);
                    }
                }
                else if (rx_chars[0] == 0xD0)
                {
                    SIM800_UART_Get_Chars(rx_chars, 2, 0);
                    if (rx_chars[0] == 0xD0 && rx_chars[1] == 0x00)
                    {
                        SIM800_Response_Flags.SIM800_RESP_MQTT_PUBACK = 1;
                        SIM800_MQTT_Ping_Callback();
                    }
                }
                else if (rx_chars[0] == '\r')
                {
                    SIM800_UART_Get_Chars(rx_chars, 10, 0);
                    if (strstr(rx_chars, "\r\nCLOSED\r\n") != NULL)
                    {
                        /** TCP connection closed due to inactivity */
                        /** set SIM800_State to SIM800_RESET_OK to indicate new tcp connection is required */
                        SIM800_State = SIM800_RESET_OK;
                    }
                }
                else
                {
                    SIM800_UART_Get_Char();
                }
            }
        }
        else
        {
            /** in AT mode */
            char line[64] = "";

            SIM800_Get_Response(line, sizeof(line), 0);

            if (strcmp(line, "OK") == 0)
            {
                SIM800_Response_Flags.SIM800_RESP_OK = 1;
            }
            else if (strcmp(line, "Call Ready") == 00)
            {
                SIM800_Response_Flags.SIM800_RESP_CALL_READY = 1;
            }
            else if (strcmp(line, "SMS Ready") == 0)
            {
                SIM800_Response_Flags.SIM800_RESP_SMS_READY = 1;
            }
            else if (strcmp(line, "+CGATT: 1") == 0)
            {
                SIM800_Response_Flags.SIM800_RESP_GPRS_READY = 1;
            }
            else if (strcmp(line, "SHUT OK") == 0)
            {
                SIM800_Response_Flags.SIM800_RESP_SHUT_OK = 1;
            }
            else if (strcmp(line, "CONNECT") == 0)
            {
                SIM800_Response_Flags.SIM800_RESP_CONNECT = 1;
            }
            else if (CH_In_STR('.', line) == 3)
            {
                SIM800_Response_Flags.SIM800_RESP_IP = 1;
            }
            else if (strncmp(line, "+CCLK: ", 7) == 0)
            {
                SIM800_Response_Flags.SIM800_RESP_TIME = 1;
                struct SIM800_Date_Time_t dt;
                dt.Year = (line[8] - '0') * 10 + line[9] - '0';
                dt.Month = (line[11] - '0') * 10 + line[12] - '0';
                dt.Date = (line[14] - '0') * 10 + line[15] - '0';

                dt.Hours = (line[17] - '0') * 10 + line[18] - '0';
                dt.Minutes = (line[20] - '0') * 10 + line[21] - '0';
                dt.Seconds = (line[23] - '0') * 10 + line[24] - '0';
                SIM800_MQTT_Date_Time_Callback(&dt);
            }
        }
    }
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

/** WAEK callbacks need to define by user app ****/
__weak void APP_SIM800_Reset_CB(uint8_t reset_ok)
{
}
__weak void APP_SIM800_Date_Time_CB(struct SIM800_Date_Time_t *dt)
{
}
__weak void APP_SIM800_TCP_CONN_CB(uint8_t tcp_ok)
{
}
__weak void APP_SIM800_MQTT_CONN_CB(uint8_t mqtt_ok)
{
}
__weak void APP_SIM800_MQTT_PUBACK_CB(uint16_t message_id)
{
}
__weak void APP_SIM800_MQTT_SUBACK_CB(uint16_t packet_id, uint8_t qos)
{
}
__weak void APP_SIM800_MQTT_Ping_CB(void)
{
}
__weak void APP_SIM800_MQTT_MSG_CB(char *topic,
                                   char *message,
                                   uint32_t msg_len,
                                   uint8_t dup,
                                   uint8_t qos,
                                   uint16_t message_id)
{
}
