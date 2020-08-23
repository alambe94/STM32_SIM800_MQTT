/** standard includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>

/** ST includes */
#include "main.h"

/** app includes */
#include "sim800_mqtt.h"
#include "sim800_uart.h"

struct PUBACK_Status_t
{
    uint8_t ACK_Flag;
    uint16_t Packet_ID;
} PUBACK_Status;

struct CONNACK_Status_t
{
    uint8_t ACK_Flag;
    uint8_t RC; /** return code */
} CONNACK_Status;

struct SUBACK_Status_t
{
    uint8_t ACK_Flag;
    uint8_t RC; /** return code */
} SUBACK_Status;

struct Ping_Status_t
{
    uint8_t Ping_Flag;
    uint8_t RL; /** remaining length */
} Ping_Status;

enum SIM800_Response_t
{
    SIM800_RESP_NONE,
    SIM800_RESP_ANY,
    SIM800_RESP_OK,
    SIM800_RESP_IP,
    SIM800_RESP_CONNECT,
    SIM800_RESP_SHUT_OK,
    SIM800_RESP_SMS_READY,
    SIM800_RESP_GPRS_READY,
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
    SIM800_TCP_CONNECTING,
    SIM800_TCP_CONNECTED, /** after thismodem is in transparent mode */
    SIM800_MQTT_CONNECTING,
    SIM800_MQTT_CONNECTED,
    SIM800_PUBLISHING,
    SIM800_RECEIVING,
} SIM800_State;

typedef enum SIM800_Status_t
{
    SIM800_SUCCESS,
    SIM800_BUSY,
    SIM800_FAILED
} SIM800_Status_t;

/**
  * @brief  one of the unused gpio pin interrupt is used to handle SIM800 background task
  *         it will be software triggered by sim800 uart idle interrupt
  * @note   set NVIC to 4 bit preemption 0 bit for sub priority
  */
void SIM800_Task_Init(void)
{
    HAL_NVIC_SetPriority((IRQn_Type)EXTI1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)EXTI1_IRQn);
}

/**
  * @brief trigger sim800 interrupt
  */
void SIM800_Task_Trigger(void)
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
}

/**
 * @brief reset sim800
 */
SIM800_Status_t SIM800_Reset(void)
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
 * @brief open tcp connection to mqtt broker
 * @param sim_apn simcard apn such "www" for vodafone and "airtelgprs.com" for airtel
 * @param broker broker mqtt address
 * @param port   broker mqtt port
 * @retval return 1 if success else 0
 */
uint8_t SIM800_TCP_Connect(char *sim_apn, char *broker, uint16_t port)
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
                SIM800_UART_Printf("AT+CSTT=\"%s\",\"\",\"\"\r\n", sim_apn);
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
                                   broker,
                                   port);
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
uint8_t SIM800_MQTT_Connect(char *protocol_name,
                            uint8_t protocol_version,
                            uint8_t flags,
                            uint32_t keep_alive,
                            char *my_id,
                            char *user_name,
                            char *password)
{
    static uint8_t conn_step = 0;
    static SIM800_Status_t sim800_result = SIM800_BUSY;

    static uint32_t next_delay = 0;
    static uint32_t loop_ticks = 0;

    if (HAL_GetTick() - loop_ticks > next_delay)
    {
        loop_ticks = HAL_GetTick();

        if (conn_step == 0)
        {
            conn_step = 1;

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

            SIM800_Expected_Response = SIM800_RESP_MQTT_CONNACK;
            sim800_result = SIM800_BUSY;
            next_delay = 300;
        }
        else
        {
            conn_step = 0;

            /** check response of previous cmd (MQTT connect) */
            if (SIM800_Received_Response == SIM800_RESP_MQTT_CONNACK)
            {
                SIM800_Received_Response = SIM800_RESP_NONE;
                sim800_result = SIM800_SUCCESS;
                next_delay = 0;
            }
            else
            {
                sim800_result = SIM800_FAILED;
            }
        }
    }
    return sim800_result;
}

/**
 * @brief send disconnect packet
 */
void SIM800_MQTT_Disconnect(void)
{
    SIM800_UART_Send_Char(0xD0); /** MQTT disconnect */
    SIM800_UART_Send_Char(0x00);
}

/**
 * @brief send ping packet
 */
uint8_t SIM800_MQTT_Ping(void)
{
    SIM800_UART_Send_Char(0xC0); /** MQTT ping */
    SIM800_UART_Send_Char(0x00);

    SIM800_Expected_Response = SIM800_RESP_MQTT_PINGACK;

    return 1;
}

/**
 * @brief publish message to a topic
 * @param topic topic to which message will be published
 * @param mesaage message to published
 * @param mesaage_len message length
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Publish(char *topic, char *mesaage, uint32_t mesaage_len, uint8_t dup, uint8_t qos, uint8_t retain, uint16_t mesaage_id)
{
    uint8_t topic_len = strlen(topic);

    uint8_t pub = 0x30 | (dup << 4) | (qos << 1) | retain;

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

    SIM800_UART_Send_Bytes(mesaage, mesaage_len);

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
}

/**
 * @brief mqtt processing loop TODO
 */
void SIM800_MQTT_Loop()
{
    SIM800_Status_t sim800_result;

    switch (SIM800_State)
    {
    case SIM800_IDLE:
        SIM800_State = SIM800_RESETING;
        SIM800_Init();
        break;

    case SIM800_RESETING:
        sim800_result = SIM800_Reset();
        if (sim800_result == SIM800_SUCCESS)
        {
            SIM800_State = SIM800_TCP_CONNECTING;
        }
        else if (sim800_result == SIM800_FAILED)
        {
            SIM800_State = SIM800_IDLE;
        }

    case SIM800_TCP_CONNECTING:
        sim800_result = SIM800_TCP_Connect("", "", 123);
        if (sim800_result == SIM800_SUCCESS)
        {
            SIM800_State = SIM800_MQTT_CONNECTING;
        }
        else if (sim800_result == SIM800_FAILED)
        {
            SIM800_State = SIM800_IDLE;
        }

    case SIM800_MQTT_CONNECTING:
        sim800_result = SIM800_MQTT_Connect("", 4, 0xC2, 60, "", "", "");
        if (sim800_result == SIM800_SUCCESS)
        {
            SIM800_State = SIM800_MQTT_CONNECTED;
        }
        else if (sim800_result == SIM800_FAILED)
        {
            SIM800_State = SIM800_IDLE;
        }
        break;

    default:
        break;
    }
}

/******************** callbacks **************************/
/**
 * @brief called when message is received TODO
 * @param topic topic on which message is received
 * @param message received message
 */
void SIM800_MQTT_Received_Callback(char *topic, char *message)
{
}

/**
 * @brief called when PUBACK is received
 * @param topic message on which ack is received
 */
void SIM800_MQTT_PUBACK_Callback(uin16_t mesaage_id)
{
}

/**
 * @brief called when SUBACK is received
 * @param topic message on which ack is received
 */
void SIM800_MQTT_PUBACK_Callback(uin16_t packet_id)
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
  * @brief trigger software in sim800 uart idle isr
  */
void EXTI1_IRQHandler(void)
{
}
