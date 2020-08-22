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

/**
 * @brief Init peripheral ised by sim800
 */
uint8_t SIM800_Init(void)
{
    /** uart used for comm is configured in cube @see usart.c */

    uint8_t sim800_result = 0;

    HAL_GPIO_WritePin(RST_SIM800_GPIO_Port, RST_SIM800_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(RST_SIM800_GPIO_Port, RST_SIM800_Pin, GPIO_PIN_SET);
    HAL_Delay(5000);

    SIM800_UART_Init();

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

    lines = 20;
    while (lines--)
    {
        SIM800_UART_Send_String("AT+CGATT?\r\n"); /** GPRS Service’s status */
        sim800_result = SIM800_Check_Response("+CGATT: 1", 1000);
        if (sim800_result)
        {
            break;
        }
        sim800_result = SIM800_Check_Response("OK", 1000);
        HAL_Delay(3000);
    }

    SIM800_UART_Flush_RX();

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
    char sim800_reply[32];
    uint8_t sim800_result;

    SIM800_UART_Send_String("AT+CIPSHUT\r\n");
    sim800_result = SIM800_Check_Response("SHUT OK", 1000);

    SIM800_UART_Send_String("AT+CIPMODE=1\r\n");
    sim800_result = SIM800_Check_Response("OK", 1000);

    if (sim800_result)
    {
        /** assemble sim apn */
        SIM800_UART_Printf("AT+CSTT=\"%s\",\"\",\"\"\r\n", sim_apn);
        sim800_result = SIM800_Check_Response("OK", 3000);
    }

    HAL_Delay(3000);

    if (sim800_result)
    {
        /** Bring up wireless connection (GPRS or CSD) */
        SIM800_UART_Send_String("AT+CIICR\r\n");
        sim800_result = SIM800_Check_Response("OK", 3000);
    }

    if (sim800_result)
    {
        /** Get local IP address */
        SIM800_UART_Send_String("AT+CIFSR\r\n");
        SIM800_Get_Response(sim800_reply, 3000);
    }

    if (sim800_result)
    {
        /** assemble server ip and port */
        SIM800_UART_Printf("AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n",
                           broker,
                           port);
        sim800_result = SIM800_Check_Response("OK", 5000); /** expected reply "OK" and "CONNECT OK" within 10 second */
        sim800_result = SIM800_Check_Response("CONNECT", 5000);
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
    char conn_ack[4];

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

    SIM800_UART_Get_Chars(conn_ack, 4, 5000);

    if (conn_ack[0] == 0x20 && conn_ack[1] == 0x02 && conn_ack[2] == 0x00 && conn_ack[3] == 0x00)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief send disconnect packet
 */
void SIM800_MQTT_Disconnect(void)
{
}

/**
 * @brief send ping packet
 */
uint8_t SIM800_MQTT_Ping(void)
{
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
    char pub_ack[4];

    uint8_t topic_len = strlen(topic);

    uint8_t pub = 0x30 | (dup << 4) | (qos << 1) | retain;

    SIM800_UART_Send_Char(pub); /** MQTT publish fixed header */

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

    if (qos)
    {
        SIM800_UART_Send_Char(mesaage_id >> 8);
        SIM800_UART_Send_Char(mesaage_id & 0xFF);
    }

    SIM800_UART_Send_Bytes(mesaage, mesaage_len);

    SIM800_UART_Get_Chars(pub_ack, 4, 5000);

    if (qos)
    {
        if (pub_ack[0] == 0x40 && pub_ack[1] == 0x02 && (pub_ack[2] << 8 | pub_ack[3]) == mesaage_id)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    return 1;
}

/**
 * @brief subscribe to a topic
 * @param topic topic to be subscribe to
 * @param packet_id message ID can be arbitrary? TODO
 * @param qos 0, 1 or 2
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos)
{
    char sub_ack[5];

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

    SIM800_UART_Get_Chars(sub_ack, 5, 5000);

    if (sub_ack[0] == 0x90 &&
        sub_ack[1] == 0x02 &&
        (sub_ack[2] << 8 | sub_ack[3]) == packet_id &&
        (sub_ack[4] == 0 || sub_ack[4] == 1 || sub_ack[4] == 2))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

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
    while (SIM800_UART_Get_Count())
    {
    }
}
