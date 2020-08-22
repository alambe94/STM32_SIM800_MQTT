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
#include "MQTTPacket.h"

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
 * @brief uart read wrapper
 */
int transport_getdata(unsigned char *buf, int count)
{
    return SIM800_UART_Get_Chars((char *)buf, count, 5000);
}

/**
 * @brief send connect packet to broker
 * @param topic topic to which message will be published
 * @param mesaage message to published
 * @param mesaage_len message length
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Connect(MQTTPacket_connectData *param)
{
    unsigned char buf[128];
    uint32_t buflen = sizeof(buf);

    uint32_t len = MQTTSerialize_connect(buf, buflen, param);

    SIM800_UART_Send_Bytes((char *)buf, len);

    /* wait for connack */
    if (MQTTPacket_read(buf, buflen, transport_getdata) == CONNACK)
    {
        unsigned char sessionPresent, connack_rc;

        if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen) != 1 || connack_rc != 0)
        {
            printf("Unable to connect, return code %d\n", connack_rc);
            return 0;
        }
    }

    return 1;
}

/**
 * @brief send disconnect packet
 */
void SIM800_MQTT_Disconnect(void)
{
    unsigned char buf[32];
    uint32_t buflen = sizeof(buf);

    uint32_t len = MQTTSerialize_disconnect(buf, buflen);

    SIM800_UART_Send_Bytes((char *)buf, len);
}

/**
 * @brief send ping packet
 */
uint8_t SIM800_MQTT_Ping(void)
{
    unsigned char buf[32];
    uint32_t buflen = sizeof(buf);

    uint32_t len = MQTTSerialize_pingreq(buf, buflen);

    SIM800_UART_Send_Bytes((char *)buf, len);

    if (MQTTPacket_read(buf, buflen, transport_getdata) == PINGRESP)
    {
        return 1;
    }

    return 0;
}

/**
 * @brief publish message to a topic
 * @param topic topic to which message will be published
 * @param mesaage message to published
 * @param mesaage_len message length
 * @retval return 1 if success else 0
 */
uint8_t SIM800_MQTT_Publish(char *topic, char *mesaage, uint32_t mesaage_len, uint8_t dup, uint8_t qos, uint8_t retain, uint32_t mesaage_id)
{
    unsigned char buf[128];
    uint32_t buflen = sizeof(buf);

    MQTTString topicString;
    topicString.cstring = topic;

    uint32_t len = MQTTSerialize_publish(buf, buflen, dup, qos, retain, mesaage_id, topicString, (unsigned char *)mesaage, mesaage_len);

    SIM800_UART_Send_Bytes((char *)buf, len);

    if (MQTTPacket_read(buf, buflen, transport_getdata) == PUBACK) /* wait for puback */
    {
        unsigned char packettype;
        unsigned char dup;
        unsigned short packetid;

        return MQTTDeserialize_ack(&packettype, &dup, &packetid, buf, buflen);
    }

    return 0;
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
    unsigned char buf[128];
    uint32_t buflen = sizeof(buf);

    MQTTString topicFilters[1];
    int requestedQoSs[1];

    topicFilters[0].cstring = topic;
    requestedQoSs[0] = qos;

    uint32_t len = MQTTSerialize_subscribe(buf, buflen, 0, packet_id, 1, topicFilters, requestedQoSs);

    SIM800_UART_Send_Bytes((char *)buf, len);

    if (MQTTPacket_read(buf, buflen, transport_getdata) == SUBACK) /* wait for suback */
    {
        unsigned short submsgid;
        int subcount;
        int granted_qos;

        return MQTTDeserialize_suback(&submsgid, 1, &subcount, &granted_qos, buf, buflen);
    }

    return 0;
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
