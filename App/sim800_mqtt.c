/** standard includes */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/** ST includes */
#include "main.h"

/** app includes */
#include "sim800_mqtt.h"
#include "sim800_uart.h"
#include "MQTTPacket.h"
#include "transport.h"

enum states
{
    READING,
    PUBLISHING
};

int SIM800_Send(unsigned char *address, unsigned int bytes);
int SIM800_Receive(unsigned char *address, unsigned int maxbytes);

static transport_iofunctions_t iof = {SIM800_Send, SIM800_Receive};

int SIM800_Send(unsigned char *address, unsigned int bytes)
{
	return 1;
}

int SIM800_Receive(unsigned char *address, unsigned int maxbytes)
{
	return 1;
}

void MQTT_Connect(void)
{
    MQTTPacket_connectData conn_packet = MQTTPacket_connectData_initializer;
    MQTTTransport mytransport;

    int mysock = 0;
    unsigned char buf[200];
    int buflen = sizeof(buf);
    int msgid = 1;
    int len = 0;

    // connect to TCP

    mysock = transport_open(&iof);

    /*  You will (or already are) 'somehow' connect(ed) to host:port via your hardware specifics. E.g.:
		you have a serial (RS-232/UART) link
		you have a cell modem and you issue your AT+ magic
		you have some TCP/IP which is not lwIP (nor a full-fledged socket compliant one)
		and you TCP connect
	*/

    mytransport.sck = &mysock;
    mytransport.getfn = transport_getdatanb;
    mytransport.state = 0;

    conn_packet.clientID.cstring = "me";
    conn_packet.keepAliveInterval = 20;
    conn_packet.cleansession = 1;
    conn_packet.username.cstring = "testuser";
    conn_packet.password.cstring = "testpassword";

    len = MQTTSerialize_connect(buf, buflen, &conn_packet);
    /* This one blocks until it finishes sending, you will probably not want this in real life,
	   in such a case replace this call by a scheme similar to the one you'll see in the main loop */
    transport_sendPacketBuffer(mysock, buf, len);

    printf("Sent MQTT connect\n");

    /* wait for connack */
    int frc;
    if ((frc = MQTTPacket_readnb(buf, buflen, &mytransport)) == CONNACK)
    {
        unsigned char sessionPresent, connack_rc;
        if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen) != 1 || connack_rc != 0)
        {
            printf("Unable to connect, return code %d\n", connack_rc);
        }
    }
    else if (frc == -1)
    {
    }

    printf("MQTT connected\n");

    len = MQTTSerialize_disconnect(buf, buflen);
    /* Same blocking related stuff here */
    transport_sendPacketBuffer(mysock, buf, len);

    transport_close(mysock);

    // close tcp
}





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
        SIM800_UART_Send_String("AT+CGATT?\r\n");                 /** GPRS Service’s status */
        sim800_result = SIM800_Check_Response("+CGATT: 1", 1000); /** expected reply  "+CGATT: 1" and "OK" within 1 second "*/
        if (sim800_result)
        {
            break;
        }
        sim800_result = SIM800_Check_Response("OK", 1000);
        HAL_Delay(3000);
    }

    SIM800_Flush_RX();

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
    uint8_t sim800_result;

    /**************************************************************** TCP connection start *********************************************************************/

    SIM800_UART_Send_String("AT+CIPSHUT\r\n");
    sim800_result = SIM800_Check_Response("SHUT OK", 1000); /** expected reply "OK" within 1 second "*/

    SIM800_UART_Send_String("AT+CIPMODE=1\r\n");
    sim800_result = SIM800_Check_Response("OK", 1000); /** expected reply "OK" within 1 second "*/

    if (sim800_result)
    {
        /** assemble sim apn */
        SIM800_UART_Printf("AT+CSTT=\"%s\",\"\",\"\"\r\n", sim_apn);
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
        sim800_result = SIM800_Check_Response("CONNECT", 5000);
    }

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
    return 1;
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
    while (1)
    {
    }
}


