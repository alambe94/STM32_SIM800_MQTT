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

typedef struct SIM800_Response_Flags_t
{
    uint8_t SIM800_RESP_OK;
    uint8_t SIM800_RESP_SMS_READY;
    uint8_t SIM800_RESP_CALL_READY;
    uint8_t SIM800_RESP_GPRS_READY;
    uint8_t SIM800_RESP_DATE_TIME;

    uint8_t SIM800_RESP_SHUT_OK;
    uint8_t SIM800_RESP_IP;
    uint8_t SIM800_RESP_CONNECT;
    uint8_t SIM800_RESP_CLOSED;

    uint8_t SIM800_RESP_MQTT_CONNACK;
    uint8_t SIM800_RESP_MQTT_PUBACK;
    uint8_t SIM800_RESP_MQTT_PUBREC;
    uint8_t SIM800_RESP_MQTT_SUBACK;
    uint8_t SIM800_RESP_MQTT_PINGACK;
} SIM800_Response_Flags_t;

typedef enum SIM800_Status_t
{
    SIM800_SUCCESS,
    SIM800_BUSY,
    SIM800_FAILED
} SIM800_Status_t;

/** store info about TCP */
typedef struct SIM800_TCP_Data_t
{
    char SIM_APN[32];
    char Broker_IP[32];
    char MY_IP[32];
    uint16_t Broker_Port;
    uint8_t TCP_Step;
    uint8_t TCP_Retry;
} SIM800_TCP_Data_t;

/** store info about MSG received from broker */
typedef struct MQTT_PUBREC_Data_t
{
    uint8_t PUBACK_Flag;
    uint8_t DUP;
    uint8_t QOS;
    uint16_t MSG_ID;
    uint32_t MSG_Len;
    char Topic[64];
    char MSG[1500];
} MQTT_PUBREC_Data_t;

/** store info about CONNACK received from broker */
typedef struct MQTT_CONNACK_Data_t
{
    uint16_t Code;
} MQTT_CONNACK_Data_t;

/** store info about PUBACK received from broker */
typedef struct MQTT_PUBACK_Data_t
{
    uint16_t MSG_ID;
} MQTT_PUBACK_Data_t;

/** store info about SUBACK received from broker */
typedef struct MQTT_SUBACK_Data_t
{
    uint8_t QOS;
    uint16_t MSG_ID;
} MQTT_SUBACK_Data_t;

typedef struct SIM800_Handle_t
{
    SIM800_Response_Flags_t RESP_Flags;
    SIM800_State_t State;

    uint8_t Reset_Step;
    uint8_t Reset_Retry;

    uint8_t UART_TX_Busy;
    uint8_t RX_Ready;

    SIM800_Date_Time_t Time;

    SIM800_TCP_Data_t TCP;

    MQTT_PUBREC_Data_t PUBREC;
    MQTT_CONNACK_Data_t CONNACK;
    MQTT_PUBACK_Data_t PUBACK;
    MQTT_SUBACK_Data_t SUBACK;

    uint32_t Next_Tick;

    HAL_LockTypeDef Lock_TX; /** lock TX state machine */
} SIM800_Handle_t;

/** hold sim800 handle */
SIM800_Handle_t hSIM800;

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
  * @brief  sim800 state machine, one of the timer is used to generate periodic interrupt at 10ms (adjustable)
  * @note set NVIC to 4 bit preemption 0 bit for sub priority, this interrupt priority < systick 
  */
void SIM800_SM_Task_Init(void)
{
    /** configured in cube @see tim.c */
}

/**
 * @brief Init peripheral used by sim800
 */
void SIM800_Init(void)
{
    /** uart used for comm is configured in cube @see usart.c */
    SIM800_UART_Init();

    SIM800_SM_Task_Init();

    hSIM800.State = SIM800_IDLE;

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
    return (hSIM800.State >= SIM800_MQTT_CONNECTED);
}

/**
 * @brief return the state of sim800
 */
SIM800_State_t SIM800_Get_State(void)
{
    return hSIM800.State;
}

/**
 * @brief get network time
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_Get_Time(void)
{
    if (hSIM800.State < SIM800_TCP_CONNECTED)
    {
        SIM800_UART_Send_String("AT+CCLK?\r\n");
        return 1;
    }

    return 0;
}

/**
 * @brief reset sim800
 *        result callback is @see SIM800_Reset_Complete_Callback
 * @param none
 * @retval return 1 if command can be executed     
 */
uint8_t SIM800_Reset(void)
{
    hSIM800.Lock_TX = 1;

    hSIM800.State = SIM800_RESETING;

    hSIM800.RESP_Flags.SIM800_RESP_OK = 0;
    hSIM800.RESP_Flags.SIM800_RESP_SMS_READY = 0;
    hSIM800.RESP_Flags.SIM800_RESP_CALL_READY = 0;
    hSIM800.RESP_Flags.SIM800_RESP_GPRS_READY = 0;
    hSIM800.RESP_Flags.SIM800_RESP_DATE_TIME = 0;
    hSIM800.RESP_Flags.SIM800_RESP_SHUT_OK = 0;
    hSIM800.RESP_Flags.SIM800_RESP_IP = 0;
    hSIM800.RESP_Flags.SIM800_RESP_CONNECT = 0;

    hSIM800.RESP_Flags.SIM800_RESP_MQTT_CONNACK = 0;
    hSIM800.RESP_Flags.SIM800_RESP_MQTT_PUBACK = 0;
    hSIM800.RESP_Flags.SIM800_RESP_MQTT_PUBREC = 0;
    hSIM800.RESP_Flags.SIM800_RESP_MQTT_SUBACK = 0;
    hSIM800.RESP_Flags.SIM800_RESP_MQTT_PINGACK = 0;

    hSIM800.Reset_Step = 0;
    hSIM800.Reset_Retry = 0;

    SIM800_UART_Restart();

    SIM800_UART_Flush_RX();

    /** start reset sequence after 100ms */
    hSIM800.Next_Tick = HAL_GetTick() + 100;

    hSIM800.Lock_TX = 0;

    return 1;
}
static SIM800_Status_t _SIM800_Reset(void)
{
    SIM800_Status_t sim800_result = SIM800_BUSY;

    if (HAL_GetTick() > hSIM800.Next_Tick)
    {
        uint32_t tick_now = HAL_GetTick();

        switch (hSIM800.Reset_Step)
        {
        case 0:
            HAL_GPIO_WritePin(RST_SIM800_GPIO_Port, RST_SIM800_Pin, GPIO_PIN_RESET);
            hSIM800.Reset_Step++;
            hSIM800.Next_Tick = tick_now + 1000;
            break;

        case 1:
            HAL_GPIO_WritePin(RST_SIM800_GPIO_Port, RST_SIM800_Pin, GPIO_PIN_SET);
            hSIM800.Reset_Step++;
            hSIM800.Next_Tick = tick_now + 5000;
            break;

        case 2:
            /** send dummy, so sim800 can auto adjust its baud */
            SIM800_UART_Send_String("AT\r\n");
            hSIM800.Reset_Step++;
            hSIM800.Next_Tick = tick_now + 100;
            break;

        case 3:
            if (hSIM800.RESP_Flags.SIM800_RESP_OK)
            {
                hSIM800.RESP_Flags.SIM800_RESP_OK = 0;
                /** disable echo */
                SIM800_UART_Send_String("ATE0\r\n");
                hSIM800.Reset_Step++;
                hSIM800.Next_Tick = tick_now + 1000;
            }
            else
            {
                hSIM800.Reset_Step = 0;
                hSIM800.Reset_Retry = 0;
                sim800_result = SIM800_FAILED; /** failed */
            }
            break;

        case 4:
            SIM800_UART_Flush_RX();
            hSIM800.RESP_Flags.SIM800_RESP_OK = 0;
            hSIM800.Reset_Step++;
            hSIM800.Next_Tick = tick_now + 100;
            break;

        case 5:
            /** wait SMS Ready flag */
            if (hSIM800.RESP_Flags.SIM800_RESP_SMS_READY)
            {
                hSIM800.RESP_Flags.SIM800_RESP_SMS_READY = 0;
                hSIM800.Reset_Step++;
                hSIM800.Next_Tick = tick_now + 100;
                hSIM800.Reset_Retry = 0;
            }
            else
            {
                hSIM800.Next_Tick = tick_now + 2000;
                hSIM800.Reset_Retry++;
                if (hSIM800.Reset_Retry >= 10)
                {
                    hSIM800.Reset_Step = 0;
                    hSIM800.Reset_Retry = 0;
                    sim800_result = SIM800_FAILED; /** failed */
                }
            }
            break;

        case 6:
            /** wait GPRS Service’s status */
            if (hSIM800.RESP_Flags.SIM800_RESP_GPRS_READY)
            {
                hSIM800.RESP_Flags.SIM800_RESP_GPRS_READY = 0;
                hSIM800.Reset_Step++;
                hSIM800.Reset_Retry = 0;
                hSIM800.Next_Tick = tick_now + 1000;
            }
            else
            {
                SIM800_UART_Send_String("AT+CGATT?\r\n");
                hSIM800.Next_Tick = tick_now + 3000;
                hSIM800.Reset_Retry++;
                if (hSIM800.Reset_Retry >= 20)
                {
                    hSIM800.Reset_Step = 0;
                    hSIM800.Reset_Retry = 0;
                    sim800_result = SIM800_FAILED; /** failed */
                }
            }
            break;

        case 7:
            /** wait for network time */
            if (hSIM800.RESP_Flags.SIM800_RESP_DATE_TIME)
            {
                hSIM800.RESP_Flags.SIM800_RESP_DATE_TIME = 0;
                APP_SIM800_Date_Time_CB(&hSIM800.Time);
                hSIM800.Reset_Step++;
                hSIM800.Reset_Retry = 0;
                hSIM800.Next_Tick = tick_now + 1000;
            }
            else
            {
                SIM800_UART_Send_String("AT+CCLK?\r\n");
                hSIM800.Next_Tick = tick_now + 1000;
                hSIM800.Reset_Retry++;
                if (hSIM800.Reset_Retry >= 2)
                {
                    hSIM800.Reset_Step++;
                }
            }
            break;

        case 8:
            SIM800_UART_Flush_RX();
            hSIM800.Reset_Step = 0;
            hSIM800.Reset_Retry = 0;
            hSIM800.Next_Tick = tick_now + 100;
            sim800_result = SIM800_SUCCESS;
            hSIM800.RESP_Flags.SIM800_RESP_OK = 0;
            break;
        }
    }

    return sim800_result;
}

/**
 * @brief open tcp connection to mqtt broker
 *        result callback is @see SIM800_TCP_CONN_Complete_Callback
 * @param sim_apn simcard apn such "www" for vodafone and "airtelgprs.com" for airtel
 * @param broker broker mqtt address
 * @param port   broker mqtt port
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_TCP_Connect(char *sim_apn, char *broker, uint16_t port)
{
    if (hSIM800.State == SIM800_RESET_OK)
    {
        hSIM800.Lock_TX = 1;

        snprintf(hSIM800.TCP.SIM_APN, sizeof(hSIM800.TCP.SIM_APN), "%s", sim_apn);
        snprintf(hSIM800.TCP.Broker_IP, sizeof(hSIM800.TCP.Broker_IP), "%s", broker);

        hSIM800.TCP.Broker_Port = port;

        hSIM800.TCP.TCP_Step = 0;
        hSIM800.TCP.TCP_Retry = 0;
        hSIM800.Next_Tick = HAL_GetTick() + 100;
        hSIM800.State = SIM800_TCP_CONNECTING;

        hSIM800.Lock_TX = 0;

        return 1;
    }

    return 0;
}
static uint8_t _SIM800_TCP_Connect()
{
    SIM800_Status_t sim800_result = SIM800_BUSY;

    if (HAL_GetTick() > hSIM800.Next_Tick)
    {
        uint32_t tick_now = HAL_GetTick();

        switch (hSIM800.TCP.TCP_Step)
        {
        case 0:
            SIM800_UART_Send_String("AT+CIPSHUT\r\n");
            sim800_result = SIM800_BUSY;
            hSIM800.Next_Tick = tick_now + 500;
            hSIM800.TCP.TCP_Step++;
            break;

        case 1:
            /** check response of previous cmd (AT+CIPSHUT) */
            if (hSIM800.RESP_Flags.SIM800_RESP_SHUT_OK)
            {
                hSIM800.RESP_Flags.SIM800_RESP_SHUT_OK = 0;
                SIM800_UART_Send_String("AT+CIPMODE=1\r\n");
                hSIM800.Next_Tick = tick_now + 500;
                hSIM800.TCP.TCP_Step++;
            }
            else
            {
                hSIM800.TCP.TCP_Step = 0;
                hSIM800.Next_Tick = tick_now + 100;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 2:
            /** check response of previous cmd (AT+CIPMODE=1) */
            if (hSIM800.RESP_Flags.SIM800_RESP_OK)
            {
                hSIM800.RESP_Flags.SIM800_RESP_OK = 0;
                /** assemble sim apn */
                SIM800_UART_Printf("AT+CSTT=\"%s\",\"\",\"\"\r\n", hSIM800.TCP.SIM_APN);
                hSIM800.Next_Tick = tick_now + 1000;
                hSIM800.TCP.TCP_Step++;
            }
            else
            {
                hSIM800.TCP.TCP_Step = 0;
                hSIM800.Next_Tick = tick_now + 100;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 3:
            /** check response of previous cmd (AT+CSTT=) */
            if (hSIM800.RESP_Flags.SIM800_RESP_OK)
            {
                hSIM800.RESP_Flags.SIM800_RESP_OK = 0;
                /** Bring up wireless connection (GPRS or CSD) */
                SIM800_UART_Send_String("AT+CIICR\r\n");
                hSIM800.Next_Tick = tick_now + 3000;
                hSIM800.TCP.TCP_Step++;
            }
            else
            {
                hSIM800.TCP.TCP_Step = 0;
                hSIM800.Next_Tick = tick_now + 100;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 4:
            /** check response of previous cmd (AT+CIICR) */
            if (hSIM800.RESP_Flags.SIM800_RESP_OK)
            {
                hSIM800.RESP_Flags.SIM800_RESP_OK = 0;
                /** Get local IP address */
                SIM800_UART_Send_String("AT+CIFSR\r\n");
                hSIM800.Next_Tick = tick_now + 3000;
                hSIM800.TCP.TCP_Step++;
            }
            else
            {
                hSIM800.TCP.TCP_Step = 0;
                hSIM800.Next_Tick = tick_now + 100;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 5:
            /** check response of previous cmd (AT+CIFSR) */
            if (hSIM800.RESP_Flags.SIM800_RESP_IP)
            {
                hSIM800.RESP_Flags.SIM800_RESP_IP = 0;
                APP_SIM800_IP_Address_CB(hSIM800.TCP.MY_IP);
                /** assemble server ip and port */
                SIM800_UART_Printf("AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n",
                                   hSIM800.TCP.Broker_IP,
                                   hSIM800.TCP.Broker_Port);
                hSIM800.Next_Tick = tick_now + 5000;
                hSIM800.TCP.TCP_Step++;
            }
            else
            {
                hSIM800.TCP.TCP_Step = 0;
                hSIM800.Next_Tick = tick_now + 100;
                sim800_result = SIM800_FAILED;
            }
            break;

        case 6:
            /** check response of previous cmd (AT+CIPSTART) */
            if (hSIM800.RESP_Flags.SIM800_RESP_CONNECT && hSIM800.RESP_Flags.SIM800_RESP_OK)
            {
                hSIM800.RESP_Flags.SIM800_RESP_CONNECT = 0;
                hSIM800.RESP_Flags.SIM800_RESP_OK = 0;
                sim800_result = SIM800_SUCCESS;
                hSIM800.Next_Tick = tick_now + 100;
                hSIM800.TCP.TCP_Step = 0;
            }
            else
            {
                hSIM800.TCP.TCP_Step = 0;
                hSIM800.Next_Tick = tick_now + 1000;
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
    if (hSIM800.State < SIM800_TCP_CONNECTED || hSIM800.UART_TX_Busy)
    {
        return 0;
    }

    uint8_t protocol_name_len = strnlen(protocol_name, 8); /** max length is set to arbitrary suitable value */
    uint8_t my_id_len = strnlen(my_id, 64);

    uint16_t user_name_len = 0;
    uint16_t password_len = 0;

    uint32_t packet_len = 2 + protocol_name_len + 1 + 1 + 2 + 2 + my_id_len;

    hSIM800.Lock_TX = 1;
    hSIM800.UART_TX_Busy = 1;
    hSIM800.State = SIM800_MQTT_CONNECTING;

    if (flags.Bits.User_Name && user_name != NULL)
    {
        user_name_len = strnlen(user_name, 64);
        packet_len += 2 + user_name_len;
        if (flags.Bits.Password && password != NULL)
        {
            password_len = strnlen(password, 128);
            packet_len += 2 + password_len;
        }
    }

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

    /** response must have been received within this period */
    hSIM800.Next_Tick = HAL_GetTick() + 5000;
    hSIM800.UART_TX_Busy = 0;
    hSIM800.Lock_TX = 0;

    return 1;
}
static SIM800_Status_t _SIM800_MQTT_Connect(void)
{
    SIM800_Status_t sim800_result = SIM800_BUSY;

    if (HAL_GetTick() > hSIM800.Next_Tick)
    {
        if (hSIM800.RESP_Flags.SIM800_RESP_MQTT_CONNACK)
        {
            hSIM800.RESP_Flags.SIM800_RESP_MQTT_CONNACK = 0;
            sim800_result = SIM800_SUCCESS;
        }
        else
        {
            sim800_result = SIM800_FAILED;
        }
    }

    return sim800_result;
}

/**
 * @brief send disconnect packet
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_MQTT_Disconnect(void)
{
    if (!SIM800_Is_MQTT_Connected() || hSIM800.UART_TX_Busy)
    {
        return 0;
    }

    hSIM800.Lock_TX = 1;
    hSIM800.UART_TX_Busy = 1; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(0xD0); /** MQTT disconnect */
    SIM800_UART_Send_Char(0x00);

    hSIM800.UART_TX_Busy = 0;
    hSIM800.State = SIM800_TCP_CONNECTED; /** mqtt disconnected, goto TCP connected */
    hSIM800.Lock_TX = 0;

    return 1;
}

/**
 * @brief send ping packet
 * @retval return 1 if command can be executed
 */
uint8_t SIM800_MQTT_Ping(void)
{
    if (!SIM800_Is_MQTT_Connected() || hSIM800.UART_TX_Busy)
    {
        return 0;
    }

    hSIM800.Lock_TX = 1;
    hSIM800.UART_TX_Busy = 1; /** indicates uart tx is busy */

    SIM800_UART_Send_Char(0xC0); /** MQTT ping */
    SIM800_UART_Send_Char(0x00);

    hSIM800.UART_TX_Busy = 0; /** indicates uart tx is done */
    hSIM800.Lock_TX = 0;

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
    if (!SIM800_Is_MQTT_Connected() || hSIM800.UART_TX_Busy)
    {
        return 0;
    }

    uint8_t topic_len = strnlen(topic, 128);

    uint8_t pub = 0x30 | ((dup & 0x01) << 3) | ((qos & 0x03) << 1) | (retain & 0x01);

    hSIM800.Lock_TX = 1;
    hSIM800.UART_TX_Busy = 1; /** indicates uart tx is busy */

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
        hSIM800.UART_TX_Busy = 0; /** indicates uart tx is done */
    }

    hSIM800.Lock_TX = 0;

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
    if (!SIM800_Is_MQTT_Connected() || hSIM800.UART_TX_Busy)
    {
        return 0;
    }

    uint8_t topic_len = strnlen(topic, 128);

    uint32_t packet_len = 2 + 2 + topic_len + 1;

    hSIM800.Lock_TX = 1;
    hSIM800.UART_TX_Busy = 1; /** indicates uart tx is busy */

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

    hSIM800.UART_TX_Busy = 0; /** indicates uart tx is done */
    hSIM800.Lock_TX = 0;

    return 1;
}

/**
 * @brief process received data on sim800 uart
 **/
void SIM800_RX_Process(void)
{
    while (SIM800_UART_Get_Count())
    {
        if (hSIM800.State >= SIM800_TCP_CONNECTED)
        {
            /** in transparent mode */
            char rx_chars[64] = "";

            rx_chars[0] = SIM800_UART_Peek_Char();

            if (rx_chars[0] != -1)
            {
                /** published msg from broker received */
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

                    if (topic_len > sizeof(hSIM800.PUBREC.Topic) - 1)
                    {
                        topic_len = sizeof(hSIM800.PUBREC.Topic) - 1;
                        /**  TODO handle this exception */
                    }
                    SIM800_UART_Get_Chars(hSIM800.PUBREC.Topic, topic_len, 0);
                    hSIM800.PUBREC.Topic[topic_len] = '\0';

                    if (qos)
                    {
                        SIM800_UART_Get_Chars(rx_chars, 2, 0);
                        message_id = (rx_chars[0] << 8) | rx_chars[1];
                        msg_len -= 2;

                        hSIM800.PUBREC.QOS = qos; /** indicates need to send PUBACK*/
                        hSIM800.PUBREC.MSG_ID = message_id;
                    }
                    else
                    {
                        hSIM800.PUBREC.QOS = 0;
                        hSIM800.PUBREC.MSG_ID = 0;
                    }

                    if (msg_len > sizeof(hSIM800.PUBREC.MSG) - 1)
                    {
                        msg_len = sizeof(hSIM800.PUBREC.MSG) - 1;
                        /**  TODO handle this exception */
                    }
                    SIM800_UART_Get_Chars(hSIM800.PUBREC.MSG, msg_len, 0);
                    hSIM800.PUBREC.MSG[msg_len] = '\0';
                    hSIM800.PUBREC.DUP = dup;
                    hSIM800.PUBREC.MSG_Len = msg_len;
                    hSIM800.RESP_Flags.SIM800_RESP_MQTT_PUBREC = 1;
                }
                else if (rx_chars[0] == 0x20)
                {
                    SIM800_UART_Get_Chars(rx_chars, 4, 0);
                    {
                        if (rx_chars[0] == 0x20 && rx_chars[1] == 0x02)
                        {
                            hSIM800.CONNACK.Code = rx_chars[2] << 8 | rx_chars[3];
                            hSIM800.RESP_Flags.SIM800_RESP_MQTT_CONNACK = 1;
                        }
                    }
                }
                else if (rx_chars[0] == 0x40)
                {
                    SIM800_UART_Get_Chars(rx_chars, 4, 0);
                    if (rx_chars[0] == 0x40 && rx_chars[1] == 0x02)
                    {
                        hSIM800.PUBACK.MSG_ID = rx_chars[2] << 8 | rx_chars[3];
                        hSIM800.RESP_Flags.SIM800_RESP_MQTT_PUBACK = 1;
                    }
                }
                else if (rx_chars[0] == 0x90)
                {
                    SIM800_UART_Get_Chars(rx_chars, 5, 0);
                    if (rx_chars[0] == 0x90 && rx_chars[1] == 0x03)
                    {
                        hSIM800.SUBACK.MSG_ID = rx_chars[2] << 8 | rx_chars[3];
                        hSIM800.SUBACK.QOS = rx_chars[4];
                        hSIM800.RESP_Flags.SIM800_RESP_MQTT_SUBACK = 1;
                    }
                }
                else if (rx_chars[0] == 0xD0)
                {
                    SIM800_UART_Get_Chars(rx_chars, 2, 0);
                    if (rx_chars[0] == 0xD0 && rx_chars[1] == 0x00)
                    {
                        hSIM800.RESP_Flags.SIM800_RESP_MQTT_PINGACK = 1;
                    }
                }
                else if (rx_chars[0] == '\r')
                {
                    SIM800_UART_Get_Chars(rx_chars, 10, 0);
                    if (strstr(rx_chars, "\r\nCLOSED\r\n") != NULL)
                    {
                        hSIM800.RESP_Flags.SIM800_RESP_CLOSED = 1;
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
                hSIM800.RESP_Flags.SIM800_RESP_OK = 1;
            }
            else if (strcmp(line, "Call Ready") == 00)
            {
                hSIM800.RESP_Flags.SIM800_RESP_CALL_READY = 1;
            }
            else if (strcmp(line, "SMS Ready") == 0)
            {
                hSIM800.RESP_Flags.SIM800_RESP_SMS_READY = 1;
            }
            else if (strcmp(line, "+CGATT: 1") == 0)
            {
                hSIM800.RESP_Flags.SIM800_RESP_GPRS_READY = 1;
            }
            else if (strcmp(line, "SHUT OK") == 0)
            {
                hSIM800.RESP_Flags.SIM800_RESP_SHUT_OK = 1;
            }
            else if (strcmp(line, "CONNECT") == 0)
            {
                hSIM800.RESP_Flags.SIM800_RESP_CONNECT = 1;
            }
            else if (CH_In_STR('.', line) == 3)
            {
                strncpy(hSIM800.TCP.MY_IP, line, sizeof(hSIM800.TCP.MY_IP));
                hSIM800.RESP_Flags.SIM800_RESP_IP = 1;
            }
            else if (strncmp(line, "+CCLK: ", 7) == 0)
            {
                hSIM800.Time.Year = (line[8] - '0') * 10 + line[9] - '0';
                hSIM800.Time.Month = (line[11] - '0') * 10 + line[12] - '0';
                hSIM800.Time.Date = (line[14] - '0') * 10 + line[15] - '0';

                hSIM800.Time.Hours = (line[17] - '0') * 10 + line[18] - '0';
                hSIM800.Time.Minutes = (line[20] - '0') * 10 + line[21] - '0';
                hSIM800.Time.Seconds = (line[23] - '0') * 10 + line[24] - '0';

                hSIM800.RESP_Flags.SIM800_RESP_DATE_TIME = 1;
            }
        }
    }
}

/************************* ISR ***************************/
/**
 * @brief this is sim800 state machine task, called when sim800 timer expires every 10ms(adjustable)
 *        called from @see HAL_TIM_PeriodElapsedCallback in stm32f4xx_it.c
 **/
void SIM800_TIM_ISR(void)
{
    if (hSIM800.Lock_TX)
    {
        return;
    }

    if (hSIM800.RX_Ready)
    {
        hSIM800.RX_Ready = 0;
        SIM800_RX_Process();
    }

    SIM800_Status_t sim800_result = SIM800_BUSY;

    switch (hSIM800.State)
    {
    case SIM800_IDLE:
        break;

    case SIM800_RESETING:
    {
        sim800_result = _SIM800_Reset();
        if (sim800_result == SIM800_SUCCESS)
        {
            hSIM800.State = SIM800_RESET_OK;
            APP_SIM800_Reset_CB(1);
        }
        else if (sim800_result == SIM800_FAILED)
        {
            // failed
            hSIM800.State = SIM800_IDLE;
            APP_SIM800_Reset_CB(0);
        }
    }
    break;

    case SIM800_RESET_OK:
        break;

    case SIM800_TCP_CONNECTING:
    {
        sim800_result = _SIM800_TCP_Connect();
        if (sim800_result == SIM800_SUCCESS)
        {
            hSIM800.State = SIM800_TCP_CONNECTED;
            APP_SIM800_TCP_CONN_CB(1);
        }
        else if (sim800_result == SIM800_FAILED)
        {
            // failed
            hSIM800.State = SIM800_RESET_OK;
            APP_SIM800_TCP_CONN_CB(0);
        }
    }
    break;

    case SIM800_TCP_CONNECTED:
        break;

    case SIM800_MQTT_CONNECTING:
    {
        sim800_result = _SIM800_MQTT_Connect();
        if (sim800_result == SIM800_FAILED)
        {
            hSIM800.State = SIM800_RESET_OK;
            APP_SIM800_MQTT_CONN_Failed_CB();
        }
        else if (sim800_result == SIM800_SUCCESS)
        {
            if (hSIM800.CONNACK.Code == 0x00) /** session present ignored */
            {
                hSIM800.State = SIM800_MQTT_CONNECTED;
            }
            else
            {
                hSIM800.State = SIM800_RESET_OK;
            }
            APP_SIM800_MQTT_CONNACK_CB(hSIM800.CONNACK.Code);
        }
    }
    break;

    case SIM800_MQTT_CONNECTED:
        if (hSIM800.PUBREC.PUBACK_Flag && !hSIM800.UART_TX_Busy)
        {
            /** send PUBACK */
            hSIM800.UART_TX_Busy = 1;
            hSIM800.PUBREC.PUBACK_Flag = 0;
            SIM800_UART_Send_Char(0x40); /** PUBACK header */
            SIM800_UART_Send_Char(0x02);
            SIM800_UART_Send_Char((hSIM800.PUBREC.MSG_ID >> 8) & 0xFF);
            SIM800_UART_Send_Char(hSIM800.PUBREC.MSG_ID & 0xFF);
            hSIM800.UART_TX_Busy = 0;
        }
        break;
    }

    /** look for callbacks */
    if (hSIM800.RESP_Flags.SIM800_RESP_MQTT_PUBREC)
    {
        hSIM800.RESP_Flags.SIM800_RESP_MQTT_PUBREC = 0;

        if (hSIM800.PUBREC.QOS)
        {
            hSIM800.PUBREC.PUBACK_Flag = 1; /** need to send PUBACK for this MSG */
        }

        APP_SIM800_MQTT_PUBREC_CB(hSIM800.PUBREC.Topic,
                                  hSIM800.PUBREC.MSG,
                                  hSIM800.PUBREC.MSG_Len,
                                  hSIM800.PUBREC.DUP,
                                  hSIM800.PUBREC.QOS,
                                  hSIM800.PUBREC.MSG_ID);
    }

    if (hSIM800.RESP_Flags.SIM800_RESP_MQTT_PUBACK)
    {
        hSIM800.RESP_Flags.SIM800_RESP_MQTT_PUBACK = 0;
        APP_SIM800_MQTT_PUBACK_CB(hSIM800.PUBACK.MSG_ID);
    }

    if (hSIM800.RESP_Flags.SIM800_RESP_MQTT_SUBACK)
    {
        hSIM800.RESP_Flags.SIM800_RESP_MQTT_SUBACK = 0;
        APP_SIM800_MQTT_SUBACK_CB(hSIM800.SUBACK.MSG_ID, hSIM800.SUBACK.QOS);
    }

    if (hSIM800.RESP_Flags.SIM800_RESP_MQTT_PINGACK)
    {
        hSIM800.RESP_Flags.SIM800_RESP_MQTT_PINGACK = 0;
        APP_SIM800_MQTT_Ping_CB();
    }

    if (hSIM800.RESP_Flags.SIM800_RESP_CLOSED)
    {
        /** TCP connection closed due to inactivity or server closed the connection */
        /** set hSIM800.State to SIM800_RESET_OK to indicate new tcp connection is required */
        hSIM800.State = SIM800_RESET_OK;
        APP_SIM800_TCP_Closed_CB();
    }

    if (hSIM800.RESP_Flags.SIM800_RESP_DATE_TIME)
    {
        /** handled in reset sequence */
    }

    if (hSIM800.RESP_Flags.SIM800_RESP_IP)
    {
        /** handled in TCP connect sequence */
    }
}

/**
  * @brief indicates some data is ready to process
  *        called from @see SIM800_UART_RX_ISR in sim800_uart.c
  */
void SIM800_RX_Ready_Callback(void)
{
    hSIM800.RX_Ready = 1;
}

/**
 * @brief called when sim800 modem is using uart dma mode, @see SIM800_UART_TX_CMPLT_ISR
 * @note only applicable if tx dma is used
 */
void SIM800_TX_Complete_Callback(void)
{
    if (hSIM800.UART_TX_Busy == 1)
    {
        hSIM800.UART_TX_Busy = 0;
    }
}

/****************************** WEAK callbacks need to be defined by user app **********************/
/**
 * @brief called when SIM800 reset sequence is complete
 *        callback response for @see SIM800_Reset
 * @param code reset success(GPRS is active) of failed
 */
__weak void APP_SIM800_Reset_CB(uint8_t reset_ok)
{
}

/**
 * @brief called when network time is received
 *        callback response for @see SIM800_Get_Time
 */
__weak void APP_SIM800_Date_Time_CB(struct SIM800_Date_Time_t *dt)
{
}

/**
 * @brief called when IP adrress is assigned
 */
__weak void APP_SIM800_IP_Address_CB(char *ip)
{
}

/**
 * @brief called when TCP connection is established
 *        callback response for @see SIM800_TCP_Connect
 * @param code TCP connection success of failed
 */
__weak void APP_SIM800_TCP_CONN_CB(uint8_t tcp_ok)
{
}
/**
 * @brief called when TCP connection is closed
 */
__weak void APP_SIM800_TCP_Closed_CB(void)
{
}

/**
 * @brief called when MQTT CONN failed
 *        callback response for @see SIM800_MQTT_Connect if nothing is received from broker
 */
__weak void APP_SIM800_MQTT_CONN_Failed_CB(void)
{
}

/**
 * @brief called when CONNACK is received
 *        callback response for @see SIM800_MQTT_Connect
 * @param code return code from broker
 */
__weak void APP_SIM800_MQTT_CONNACK_CB(uint16_t code)
{
}

/**
 * @brief called when PUBACK is received
 *        callback response for @see SIM800_MQTT_Publish
 * @param message_id message on which ack is received
 */
__weak void APP_SIM800_MQTT_PUBACK_CB(uint16_t message_id)
{
}

/**
 * @brief called when SUBACK is received
 *        callback response for @see SIM800_MQTT_Subscribe
 * @param packet_id packet on which ack is received
 * @param qos of topic
 */
__weak void APP_SIM800_MQTT_SUBACK_CB(uint16_t packet_id, uint8_t qos)
{
}

/**
 * @brief called when ping response is received
 *        callback response for @see SIM800_MQTT_Ping
 */
__weak void APP_SIM800_MQTT_Ping_CB(void)
{
}

/**
 * @brief called when mqtt message is received
 * @param topic topic on which message is received
 * @param message received message
 * @param msg_len message length
 * @param dup duplicates flag
 * @param qos qos of received message
 * @param message_id message id
 */
__weak void APP_SIM800_MQTT_PUBREC_CB(char *topic,
                                      char *message,
                                      uint32_t msg_len,
                                      uint8_t dup,
                                      uint8_t qos,
                                      uint16_t message_id)
{
}
