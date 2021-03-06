#ifndef SIM800_MQTT_H_
#define SIM800_MQTT_H_

#include <stdint.h>

/**
 * mqtt connect flags
 */
typedef union CONN_Flag_t
{
    uint8_t C_Flags;
    struct
    {
        uint8_t Reserved : 1;
        uint8_t Clean_Session : 1;
        uint8_t Will_Flag : 1;
        uint8_t Will_QOS : 2;
        uint8_t Will_Retain : 1;
        uint8_t Password : 1;
        uint8_t User_Name : 1;
    } Bits;
} CONN_Flag_t;

typedef enum SIM800_State_t
{
    SIM800_IDLE,
    SIM800_RESETING,
    SIM800_RESET_OK,
    SIM800_TCP_CONNECTING,
    SIM800_TCP_CONNECTED, /** after this modem is in transparent mode */

    SIM800_MQTT_CONNECTING,
    SIM800_MQTT_CONNECTED,
} SIM800_State_t;

typedef struct SIM800_Date_Time_t
{
    uint8_t Year;
    uint8_t Month;
    uint8_t Date;

    uint8_t Hours;
    uint8_t Minutes;
    uint8_t Seconds;
} SIM800_Date_Time_t;

void SIM800_Init(void);

uint8_t SIM800_Reset(void);

uint8_t SIM800_Get_Time(void);

uint8_t SIM800_MQTT_Disconnect(void);

uint8_t SIM800_Is_MQTT_Connected();

SIM800_State_t SIM800_Get_State(void);

uint8_t SIM800_MQTT_Ping(void);

uint8_t SIM800_TCP_Connect(char *sim_apn, char *broker, uint16_t port);

uint8_t SIM800_MQTT_Connect(char *protocol_name,
                            uint8_t protocol_version,
                            CONN_Flag_t flags,
                            uint16_t keep_alive,
                            char *my_id,
                            char *user_name,
                            char *password);

uint32_t SIM800_Get_Response(char *buff, uint32_t buff_size, uint32_t timeout);

uint8_t SIM800_Check_Response(char *buff, uint32_t timeout);

uint8_t SIM800_MQTT_Publish(char *topic,
                            char *message,
                            uint32_t message_len,
                            uint8_t dup,
                            uint8_t qos,
                            uint8_t retain,
                            uint16_t message_id);

uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos);

/** WAEK callbacks need to br defined by user app ****/
void APP_SIM800_Reset_CB(uint8_t reset_ok);
void APP_SIM800_Date_Time_CB(struct SIM800_Date_Time_t *dt);
void APP_SIM800_IP_Address_CB(char *ip);
void APP_SIM800_TCP_CONN_CB(uint8_t tcp_ok);
void APP_SIM800_TCP_Closed_CB(void);
void APP_SIM800_MQTT_CONN_Failed_CB(void);
void APP_SIM800_MQTT_CONNACK_CB(uint16_t mqtt_ok);
void APP_SIM800_MQTT_PUBACK_CB(uint16_t message_id);
void APP_SIM800_MQTT_SUBACK_CB(uint16_t packet_id, uint8_t qos);
void APP_SIM800_MQTT_Ping_CB(void);
void APP_SIM800_MQTT_PUBREC_CB(char *topic,
                               char *message,
                               uint32_t mesg_len,
                               uint8_t dup,
                               uint8_t qos,
                               uint16_t message_id);

#endif /* SIM800_MQTT_H_ */
