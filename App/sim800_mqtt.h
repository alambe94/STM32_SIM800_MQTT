#ifndef SIM800_MQTT_H_
#define SIM800_MQTT_H_

#include <stdint.h>

enum SIM800_State_t
{
    SIM800_IDLE,
    SIM800_RESETING,
    SIM800_RESET_OK,
    SIM800_TCP_CONNECTING,
    SIM800_TCP_CONNECTED, /** after this modem is in transparent mode */

    SIM800_MQTT_RECEIVING,
    SIM800_MQTT_TRANSMITTING, /** indicates uart tx is busy */
};

void SIM800_Init(void);

uint8_t SIM800_Reset(void);

void SIM800_RX_Task_Trigger(void);

uint8_t SIM800_MQTT_Disconnect(void);

uint8_t SIM800_Is_MQTT_Connected();

enum SIM800_State_t SIM800_Get_State(void);

uint8_t SIM800_MQTT_Ping(void);

uint8_t SIM800_TCP_Connect(char *sim_apn, char *broker, uint16_t port);

uint8_t SIM800_MQTT_Connect(char *protocol_name,
                            uint8_t protocol_version,
                            uint8_t flags,
                            uint16_t keep_alive,
                            char *my_id,
                            char *user_name,
                            char *password);

uint32_t SIM800_Get_Response(char *buff, uint32_t buff_size, uint32_t timeout);

uint8_t SIM800_Check_Response(char *buff, uint32_t timeout);

uint8_t SIM800_MQTT_Publish(char *topic, char *message, uint32_t message_len, uint8_t dup, uint8_t qos, uint8_t retain, uint16_t message_id);

uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos);

/** WAEK callbacks need to define by user app ****/
void APP_SIM800_MQTT_CONN_CB(uint8_t mqtt_ok);
void APP_SIM800_TCP_CONN_CB(uint8_t tcp_ok);
void APP_SIM800_Reset_CB(uint8_t reset_ok);
void APP_SIM800_MQTT_PUBACK_CB(uint16_t message_id);
void APP_SIM800_MQTT_SUBACK_CB(uint16_t packet_id, uint8_t qos);
void APP_SIM800_MQTT_Ping_CB(void);
void APP_SIM800_MQTT_MSG_CB(char *topic,
                            char *message,
                            uint32_t mesg_len,
                            uint8_t dup,
                            uint8_t qos,
                            uint16_t message_id);

#endif /* SIM800_MQTT_H_ */
