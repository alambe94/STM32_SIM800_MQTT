#ifndef SIM800_MQTT_H_
#define SIM800_MQTT_H_

#include <stdint.h>

void SIM800_Init(void);

uint8_t SIM800_Reset(void);

void SIM800_RX_Task_Trigger(void);

void SIM800_MQTT_Disconnect(void);

uint8_t SIM800_Is_Connected();

uint8_t SIM800_MQTT_Ping(void);

uint8_t SIM800_TCP_Connect(char *sim_apn, char *broker, uint16_t port);

uint8_t SIM800_MQTT_Connect(char *protocol_name,
                            uint8_t protocol_version,
                            uint8_t flags,
                            uint16_t keep_alive,
                            char *my_id,
                            char *user_name,
                            char *password);

uint32_t SIM800_Get_Response(char *buff, uint32_t timeout);

uint8_t SIM800_Check_Response(char *buff, uint32_t timeout);

uint8_t SIM800_MQTT_Publish(char *topic, char *mesaage, uint32_t mesaage_len, uint8_t dup, uint8_t qos, uint8_t retain, uint16_t mesaage_id);

uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos);

void SIM800_MQTT_Loop();


#endif /* SIM800_MQTT_H_ */
