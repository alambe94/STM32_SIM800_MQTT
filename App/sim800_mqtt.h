#ifndef SIM800_MQTT_H_
#define SIM800_MQTT_H_

uint8_t SIM800_Init(void);

uint8_t SIM800_MQTT_Connect(char *sim_apn,
                         char *broker,
                         uint32_t port,
                         char *protocol_name,
                         uint8_t protocol_version,
                         uint8_t flags,
                         uint32_t keep_alive,
                         char *my_id,
                         char *user_name,
                         char *password);

uint32_t SIM800_Get_Response(char *buff, uint32_t timeout);

uint8_t SIM800_Check_Response(char *buff, uint32_t timeout);

uint8_t SIM800_MQTT_Publish(char *topic, char *payload, uint32_t payload_len);

uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos);

void SIM800_MQTT_Loop();


#endif /* SIM800_MQTT_H_ */
