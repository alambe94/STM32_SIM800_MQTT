#ifndef SIM800_MQTT_H_
#define SIM800_MQTT_H_

void SIM800_MQTT_Init(void);

uint8_t SIM800_MQTT_Connect(char *sim_apn,
                         char *broker,
                         uint16_t port,
                         char *protocol_name,
                         uint8_t protocol_version,
                         uint8_t flags,
                         uint16_t keep_alive,
                         char *my_id,
                         char *user_name,
                         char *password);

void SIM800_UART_Send_Bytes(char *data, uint32_t count);

void SIM800_UART_Send_String(char *str);

uint16_t SIM800_Get_Response(char *buff, uint16_t cnt, uint32_t timeout);

uint8_t SIM800_Check_Response(char *buff, uint32_t timeout);

uint8_t SIM800_MQTT_Publish(char *topic, char *payload, uint16_t payload_len);

uint8_t SIM800_MQTT_Subscribe(char *topic, uint8_t packet_id, uint8_t qos);

void SIM800_MQTT_Loop();


#endif /* SIM800_MQTT_H_ */
