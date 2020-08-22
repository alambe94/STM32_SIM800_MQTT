#include "sim800_mqtt.h"

enum MQTT_State_t
{
	IDLE,
	CONNECTING,
	CONNECTED,
	PUBLISHING,
	PUBLISHED,
	ERROR

}MQTT_State = IDLE;

uint32_t MQTT_Error_Count;

void App_Main(void)
{
	SIM800_Init();

	SIM800_TCP_Connect("airtelgprs.com", "io.adafruit.com", 1883);


	SIM800_MQTT_Publish("alsaad/feeds/Logger", "123", 3, 0, 1, 0, 1);
}

