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
	MQTTPacket_connectData param = MQTTPacket_connectData_initializer;

	SIM800_Init();

	SIM800_TCP_Connect("airtelgprs.com", "io.adafrute.com", 1883);

	param.MQTTVersion = 4; // 3.1.1
	param.cleansession = 1;
	param.clientID.cstring = "abcdef";
	param.keepAliveInterval = 60;
	param.password.cstring ="1234568";
	param.username.cstring = "alsaad";

	SIM800_MQTT_Connect(&param);
}

