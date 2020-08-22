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

	if(SIM800_TCP_Connect("airtelgprs.com", "io.adafruit.com", 1883))
	{
		if(SIM800_MQTT_Connect("MQTT", 4, 0xC2, 64, "bhjsabdhf", "alsaad", "aio_uwus43tL6ELXTf4x0zm5YNphD5QN"))
		{
			for(uint32_t i=0; i<10; i++)
			{
				if(SIM800_MQTT_Publish("alsaad/feeds/Logger", "123", 3, 0, 1, 0, i))
				{
				}
			}
		}
	}
}

