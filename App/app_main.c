#include "sim800_mqtt.h"
#include "stm32f4xx_hal.h"

uint32_t MQTT_Error_Count;

char packet[1024];

void App_Main(void)
{
	SIM800_Init();

	for(uint32_t i=0; i<1024; i++)
	{
		packet[i] = '0' + i%10;
	}

	if(SIM800_TCP_Connect("airtelgprs.com", "io.adafruit.com", 1883))
	{
		if(SIM800_MQTT_Connect("MQTT", 4, 0xC2, 64, "bhjsabdhf", "alsaad", "aio_uwus43tL6ELXTf4x0zm5YNphD5QN"))
		{
			for(uint32_t i=0; i<10; i++)
			{
				if(SIM800_MQTT_Publish("alsaad/feeds/Logger", packet, 1024, 0, 1, 0, i))
				{
					HAL_Delay(500);
				}
				else
				{
					break;
				}
			}
		}
	}
}

