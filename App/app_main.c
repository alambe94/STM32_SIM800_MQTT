#include "sim800_mqtt.h"
#include "stm32f4xx_hal.h"

uint32_t MQTT_Error_Count;

char packet[1404];

void SIM800_Reset_OK_CB()
{
}

void SIM800_TCP_CONN_OK_CB()
{
}

void SIM800_MQTT_CONN_OK_CB()
{
}

void App_Main(void)
{
	SIM800_Init();

	SIM800_Reset();

	SIM800_TCP_Connect("airtelgprs.com", "io.adafruit.com", 1883);

	SIM800_MQTT_Connect("MQTT", 4, 0xC2, 64, "bhjsabdhf", "alsaad", "aio_uwus43tL6ELXTf4x0zm5YNphD5QN");

	for (uint32_t i = 0; i < 1024; i++)
	{
		packet[i] = '0' + i % 10;
	}

	for (uint32_t i = 0; i < 10; i++)
	{
		if (SIM800_MQTT_Publish("alsaad/feeds/Logger", packet, 1404, 0, 1, 0, i))
		{
			HAL_Delay(500);
		}
		else
		{
			break;
		}
	}
}
