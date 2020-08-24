#include "sim800_mqtt.h"
#include "stm32f4xx_hal.h"

uint32_t MQTT_Error_Count;

char packet[1000];

uint8_t RST_Flag = 0;
uint8_t TCP_Flag = 0;
uint8_t MQTT_Flag = 0;
uint16_t PUB_Message_ID = 0;

uint16_t SUB_Packet_ID = 0;
uint8_t SUB_QOS = 0;
uint8_t SUB_Flag = 0;

uint8_t Ping_Flag = 0;

void APP_SIM800_Reset_OK_CB(uint8_t reset_ok)
{
	RST_Flag = reset_ok;
}

void APP_SIM800_TCP_CONN_OK_CB(uint8_t tcp_ok)
{
	TCP_Flag = tcp_ok;
}

void APP_SIM800_MQTT_CONN_OK_CB(uint8_t mqtt_ok)
{
	MQTT_Flag = mqtt_ok;
}

void APP_SIM800_MQTT_PUBACK_CB(uint16_t message_id)
{
	PUB_Message_ID = message_id;
}

void APP_SIM800_MQTT_SUBACK_CB(uint16_t packet_id, uint8_t qos)
{
	SUB_Packet_ID = packet_id;
	SUB_QOS = qos;
	SUB_Flag = 1;
}

void APP_SIM800_MQTT_Ping_CB(void)
{
	Ping_Flag = 1;
}
void APP_SIM800_MQTT_MSG_CB(char *topic, char *message, uint32_t mesg_len, uint8_t dup, uint8_t qos, uint16_t message_id)
{
	dup++;
}

void App_Main(void)
{
	SIM800_Init();

	SIM800_Reset();
	while (!RST_Flag)
		;

	SIM800_TCP_Connect("airtelgprs.com", "io.adafruit.com", 1883);
	while (!TCP_Flag)
		;

	SIM800_MQTT_Connect("MQTT", 4, 0xC2, 64, "bhjsabdhf", "alsaad", "aio_uwus43tL6ELXTf4x0zm5YNphD5QN");
	while (!MQTT_Flag)
		;

	SIM800_MQTT_Subscribe("alsaad/feeds/Logger", 45, 1);
	while (!SUB_Flag)
		;

	for (uint32_t i = 0; i < 1024; i++)
	{
		packet[i] = '0' + i % 10;
	}

	for (uint32_t i = 0; i < 10; i++)
	{
		if (SIM800_MQTT_Publish("alsaad/feeds/Logger", packet, 1000, 0, 1, 0, i))
		{
			HAL_Delay(1000);
		}
		else
		{
			break;
		}
	}

	while (1)
		;
}
