#include "sim800_mqtt.h"
#include "sim800_uart.h"
#include "stm32f4xx_hal.h"

uint32_t MQTT_Error_Count;

char Packet[1024];

uint8_t RST_Flag = 0;
uint8_t TCP_Flag = 0;
uint8_t MQTT_Flag = 0;
uint16_t PUB_Message_ID = 0;

uint16_t SUB_Packet_ID = 0;
uint8_t SUB_QOS = 0;
uint8_t SUB_Flag = 0;

uint16_t MSG_Received_Count = 0;

uint8_t Ping_Flag = 0;

void App_Main(void)
{
	SIM800_Init();

	for(uint16_t i=0; i<sizeof(Packet); i++)
	{
		Packet[i] = i%10 + 48;
	}

	Packet[sizeof(Packet)-1] = '\n';

	while (1)
	{
		if (SIM800_Get_State() == SIM800_IDLE)
		{
			SIM800_Reset();
		}

		if (SIM800_Get_State() == SIM800_RESET_OK)
		{
			//SIM800_TCP_Connect("airtelgprs.com", "jarsservices.info", 1883);
		}

		if (SIM800_Get_State() == SIM800_TCP_CONNECTED)
		{
			CONN_Flag_t flags = {.C_Flags = 0xC2};
			flags.Bits.Password = 0;
			flags.Bits.User_Name = 0;
			SIM800_MQTT_Connect("MQTT", 4, flags, 64, "sfsgfsg", "alsaad", "aio_uwus43tL6ELXTf4x0zm5YNphD5QN");
		}

		if (SIM800_Is_MQTT_Connected())
		{
			for (uint32_t i = 0; i < 10; i++)
			{
				if (SIM800_MQTT_Publish("alsaad/feeds/abcd", Packet, sizeof(Packet), 0, 1, 0, i))
				{
					HAL_Delay(1000);
				}

				if (SIM800_MQTT_Ping())
				{
					HAL_Delay(1000);
				}
			}

			while(1);
		}

	}
}

void APP_SIM800_Reset_CB(uint8_t reset_ok)
{
	RST_Flag = reset_ok;
}

void APP_SIM800_Date_Time_CB(struct SIM800_Date_Time_t *dt)
{
	dt->Year;
	dt->Minutes;
}

void APP_SIM800_TCP_CONN_CB(uint8_t tcp_ok)
{
	TCP_Flag = tcp_ok;
}

void APP_SIM800_MQTT_CONN_CB(uint8_t mqtt_ok)
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
	Ping_Flag++;
}

void APP_SIM800_MQTT_MSG_CB(char *topic,
							char *message,
							uint32_t mesg_len,
							uint8_t dup,
							uint8_t qos,
							uint16_t message_id)
{
	MSG_Received_Count++;
}
