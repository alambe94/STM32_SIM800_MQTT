#ifndef SIM800_UART_H_
#define SIM800_UART_H_

/** standard includes */
#include <stdint.h>

void SIM800_UART_Init();
void SIM800_UART_Send_Char(char data);
void SIM800_UART_Send_Bytes(char *data, uint32_t count);
void SIM800_UART_Send_Bytes_DMA(char *data, uint32_t count);
void SIM800_UART_Send_String(char *str);
void SIM800_UART_Printf(const char *fmt, ...);
void SIM800_UART_Flush_RX();

uint32_t SIM800_UART_Get_Count(void);
int SIM800_UART_Get_Char(uint32_t timeout);
uint32_t SIM800_UART_Get_Chars(char *buffer, uint32_t count, uint32_t timeout);
uint32_t SIM800_UART_Get_Line(char *buffer, uint32_t buff_size, uint32_t timeout);

#endif /* SIM800_UART_H_ */
