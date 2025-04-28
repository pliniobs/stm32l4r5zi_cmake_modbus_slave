
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__


/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define MODBUS_UART_BAUDRATE 115200
#define MODBUS_UART_BUFFER_SIZE 100


extern UART_HandleTypeDef hlpuart1;

extern uint16_t MODBUS_UART_Rx_Count; // Number of bytes received
extern uint8_t MODBUS_UART_Tx_Buffer[MODBUS_UART_BUFFER_SIZE]; // Buffer for sending data
extern uint8_t MODBUS_UART_Rx_Buffer[MODBUS_UART_BUFFER_SIZE]; // Buffer for receiving data

void MX_LPUART1_UART_Init(void);
void MX_LPUART1_UART_DeInit(void);


#endif /* __USART_H__ */

