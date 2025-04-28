#include "MODBUS_Server.h"
#include "stm32l4xx_hal.h"
#include "MODBUS_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "MODBUS_usart.h"
#include <stdio.h>


TaskHandle_t xModbusTaskHandle = NULL;

void ModbusTask(void *pvParameters);

void Modbus_Server_Init(void)
{
    // Initialize the MODBUS server
    // This function should set up the necessary configurations for the MODBUS server
    // such as initializing UART, setting up timers, etc.
    
    // Example: Initialize UART for MODBUS communication
    MX_LPUART1_UART_Init();
    

    // Set up any other necessary configurations
    // such as timers, GPIOs, etc.
    // Example: Initialize timers for MODBUS timing requirements
    // Create a FreeRTOS task to manage MODBUS incoming messages
    if( xTaskCreate(ModbusTask, "ModbusTask", 256, NULL, tskIDLE_PRIORITY + 1, &xModbusTaskHandle) != pdPASS )
    {
        while (1); // Handle task creation failure        
    }
}

// FreeRTOS task to handle MODBUS incoming messages
void ModbusTask(void *pvParameters)
{
    uint16_t Tx_Size = 0;
    
    (void)pvParameters; // Avoid unused parameter warning

    for (;;)
    {
        if(xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE)
        {
            // Process the received data in MODBUS_UART_Rx_Buffer
            // For example, you can print it to the console or send it back via UART
            MODBUS_UART_Rx_Buffer[MODBUS_UART_Rx_Count] = '\0'; // Null-terminate the received string
            Tx_Size = snprintf((char*)MODBUS_UART_Tx_Buffer, sizeof(MODBUS_UART_Tx_Buffer), "Received %d bytes: %s \n", MODBUS_UART_Rx_Count, MODBUS_UART_Rx_Buffer);
            HAL_UART_Transmit_IT(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size);
            
            HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, MODBUS_UART_Rx_Buffer, MODBUS_UART_BUFFER_SIZE);            // Reset the receive count            
        }
        // Add a small delay to prevent the task from consuming too much CPU time
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 ms                                                                                             
    }
}
