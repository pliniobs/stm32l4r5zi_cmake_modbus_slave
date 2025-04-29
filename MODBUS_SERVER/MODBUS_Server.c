#include "MODBUS_Server.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "MODBUS_usart.h"
#include "queue.h"
#include "stm32l4xx_hal.h"
#include "task.h"

TaskHandle_t xModbusTaskHandle = NULL;
uint16_t MODBUS_Server_Holding_Registers[MODBUS_SERVER_HH_SIZE] = {0};  // Example holding registers
uint16_t MODBUS_Server_Input_Registers[MODBUS_SERVER_IR_SIZE] = {0};    // Example Input registers
uint8_t MODBUS_Server_Discrete_Input[MODBUS_SERVER_DI_SIZE] = {0};      // Example Discrete inputs
uint8_t MODBUS_Server_Discrete_Output[MODBUS_SERVER_DO_SIZE] = {0};     // Example Discrete outputs

void ModbusTask(void *pvParameters);

/**
 * The Modbus_RTU_CRC16 function calculates a CRC-16 checksum for a given data array using a predefined
 * lookup table.
 *
 * @param nData The `nData` parameter is a pointer to an array of uint8_t data that you want to
 * calculate the CRC16 checksum for.
 * @param wLength The `wLength` parameter in the `Modbus_RTU_CRC16` function represents the length of
 * the data array `nData` that you want to calculate the CRC for. It indicates the number of bytes in
 * the data array that should be considered for calculating the CRC checksum.
 *
 * @return The function `Modbus_RTU_CRC16` returns a `uint16_t` value, which is the calculated CRC-16
 * checksum for the given data.
 */
uint16_t Modbus_RTU_CRC16(uint8_t *nData, uint16_t wLength);

// FreeRTOS task to handle MODBUS incoming messages
void ModbusTask(void *pvParameters) {
  uint16_t Tx_Size = 0;
  uint16_t Calculated_CRC16 = 0;
  uint8_t function_code;
  uint16_t start_address;
  uint16_t data_length;
  uint8_t device_address;
  (void)pvParameters;  // Avoid unused parameter warning

  for (;;) {
    if (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE) {
        if (MODBUS_UART_Rx_Count >= MODBUS_SERVER_MINIMUM_INPUT_FRAME_SIZE) {
            // Calculate the CRC16 for the received data
            // Incommin                                                                                                                                                                                 g_CRC16 = (MODBUS_UART_Rx_Buffer[MODBUS_UART_Rx_Count - 2] << 8) | MODBUS_UART_Rx_Buffer[MODBUS_UART_Rx_Count - 1];
            Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Rx_Buffer, MODBUS_UART_Rx_Count);

            if (!Calculated_CRC16) {
            // Valid MODBUS frame received
            // Process the MODBUS request here
            // Step 1: Extract the function code, Start Address, and Data Length
            device_address = MODBUS_UART_Rx_Buffer[0];  // Device address
            // Check if the device address matches the server address
            if (device_address == MODBUS_FUNCTION_SERVER_ADDRESS) {
                
                function_code = MODBUS_UART_Rx_Buffer[1];
                start_address = (MODBUS_UART_Rx_Buffer[2] << 8) | MODBUS_UART_Rx_Buffer[3];
                data_length = (MODBUS_UART_Rx_Buffer[4] << 8) | MODBUS_UART_Rx_Buffer[5];

                // Step 2: Handle the MODBUS request based on the function code
                switch (function_code) {
                    case MODBUS_FUNCTION_READ_COILS:
                        // Handle read coils request
                        if((start_address >= MODBUS_SERVER_DO_START_ADDRESS) && ((start_address + data_length) <= MODBUS_SERVER_DO_END_ADDRESS)) {
                            // Start to assemble the response frame
                            uint16_t coil_index = start_address - MODBUS_SERVER_DO_START_ADDRESS;
                            
                            // Calculate the number of bytes required to represent the coils
                            uint16_t byte_count = (data_length + 7) / 8;

                            Tx_Size = 0;
                            // Prepare the response frame
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = device_address;  // Device address
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = function_code;  // Function code
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = byte_count;  // Byte count
                            // Place the requested coil values in the response frame
                            for (uint16_t i = 0; i < byte_count; i++) {
                                uint8_t coil_byte = 0;
                                for (uint8_t bit = 0; bit < 8; bit++) {
                                    uint16_t coil_position = coil_index + (i * 8) + bit;
                                    if (coil_position < (start_address + data_length)) {
                                        if (MODBUS_Server_Discrete_Input[coil_position]) {
                                            coil_byte |= (1 << bit);
                                        }
                                    }
                                }
                                MODBUS_UART_Tx_Buffer[Tx_Size++] = coil_byte;
                            }

                            // Calculate the CRC16 for the response frame and place it at the end of the frame
                            Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Tx_Buffer, Tx_Size);
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = Calculated_CRC16 & 0xFF;  // Low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (Calculated_CRC16 >> 8) & 0xFF;  // High byte

                            // Send the response frame
                            HAL_UART_Transmit(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size, 100);
                        }
                    break;
                    case MODBUS_FUNCTION_READ_DISCRETE_INPUTS:
                        // Handle read discrete inputs request
                        if((start_address >= MODBUS_SERVER_DI_START_ADDRESS) && ((start_address + data_length) <= MODBUS_SERVER_DI_END_ADDRESS)) {
                            // Start to assemble the response frame
                            uint16_t coil_index = start_address - MODBUS_SERVER_DI_START_ADDRESS;
                            
                            // Calculate the number of bytes required to represent the coils
                            uint16_t byte_count = (data_length + 7) / 8;

                            Tx_Size = 0;
                            // Prepare the response frame
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = device_address;  // Device address
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = function_code;  // Function code
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = byte_count;  // Byte count
                            // Place the requested coil values in the response frame
                            for (uint16_t i = 0; i < byte_count; i++) {
                                uint8_t coil_byte = 0;
                                for (uint8_t bit = 0; bit < 8; bit++) {
                                    uint16_t coil_position = coil_index + (i * 8) + bit;
                                    if (coil_position < (start_address + data_length)) {
                                        if (MODBUS_Server_Discrete_Output[coil_position]) {
                                            coil_byte |= (1 << bit);
                                        }
                                    }
                                }
                                MODBUS_UART_Tx_Buffer[Tx_Size++] = coil_byte;
                            }

                            // Calculate the CRC16 for the response frame and place it at the end of the frame
                            Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Tx_Buffer, Tx_Size);
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = Calculated_CRC16 & 0xFF;  // Low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (Calculated_CRC16 >> 8) & 0xFF;  // High byte

                            // Send the response frame
                            HAL_UART_Transmit(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size, 100);
                        }
                    break;
                    
                    case MODBUS_FUNCTION_READ_HOLDING_REGISTERS:
                        // Check the address range for holding registers
                        if((start_address >= MODBUS_SERVER_HH_START_ADDRESS) && ((start_address + data_length) <= MODBUS_SERVER_HH_END_ADDRESS)) {
                            // Start to assembly the response frame
                            // Read holding registers
                            uint16_t register_index = start_address - MODBUS_SERVER_HH_START_ADDRESS;
                            Tx_Size = 0;
                            // Prepare the response frame
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = device_address;  // Device address
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = function_code;  // Function code
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = data_length * 2;  // Byte count
                            // Place the requested register values in the response frame
                            for(uint16_t i = register_index; i < (register_index + data_length); i++) {
                                // Read the holding register value
                                uint16_t register_value = MODBUS_Server_Holding_Registers[i];
                                MODBUS_UART_Tx_Buffer[Tx_Size++] = (register_value >> 8) & 0xFF;  // High byte
                                MODBUS_UART_Tx_Buffer[Tx_Size++] = register_value & 0xFF;  // Low byte
                            }
                            // Calculate the CRC16 for the response frame and place on the end of the frame
                            Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Tx_Buffer, Tx_Size);

                            MODBUS_UART_Tx_Buffer[Tx_Size++] = Calculated_CRC16 & 0xFF;  // Low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (Calculated_CRC16 >> 8) & 0xFF;  // High byte
                            
                            // Send the response frame
                            HAL_UART_Transmit(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size, 100);
                            
                        }
                    break;

                    case MODBUS_FUNCTION_READ_INPUT_REGISTERS:
                    // Handle read input registers request
                    if((start_address >= MODBUS_SERVER_IR_START_ADDRESS) && ((start_address + data_length) < MODBUS_SERVER_IR_END_ADDRESS)) {
                        // Start to assembly the response frame
                        // Read holding registers
                        uint16_t register_index = start_address - MODBUS_SERVER_IR_START_ADDRESS;
                        Tx_Size = 0;
                        // Prepare the response frame
                        MODBUS_UART_Tx_Buffer[Tx_Size++] = device_address;  // Device address
                        MODBUS_UART_Tx_Buffer[Tx_Size++] = function_code;  // Function code
                        MODBUS_UART_Tx_Buffer[Tx_Size++] = data_length * 2;  // Byte count
                        // Place the requested register values in the response frame
                        for(uint16_t i = register_index; i < (register_index + data_length); i++) {
                            // Read the holding register value
                            uint16_t register_value = MODBUS_Server_Input_Registers[i];
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (register_value >> 8) & 0xFF;  // High byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = register_value & 0xFF;  // Low byte
                        }
                        // Calculate the CRC16 for the response frame and place on the end of the frame
                        Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Tx_Buffer, Tx_Size);

                        MODBUS_UART_Tx_Buffer[Tx_Size++] = Calculated_CRC16 & 0xFF;  // Low byte
                        MODBUS_UART_Tx_Buffer[Tx_Size++] = (Calculated_CRC16 >> 8) & 0xFF;  // High byte
                        
                        // Send the response frame
                        HAL_UART_Transmit(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size, 100);
                        
                    }
                    break;
                    case MODBUS_FUNCTION_WRITE_SINGLE_COIL:
                        // Handle write single coil request
                        if((start_address >= MODBUS_SERVER_DO_START_ADDRESS) && (start_address < MODBUS_SERVER_DO_END_ADDRESS)) {
                            // Start to assembly the response frame
                            // Read holding registers
                            uint16_t register_index = start_address - MODBUS_SERVER_DO_START_ADDRESS;

                            MODBUS_Server_Discrete_Output[register_index] = (data_length >> 8) & 0xFF; // Write the value to the holding register
                            
                            Tx_Size = 0;
                            // Prepare the response frame
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = device_address;  // Device address
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = function_code;   // Function code
                            MODBUS_UART_Tx_Buffer[Tx_Size++] =  (start_address >> 8) & 0xFF; // Address HI
                            MODBUS_UART_Tx_Buffer[Tx_Size++] =  start_address & 0xFF; // Address LO
                            
                            if( data_length > 0x00) {
                                MODBUS_UART_Tx_Buffer[Tx_Size++] =  0xFF; // Data Length HI
                                MODBUS_UART_Tx_Buffer[Tx_Size++] =  0x00; // Data Length LO   
                            }else{
                                MODBUS_UART_Tx_Buffer[Tx_Size++] = 0x00; // Data Length HI
                                MODBUS_UART_Tx_Buffer[Tx_Size++] = 0x00; // Data Length LO
                            }

                            // Calculate the CRC16 for the response frame and place on the end of the frame
                            Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Tx_Buffer, Tx_Size);

                            MODBUS_UART_Tx_Buffer[Tx_Size++] = Calculated_CRC16 & 0xFF;  // Low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (Calculated_CRC16 >> 8) & 0xFF;  // High byte
                            
                            // Send the response frame
                            HAL_UART_Transmit(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size, 100);
                            
                        }
                    break;
                    case MODBUS_FUNCTION_WRITE_SINGLE_REGISTER:
                        // Handle write single register request
                        if((start_address >= MODBUS_SERVER_HH_START_ADDRESS) && (start_address < MODBUS_SERVER_HH_END_ADDRESS)) {
                            // Start to assembly the response frame
                            // Read holding registers
                            uint16_t register_index = start_address - MODBUS_SERVER_HH_START_ADDRESS;

                            MODBUS_Server_Holding_Registers[register_index] = data_length; // Write the value to the holding register
                            
                            Tx_Size = 0;
                            // Prepare the response frame
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = device_address;  // Device address
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = function_code;   // Function code
                            MODBUS_UART_Tx_Buffer[Tx_Size++] =  (start_address >> 8) & 0xFF; // Address HI
                            MODBUS_UART_Tx_Buffer[Tx_Size++] =  start_address & 0xFF; // Address LO
                            MODBUS_UART_Tx_Buffer[Tx_Size++] =  (data_length >> 8) & 0xFF; // Data Length HI
                            MODBUS_UART_Tx_Buffer[Tx_Size++] =  data_length & 0xFF; // Data Length LO

                            // Calculate the CRC16 for the response frame and place on the end of the frame
                            Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Tx_Buffer, Tx_Size);

                            MODBUS_UART_Tx_Buffer[Tx_Size++] = Calculated_CRC16 & 0xFF;  // Low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (Calculated_CRC16 >> 8) & 0xFF;  // High byte
                            
                            // Send the response frame
                            HAL_UART_Transmit(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size, 100);
                            
                        }
                    break;
                    case MODBUS_FUNCTION_WRITE_MULTIPLE_COILS:
                        // Handle write multiple coils request
                        if((start_address >= MODBUS_SERVER_DO_START_ADDRESS) && ((start_address + data_length) < MODBUS_SERVER_DO_END_ADDRESS)){
                            // Start to assembly the response frame
                            // Read holding registers
                            uint16_t register_index = start_address - MODBUS_SERVER_DO_START_ADDRESS;
                            
                            uint16_t byte_counter = 8; // Start from the 8th byte of the received frame
                            uint16_t coil_value;
                            // Place the requested coil values in the response frame
                            for(uint16_t i = register_index; i < (register_index + data_length); i++) {
                                // Read the holding register value
                                coil_value = 0;
                                coil_value = (MODBUS_UART_Rx_Buffer[byte_counter++] << 8); // High byte of register
                                coil_value |= MODBUS_UART_Rx_Buffer[byte_counter++];       // Low byte of register

                                MODBUS_Server_Discrete_Output[i] = coil_value; // Write the value to the holding register                    
                            }
                            
                            Tx_Size = 0;
                            // Prepare the response frame
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = device_address;  // Device address
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = function_code;  // Function code
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (start_address >> 8) & 0xff;  // Start address high byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = start_address & 0xff;  // Start address low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (data_length >> 8) & 0xff;  // Data length high byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = data_length & 0xff;  // Data length low byte
                            // Calculate the CRC16 for the response frame and place on the end of the frame
                            Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Tx_Buffer, Tx_Size);
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = Calculated_CRC16 & 0xFF;  // Low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (Calculated_CRC16 >> 8) & 0xFF;  // High byte
                            
                            // Send the response frame
                            HAL_UART_Transmit(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size, 100);
                        }
                    break;
                    case MODBUS_FUNCTION_WRITE_MULTIPLE_REGISTERS:
                        // Handle write multiple registers request
                        if((start_address >= MODBUS_SERVER_HH_START_ADDRESS) && ((start_address + data_length) < MODBUS_SERVER_HH_END_ADDRESS)){
                            // Start to assembly the response frame
                            // Read holding registers
                            uint16_t register_index = start_address - MODBUS_SERVER_HH_START_ADDRESS;
                            
                            uint16_t byte_counter = 8; // Start from the 8th byte of the received frame
                            uint16_t register_value;
                            // Place the requested register values in the response frame
                            for(uint16_t i = register_index; i < (register_index + data_length); i++) {
                                // Read the holding register value
                                register_value = 0;
                                register_value = (MODBUS_UART_Rx_Buffer[byte_counter++] << 8); // High byte of register
                                register_value |= MODBUS_UART_Rx_Buffer[byte_counter++];       // Low byte of register

                                MODBUS_Server_Input_Registers[i] = register_value; // Write the value to the holding register                    
                            }
                            
                            Tx_Size = 0;
                            // Prepare the response frame
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = device_address;  // Device address
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = function_code;  // Function code
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (start_address >> 8) & 0xff;  // Start address high byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = start_address & 0xff;  // Start address low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (data_length >> 8) & 0xff;  // Data length high byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = data_length & 0xff;  // Data length low byte
                            // Calculate the CRC16 for the response frame and place on the end of the frame
                            Calculated_CRC16 = Modbus_RTU_CRC16(MODBUS_UART_Tx_Buffer, Tx_Size);
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = Calculated_CRC16 & 0xFF;  // Low byte
                            MODBUS_UART_Tx_Buffer[Tx_Size++] = (Calculated_CRC16 >> 8) & 0xFF;  // High byte
                            
                            // Send the response frame
                            HAL_UART_Transmit(&hlpuart1, MODBUS_UART_Tx_Buffer, Tx_Size, 100);   
                        }
                    break;
                    case MODBUS_FUNCTION_READ_EXCEPTION_STATUS:
                        // Handle read exception status request
                    break;
                }
            // Step 3: Prepare the response frame}
            } 
        }
    }

      if (HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, MODBUS_UART_Rx_Buffer, MODBUS_UART_BUFFER_SIZE) != HAL_OK)  // Start receiving data in interrupt mode
      {
        while (1);  // Handle error if needed
      }
    }
  }
}

void Modbus_Server_Init(void) {
  // Initialize the MODBUS server
  // This function should set up the necessary configurations for the MODBUS server
  // such as initializing UART, setting up timers, etc.

  // Example: Initialize UART for MODBUS communication
  MX_LPUART1_UART_Init();

  // Set up any other necessary configurations
  // such as timers, GPIOs, etc.
  // Example: Initialize timers for MODBUS timing requirements
  // Create a FreeRTOS task to manage MODBUS incoming messages
  if (xTaskCreate(ModbusTask, "ModbusTask", 256, NULL, tskIDLE_PRIORITY + 1, &xModbusTaskHandle) != pdPASS) {
    while (1);  // Handle task creation failure
  }
}

/**
 * The Modbus_RTU_CRC16 function calculates a CRC-16 checksum for a given data array using a predefined
 * lookup table.
 *
 * @param nData The `nData` parameter is a pointer to an array of uint8_t data that you want to
 * calculate the CRC16 checksum for.
 * @param wLength The `wLength` parameter in the `Modbus_RTU_CRC16` function represents the length of
 * the data array `nData` that you want to calculate the CRC for. It indicates the number of bytes in
 * the data array that should be considered for calculating the CRC checksum.
 *
 * @return The function `Modbus_RTU_CRC16` returns a `uint16_t` value, which is the calculated CRC-16
 * checksum for the given data.
 */
uint16_t Modbus_RTU_CRC16(uint8_t *nData, uint16_t wLength) {
  static const uint16_t wCRCTable[] = {0X0000, 0XC0C1, 0XC181, 0X0140,
                                       0XC301, 0X03C0, 0X0280, 0XC241, 0XC601, 0X06C0, 0X0780, 0XC741,
                                       0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0, 0X0D80, 0XCD41,
                                       0X0F00, 0XCFC1, 0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81, 0X0B40,
                                       0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 0X18C0, 0X1980, 0XD941,
                                       0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40,
                                       0XDD01, 0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540,
                                       0XD701, 0X17C0, 0X1680, 0XD641, 0XD201, 0X12C0, 0X1380, 0XD341,
                                       0X1100, 0XD1C1, 0XD081, 0X1040, 0XF001, 0X30C0, 0X3180, 0XF141,
                                       0X3300, 0XF3C1, 0XF281, 0X3240, 0X3600, 0XF6C1, 0XF781, 0X3740,
                                       0XF501, 0X35C0, 0X3480, 0XF441, 0X3C00, 0XFCC1, 0XFD81, 0X3D40,
                                       0XFF01, 0X3FC0, 0X3E80, 0XFE41, 0XFA01, 0X3AC0, 0X3B80, 0XFB41,
                                       0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981, 0X2940,
                                       0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41,
                                       0X2D00, 0XEDC1, 0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541,
                                       0X2700, 0XE7C1, 0XE681, 0X2640, 0X2200, 0XE2C1, 0XE381, 0X2340,
                                       0XE101, 0X21C0, 0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141,
                                       0X6300, 0XA3C1, 0XA281, 0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740,
                                       0XA501, 0X65C0, 0X6480, 0XA441, 0X6C00, 0XACC1, 0XAD81, 0X6D40,
                                       0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01, 0X6AC0, 0X6B80, 0XAB41,
                                       0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1, 0XB981, 0X7940,
                                       0XBB01, 0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80, 0XBF41,
                                       0X7D00, 0XBDC1, 0XBC81, 0X7C40, 0XB401, 0X74C0, 0X7580, 0XB541,
                                       0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381, 0X7340,
                                       0XB101, 0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140,
                                       0X9301, 0X53C0, 0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741,
                                       0X5500, 0X95C1, 0X9481, 0X5440, 0X9C01, 0X5CC0, 0X5D80, 0X9D41,
                                       0X5F00, 0X9FC1, 0X9E81, 0X5E40, 0X5A00, 0X9AC1, 0X9B81, 0X5B40,
                                       0X9901, 0X59C0, 0X5880, 0X9841, 0X8801, 0X48C0, 0X4980, 0X8941,
                                       0X4B00, 0X8BC1, 0X8A81, 0X4A40, 0X4E00, 0X8EC1, 0X8F81, 0X4F40,
                                       0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 0X8581, 0X4540,
                                       0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341,
                                       0X4100, 0X81C1, 0X8081, 0X4040};

  uint8_t nTemp;
  uint16_t wCRCWord = 0xFFFF;

  while (wLength--) {
    nTemp = *nData++ ^ wCRCWord;
    wCRCWord >>= 8;
    wCRCWord ^= wCRCTable[nTemp];
  }
  return wCRCWord;
}