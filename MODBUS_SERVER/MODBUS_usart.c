
/* Includes ------------------------------------------------------------------*/
#include "MODBUS_usart.h"
#include "MODBUS_Server.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"


UART_HandleTypeDef hlpuart1;

uint16_t MODBUS_UART_Rx_Count = 0;
uint8_t MODBUS_UART_Rx_Char = 0;
uint8_t MODBUS_UART_Tx_Buffer[MODBUS_UART_BUFFER_SIZE];
uint8_t MODBUS_UART_Rx_Buffer[MODBUS_UART_BUFFER_SIZE];

/**
 * @brief This function handles LPUART1 global interrupt.
 */
void LPUART1_IRQHandler(void) {
  HAL_UART_IRQHandler(&hlpuart1);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  BaseType_t xHigherPriorityTaskWoken;  // Variable to check if a higher priority task was woken  

  if (huart->Instance == LPUART1)  // Check if the interrupt is from LPUART1
  {
    MODBUS_UART_Rx_Count = Size;  // Store the number of bytes received
    // Process the received data in MODBUS_UART_Rx_Buffer
    // For example, you can print it to the console or send it back via UART
    MODBUS_UART_Rx_Buffer[MODBUS_UART_Rx_Count] = '\0';  // Null-terminate the received string
    xTaskNotifyFromISR(xModbusTaskHandle, 0, eSetBits, &xHigherPriorityTaskWoken);  // Notify the MODBUS task to process the received data
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );  // Yield to the higher priority task if needed
  }
}

/**
 * @brief This function is called when a complete reception is detected.
 * @param huart: UART handle pointer
 * @retval None
 */
// void MODBUS_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//   (void)huart; // Avoid unused parameter warning

//   if(MODBUS_UART_Rx_Count < MODBUS_UART_BUFFER_SIZE) {
//     MODBUS_UART_Rx_Buffer[MODBUS_UART_Rx_Count] = MODBUS_UART_Rx_Char; // Null-terminate the received string
//     MODBUS_UART_Rx_Count++;
//   } else {
//     MODBUS_UART_Rx_Count = 0; // Reset the count if buffer is full
//   }

//   if(HAL_UART_Receive_IT(&hlpuart1, &MODBUS_UART_Rx_Char, 1) != HAL_OK) // Start receiving data in interrupt mode
//     while(1); // Handle error if needed
// }

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
void MX_LPUART1_UART_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }

  /* LPUART1 clock enable */
  __HAL_RCC_LPUART1_CLK_ENABLE();

  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  /**LPUART1 GPIO Configuration
  PG7     ------> LPUART1_TX
  PG8     ------> LPUART1_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* LPUART1 interrupt Init */
  HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(LPUART1_IRQn);

  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = MODBUS_UART_BAUDRATE;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_UARTEx_ReceiveToIdle_IT(&hlpuart1, MODBUS_UART_Rx_Buffer, MODBUS_UART_BUFFER_SIZE) != HAL_OK)  // Start receiving data in interrupt mode
  {
    while (1);  // Handle error if needed
  }

  // HAL_UART_RegisterCallback(&hlpuart1, HAL_UART_RX_COMPLETE_CB_ID, MODBUS_UART_RxCpltCallback);

  // if(HAL_UART_Receive_IT(&hlpuart1, &MODBUS_UART_Rx_Char, 1) != HAL_OK) // Start receiving data in interrupt mode
  //   while(1); // Handle error if needed
}

/**
 * @brief LPUART1 DeInitialization Function
 * @param None
 * @retval None
 */
void MX_LPUART1_UART_DeInit(void) {
  /* Peripheral clock disable */
  __HAL_RCC_LPUART1_CLK_DISABLE();

  /**LPUART1 GPIO Configuration
  PG7     ------> LPUART1_TX
  PG8     ------> LPUART1_RX
  */
  HAL_GPIO_DeInit(GPIOG, GPIO_PIN_7 | GPIO_PIN_8);

  /* LPUART1 interrupt Deinit */
  HAL_NVIC_DisableIRQ(LPUART1_IRQn);
  /* USER CODE BEGIN LPUART1_MspDeInit 1 */
}
