#include "ODOBNNodeBSP.h"
#include "main.h"
#include "stm32l4xx_hal.h"

void BSP_LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable the GPIO clocks for the LED ports
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure the LED pins as output
    GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(LED_RED_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_Init(LED_GREEN_GPIO_PORT, &GPIO_InitStruct);

    // Initialize the LEDs to be off
    HAL_GPIO_WritePin(LED_RED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_PORT, LED_GREEN_PIN, GPIO_PIN_RESET);
}