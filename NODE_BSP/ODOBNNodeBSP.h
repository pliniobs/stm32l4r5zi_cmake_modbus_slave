#ifndef NUCLEO_L4R5_BSP_H
#define NUCLEO_L4R5_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

// LED Definitions
#define LED_RED_PIN        GPIO_PIN_14
#define LED_RED_GPIO_PORT  GPIOB

#define LED_GREEN_PIN        GPIO_PIN_15
#define LED_GREEN_GPIO_PORT  GPIOB

void BSP_LED_Init(void);
void BSP_LED_On(uint16_t led_pin, GPIO_TypeDef *led_port);
void BSP_LED_Off(uint16_t led_pin, GPIO_TypeDef *led_port);
void BSP_LED_Toggle(uint16_t led_pin, GPIO_TypeDef *led_port);

#ifdef __cplusplus
}
#endif

#endif // NUCLEO_L4R5_BSP_H