#pragma once

//#include "stm32f10x.h"
#include <stm32f1xx_hal.h>

//#include "usbd_core.h"
//#include "usbd_desc.h"
//#include "usbd_hid.h"

/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_9 //PA_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_RX_PIN                    GPIO_PIN_10 //PA_10
#define USARTx_RX_GPIO_PORT              GPIOA

// On-board LED
#define LED_BUS GPIOC
#define LED_PIN GPIO_PIN_13

// UART
UART_HandleTypeDef UartHandle;

// USB
//USBD_HandleTypeDef USBD_Device;
//extern PCD_HandleTypeDef hpcd;
//__IO uint8_t JOYInitState = 0;
