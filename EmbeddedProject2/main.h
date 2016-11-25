#pragma once

//#include "stm32f10x.h"
#include <stm32f1xx_hal.h>

//#include "usbd_core.h"
//#include "usbd_desc.h"
//#include "usbd_hid.h"

/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define LED_BUS GPIOC
#define LED_OK_PIN GPIO_PIN_14
#define LED_ERROR_PIN GPIO_PIN_15


/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */


#define USARTx                           USART1
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART1_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART1_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART1_RELEASE_RESET()

// Definition for USARTx Pins
#define USARTx_TX_PIN                    GPIO_PIN_9 //PA_9
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_RX_PIN                    GPIO_PIN_10 //PA_10
#define USARTx_RX_GPIO_PORT              GPIOA

#ifndef STDOUT_USART
#define STDOUT_USART 1
#endif

#ifndef STDERR_USART
#define STDERR_USART 1
#endif

#ifndef STDIN_USART
#define STDIN_USART 1
#endif


GPIO_InitTypeDef GPIO_OK_InitStructure, GPIO_ERROR_InitStructure;

// UART
UART_HandleTypeDef UartHandle;

static void Error_Handler(void);
void SystemClock_Config(void);

// USB
//USBD_HandleTypeDef USBD_Device;
//extern PCD_HandleTypeDef hpcd;
//__IO uint8_t JOYInitState = 0;
