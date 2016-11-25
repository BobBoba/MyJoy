#include "main.h"

int main(void)
{
	HAL_Init();
	
	SystemClock_Config();

	// needed for GPIO C
	__GPIOC_CLK_ENABLE();
	

	GPIO_OK_InitStructure.Pin = LED_OK_PIN;

	GPIO_OK_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_OK_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_OK_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LED_BUS, &GPIO_OK_InitStructure);

	GPIO_ERROR_InitStructure.Pin = LED_ERROR_PIN;

	GPIO_ERROR_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_ERROR_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_ERROR_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LED_BUS, &GPIO_ERROR_InitStructure);

	
	// UART
	//__GPIOA_CLK_ENABLE();
	UartHandle.Instance = USARTx;

	UartHandle.Init.BaudRate = 9600;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	
	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
	  /* Initialization Error */
		Error_Handler();
	}

	
	for (;;)
	{
		if (HAL_UART_Transmit(&UartHandle, (uint8_t *)"+", 1, 1000) != HAL_OK)
			Error_Handler();
		HAL_GPIO_WritePin(LED_BUS, LED_OK_PIN, GPIO_PIN_SET);
		HAL_Delay(500);
		
		HAL_GPIO_WritePin(LED_BUS, LED_OK_PIN, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
}


static void Error_Handler(void)
{
	HAL_GPIO_WritePin(LED_BUS, LED_ERROR_PIN, GPIO_PIN_SET);
	while (1)
	{
	}
}


void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef clkinitstruct = { 0 };
	RCC_OscInitTypeDef oscinitstruct = { 0 };
	RCC_PeriphCLKInitTypeDef rccperiphclkinit = { 0 };
  
	/* Enable HSE Oscillator and activate PLL with HSE as source */
	oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
	oscinitstruct.HSEState        = RCC_HSE_ON;
	oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
	oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;
    
	oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
	oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  
	if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
	{
		asm("bkpt 255");
	}
  
	/* USB clock selection */
	rccperiphclkinit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	rccperiphclkinit.UsbClockSelection = RCC_USBPLLCLK_DIV1_5;
	HAL_RCCEx_PeriphCLKConfig(&rccperiphclkinit);
  
	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	clocks dividers */
	clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
	clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
	{
		asm("bkpt 255");
	}
}
