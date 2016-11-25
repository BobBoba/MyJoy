#include "main.h"

/*
void SysTick_Handler(void)
{
HAL_IncTick();
HAL_SYSTICK_IRQHandler();
}//*/

void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef clkinitstruct = { 0 };
	RCC_OscInitTypeDef oscinitstruct = { 0 };
	RCC_PeriphCLKInitTypeDef rccperiphclkinit = { 0 };

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	oscinitstruct.HSEState = RCC_HSE_ON;
	oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL9;

	oscinitstruct.PLL.PLLState = RCC_PLL_ON;
	oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

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

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 1000);

return ch;
}


//void print(const char* str)
//{
//	for (unsigned int i = 0; str[i]; i++)
//	{
//		while (!HAL_UART_GetState(&UartHandle))
//		{}
//		
//		HAL_UART_Transmit(&UartHandle, (uint8_t*)&str[i], 1, 1000);
//		//HAL_UART_Transmit(&UartHandle, (uint8_t*)"+", 1, 1000);
//	}
//}

static void Error_Handler(void)
{
	/* Turn LED_RED on */
	//BSP_LED_On(LED_RED);
	HAL_GPIO_WritePin(LED_BUS, LED_PIN, GPIO_PIN_SET);
	while (1)
	{
	}
}

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	//__GPIOA_CLK_ENABLE();
	/*
	GPIO_InitTypeDef GPIO_InitStructure = { 0 };
	GPIO_InitStructure.Pin = LED_PIN;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LED_BUS, &GPIO_InitStructure);
	//*/

	//*
	UartHandle.Instance = USARTx;

	UartHandle.Init.BaudRate = 9600;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	printf("Run!\n");

	//USBD_Init();
	//USBD_Register

	//HAL_PCD_EP_Open()


	//	if (HAL_UART_DeInit(&UartHandle) != HAL_OK)
	//	{
	//		// Initialization Error
	//		Error_Handler();
	//	}

	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		// Initialization Error
		Error_Handler();
	}//*/

	int step = 0;

	for (;;)
	{
		HAL_GPIO_WritePin(LED_BUS, LED_PIN, GPIO_PIN_SET);
		HAL_Delay(500);

		HAL_GPIO_WritePin(LED_BUS, LED_PIN, GPIO_PIN_RESET);
		HAL_Delay(500);

		//*
		printf("Step: %d\n", ++step);
	}
}