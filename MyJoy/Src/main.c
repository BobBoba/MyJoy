/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_customhid.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
USBD_CUSTOM_HID_ItfTypeDef  USBD_CustomHID_fops_FS;
enum Buttons
{
	s1,// = 0x1,
	s2,// = 0x2,
	s3,// = 0x4,
	s4,// = 0x8,
	s5,// = 0x10,
	s6,// = 0x20,
	s7,// = 0x40,
	s8,// = 0x80,
	s9,// = 0x100,
	_num_buttons
};
u_char buttons[_num_buttons];


/* I2C SPEEDCLOCK define to max value: 400 KHz on STM32F1xx*/
#define ADS1115_I2C_SPEEDCLOCK   400000
#define ADS1115_I2C_DUTYCYCLE    I2C_DUTYCYCLE_2



#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t) 4)     /* Size of array containing ADC converted values */
/* Variable containing ADC conversions results */
__IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];


/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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
#pragma pack(push, 1)
struct joystick_report_t
{
	//uint8_t reportId;
	int16_t throttle;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t hat_and_buttons;
	//uint8_t padding;
};
#pragma pack(pop)

enum JoystickAxis
{
	X,
	Y,
	Z,
	Th,
	_num_axis
};

void ConfiugureReadyMode()
{
	ads111x_write_pointer(1);
	SetComparatorQueue(DISABLE_COMPARATOR_E);
	SetComparatorPolarity(ACTIVE_HIGH_E);

	  // Lo_thresh register
	ads111x_write_pointer(2);
	ads111x_write_rr(0x0000, 2);
	int data;
	data = ads111x_read();

	  // Hi_thresh register
	ads111x_write_pointer(3);
	ads111x_write_rr(0x9999, 3);
	data = ads111x_read();
}

#define ConfigureAdsForX() ConfigureAdsChannel(UNIPOLAR_AIN1_E, FULL_SCALE_4096_MV_E, DATA_RATE_64_SPS_E, CONTINUOUS_CONVERSION_MODE_E)
#define ConfigureAdsForY() ConfigureAdsChannel(UNIPOLAR_AIN2_E, FULL_SCALE_4096_MV_E, DATA_RATE_64_SPS_E, CONTINUOUS_CONVERSION_MODE_E)
#define ConfigureAdsForZ() ConfigureAdsChannel(UNIPOLAR_AIN3_E, FULL_SCALE_4096_MV_E, DATA_RATE_64_SPS_E, CONTINUOUS_CONVERSION_MODE_E)
#define ConfigureAdsForTh() ConfigureAdsChannel(UNIPOLAR_AIN4_E, FULL_SCALE_4096_MV_E, DATA_RATE_64_SPS_E, CONTINUOUS_CONVERSION_MODE_E)

int ConfigureAdsChannel(enum InputMultiplexer chanel, enum ProgrammableGain scale, enum DataRate datarate, enum Mode mode)
{
	if (HAL_OK != ads111x_write_pointer(ADS1015_REG_POINTER_CONFIG))
	{
		printf("ads111x_write_pointer(1) failed\r\n");
		return 0;
	}

	SetInputMultiplexer(chanel);
	SetProgrammableGain(scale);
	SetDataRate(datarate);
	SetMode(mode);
	return 0;
}

int SetChannel(enum InputMultiplexer channel)
{
	return 0;
	if (HAL_OK != ads111x_write_pointer(1))
	{
		printf("ads111x_write_pointer(1) failed\r\n");
		return 0;
	}
	
	SetInputMultiplexer(channel);

	//  HAL_Delay(4);

	//while (GetConversionStatus() == CURRENTLY_PERFORMING_CONVERSION_E)
	//	;   
  
	ads111x_write_pointer(0);
	return ads111x_read();

}
// -1 on error
int GetMeasure(int channel)
{
	if (HAL_OK != ads111x_write_pointer(ADS1015_REG_POINTER_CONFIG))
	{
		printf("ads111x_write_pointer(1) failed, restarting i2c\r\n");
		
		HAL_I2C_DeInit(&hi2c1);
		HAL_I2C_Init(&hi2c1);
		return -1;
	}
	
	//SetInputMultiplexer(channel);
	//SetMode(POWER_DOWN_SINGLE_SHOT_MODE_E);
	
	uint16_t config = 0;
	uint16_t OldValue;
	
	// Start with default values
	config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
	        ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
	        ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
	        ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
	        ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
	        ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

	                    // Set PGA/voltage range
	config |= GAIN_ONE;	

	//OldValue = ads111x_read();

	    // 
	    // Input multiplexer configuration (ADS1115 only) 
	    // 
	
  
	switch (channel)
	{
	case 0:
		config  |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
		break;
	case 1:
		config  |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
		break;
	case 2:
		config  |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
		break;
	case 3:
		config  |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
		break;
	}
	
	//
//	switch (CONTINUOUS_CONVERSION_MODE_E)
//	{
//	case POWER_DOWN_SINGLE_SHOT_MODE_E: 
//		Value = OldValue | MODE_MASK; 
//		break;
//	case CONTINUOUS_CONVERSION_MODE_E: 
//		Value = OldValue & ~MODE_MASK;
//		break;
//	}
	config |= ADS1015_REG_CONFIG_OS_SINGLE;
  

	ads111x_write_rr(config, 1);
	
	
	//SetMode(CONTINUOUS_CONVERSION_MODE_E);

	HAL_Delay(8);

		//while (GetConversionStatus() == CURRENTLY_PERFORMING_CONVERSION_E)
		//	;   
  
	ads111x_write_pointer(ADS1015_REG_POINTER_CONVERT);
	return ads111x_read();
}

//double ConvertToVolt( unsigned short x)
//{
//  return x / 1631.0;
//}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	  /* Configure the system clock */
	SystemClock_Config();

	  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();

	  /* USER CODE BEGIN 2 */
	

	printf("-=[ Run! ]=-\r\n");
	printf("size(joystick_report_t): %d bytes\r\n", sizeof(struct joystick_report_t));
	
//	if (HAL_ADC_Start(&hadc1) != HAL_OK)
//	{
//		printf("HAL_ADC_Start: failed\r\n");
//	  /* Start Error */
//		Error_Handler();
//	}
//	
//		/* Run the ADC calibration */  
//	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
//	{
//		printf("HAL_ADCEx_Calibration_Start: failed\r\n");
//	  /* Calibration Error */
//		Error_Handler();
//	}
//	
//	/* Start ADC conversion on regular group with transfer by DMA */
//	if (HAL_ADC_Start(&hadc1
//		//, (uint32_t *)aADCxConvertedValues, ADCCONVERTEDVALUES_BUFFER_SIZE
//		) != HAL_OK)
//	{
//		printf("HAL_ADC_Start: failed\r\n");
//	  /* Start Error */
//		Error_Handler();
//	}
//	
//	
//	uint32_t adc_initial_y = HAL_ADC_GetValue(&hadc1);
//	int32_t minY = 0, maxY = 0;
	
	if (HAL_I2C_IsDeviceReady(&hi2c1, ADS1115_REMOTE_ADR, 1000, 3000) != HAL_OK)
	{
		printf("HAL_I2C_IsDeviceReady: false\r\n");
		/* Start Error */
		Error_Handler();
	}

	//ConfiugureReadyMode();
//	ConfigureAdsForX();
//	ConfigureAdsForY();
//	ConfigureAdsForZ();
//	ConfigureAdsForTh();
	while (HAL_OK != ads111x_write_pointer(ADS1015_REG_POINTER_CONFIG))
	{
		printf("ads111x_write_pointer(ADS1015_REG_POINTER_CONFIG) failed\r\n");
		HAL_Delay(1000);
	}

	for (int a = 0; a < _num_axis; ++a)
	{
		int r = GetMeasure(a);
		if (r == -1)
			printf("GetMeasure(%d) failed\r\n", a);
	}
	//SetProgrammableGain(FULL_SCALE_4096_MV_E);
	//SetDataRate(DATA_RATE_64_SPS_E);
	//SetMode(CONTINUOUS_CONVERSION_MODE_E);
	//for (int i = 0; i < 4; ++i)
	//{		
		//SetInputMultiplexer(UNIPOLAR_AIN1_E + i);
	//}
	
	const int middle = 10;
	int initial[_num_axis], initial_array[middle][_num_axis], min[_num_axis], max[_num_axis], norm[_num_axis];
	for (int i = 0; i < middle; ++i)
	{		
		//printf("initial x/y/z/th: ");
		for (int a = 0; a < _num_axis; ++a)
		{
			initial_array[i][a] = GetMeasure(a);
		
			//printf("%6d ", initial_array[i][a]);	
		}
		//printf("\r\n");
		//HAL_Delay(100);
	}
	
	printf("initial axis values for x/y/z/th: ");
	for (int a = 0; a < _num_axis; ++a)
	{
		initial[a] = min[a] = max[a] = norm[a] = 0;
		for (int i = 0; i < middle; ++i)
		{
			initial[a] += initial_array[i][a];
		}
		initial[a] /= middle;
		printf("%6d ", initial[a]);
	}
	printf("\r\n");
	
	// throttle does not started from center
	min[Th] = max[Th] = initial[Th];

//	int initialX =	GetMeasure(0);//a3
//	int initialY =	GetMeasure(1);//a0
//	int initialZ =	GetMeasure(2);//a1...
//	int initialTh = GetMeasure(3);//a2
	
	//for(int i = 0; i < 6; ++i)
	//	HAL_GPIO_WritePin(GPIOA, )
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);


		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int step = -1;
	int printOn = 1;
	u_char debugAxis = 0;
	u_char debugGpio = 1; 
	while (1)
	{
	  
		//HAL_Delay(100);
		++step;
		
		if (step % printOn == 0)
			printf("[%4d] ", step / printOn);
		//HAL_ADC_PollForConversion(&hadc1, 1000);
//		int32_t adc_y = HAL_ADC_GetValue(&hadc1) - adc_initial_y;
//		int32_t adc_y2 = HAL_ADC_GetValue(&hadc1) - adc_initial_y;
//		int32_t adc_y3 = HAL_ADC_GetValue(&hadc1) - adc_initial_y;
//		int32_t adc_y4 = HAL_ADC_GetValue(&hadc1) - adc_initial_y;
//		
//		printf("[%d] %d %d %d %d\r\n", 
//			step,
//			adc_y,
//			adc_y2, 
//			adc_y3,
//			adc_y4);
		
		
		
		
//		raw[X] = GetMeasure(0);//a3
//		raw[Y] = GetMeasure(1);//a0
//		raw[Z] = GetMeasure(2);//a1...
//		raw[Th] = GetMeasure(3);//a2
//
//		if (step % printOn == 0)
//			printf("ads1115 raw: %d %d %d %d ", raw[X], raw[Y], raw[Z], raw[Th]);
//
		if (debugAxis && step % printOn == 0)
			printf("r/z/m/M/n: ");	
		for (int a = 0; a < _num_axis; ++a)
		{
			int iter = GetMeasure(a);
			
			if (iter == -1)
				printf("\r\nGetMeasure(%d) failed\r\n", a);
			
			if (debugAxis && step % printOn == 0)
				printf("%6d", iter);
			
			if (a != Th)
			{				
			
				iter -= initial[a];
				if (debugAxis && step % printOn == 0)
					printf("%6d", iter);
			}
			
			if (iter < min[a])
				min[a] = iter;
			if (debugAxis && step % printOn == 0)
				printf("%6d", min[a]);
			
			if (iter > max[a])
				max[a] = iter;
			if (debugAxis && step % printOn == 0)
				printf("%6d", max[a]);
			
			if (a == Th)
			{
				iter -= min[a]; // bind to zero
				iter -= (max[a] - min[a]) / 2; // switch to pos/neg legs
				
				int M = (max[a] - min[a]) / 2;
				int m = -(max[a] - min[a]) / 2;
				if (iter > 0)
					iter = iter * 32767 / M;
				else if (iter < 0)
					iter = -iter * 32768 / m;
			}
			else 
			{
				if (iter > 0)
					iter = iter * 32767 / max[a];
				else if (iter < 0)
					iter = iter * 32768 / -min[a];
			}
			if (debugAxis && step % printOn == 0)
				printf("%6d|", iter);
			
			norm[a] = iter;
		}
		if (debugAxis && step % printOn == 0)
		{
			printf("\r\n");
		}
		
		
		// GPIO
		

		if (debugGpio && step % printOn == 0)
		{			
			printf("buttons: ");
		}
		

		
		for (int i = 0; i < 6; ++i)
		{
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 << i, GPIO_PIN_SET);
		}
		  /*Configure GPIO pin : PA6 */
//		GPIO_InitTypeDef GPIO_InitStruct;
//		GPIO_InitStruct.Pin = GPIO_PIN_3;
//		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); 
		//HAL_Delay(1);
		buttons[s7] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		buttons[s8] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		buttons[s9] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); 
		buttons[s2] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		buttons[s1] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		buttons[s6] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
		buttons[s5] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		buttons[s4] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		buttons[s3] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
		
//		if (debugGpio && step % printOn == 0)
//			printf("1, 4: %d, %d", 
//				HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1), 
//				HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4));
		
		for (int i = 0; i < _num_buttons; ++i)
		{
			if (debugGpio && step % printOn == 0)
				printf("[%d]:%d ", i, buttons[i]);
			
		}
		
		if (debugGpio && step % printOn == 0)
			printf("\r\n");
		
		struct joystick_report_t report = { 
			(int16_t)norm[Th], 
			(int16_t)norm[X], 
			(int16_t)norm[Y], 
			(int16_t)norm[Z], 
			0
		};
		
		//HAL_ADC_ConvCpltCallback(&hadc1);
		USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));

	  
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	    /**Initializes the CPU, AHB and APB busses clocks 
	    */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	    /**Initializes the CPU, AHB and APB busses clocks 
	    */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}

	    /**Configure the Systick interrupt time 
	    */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	    /**Configure the Systick 
	    */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	  /* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 144;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
	                           PA4 PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin : PA6 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 
	                        | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
	printf("!!! FATAL ERROR !!!");
  /* User can add his own implementation to report the HAL error return state */
	while (1) 
	{
	}
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
