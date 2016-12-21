#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"
#include "usbd_customhid.h"

extern I2C_HandleTypeDef hi2c1;
//extern DMA_HandleTypeDef hdma_i2c1_rx;

extern UART_HandleTypeDef huart1;

// USB
__IO uint8_t PrevXferComplete = 1;

//enum Buttons
//{
//	s1_fire,// = 0x1, //fire
//	s2_05,// = 0x2, //
//	s3_03,// = 0x4,   //
//	s4_02,// = 0x8,
//	s5_04,// = 0x10,
//	s6_h_u,// = 0x20,
//	s7_h_r,// = 0x40,
//	s8_h_d,// = 0x80,
//	s9_h_l,// = 0x100,
//	_num_buttons
//};
//u_char buttons[_num_buttons];


/* I2C SPEEDCLOCK define to max value: 400 KHz on STM32F1xx*/
#define ADS1115_I2C_SPEEDCLOCK   400000
#define ADS1115_I2C_DUTYCYCLE    I2C_DUTYCYCLE_2



#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t) 4)     /* Size of array containing ADC converted values */
/* Variable containing ADC conversions results */
__IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

int initial[_num_axis], 
	initial_array[MIDDLE_POINT_ITERATIONS][_num_axis], 
	min[_num_axis], 
	max[_num_axis], 
	norm[_num_axis];

const u_char INVERT_X = 1;
const u_char INVERT_Y = 0;
const u_char INVERT_Z = 0;
const u_char INVERT_Th = 0;

int step = -1;
int printOn = 20;
u_char debugAxis = 1;
u_char debugGpio = 0; 


/* Buffer used for transmission */
uint8_t aTxBuffer[16];

/* Buffer used for reception */
uint8_t aRxBuffer[RXBUFFERSIZE];


// USB report
struct joystick_report_t report = { 0 };


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
	//return 0;
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

int GetMeasure(int channel)
{
	ads111x_write_pointer(1);
	//SetMode(POWER_DOWN_SINGLE_SHOT_MODE_E);
	SetInputMultiplexer(UNIPOLAR_AIN1_E + channel);

	HAL_Delay(8);

	//while (GetConversionStatus() == CURRENTLY_PERFORMING_CONVERSION_E){}
  
	ads111x_write_pointer(0);
	return ads111x_read();
}


int GetMeasure_2(int channel)
{
	if (HAL_OK != ads111x_write_pointer(ADS1015_REG_POINTER_CONFIG))
	{
		printf("ads111x_write_pointer(1) failed\r\n");
		return -1;
	}
	
	SetInputMultiplexer(UNIPOLAR_AIN1_E + channel);
	//SetMode(POWER_DOWN_SINGLE_SHOT_MODE_E);
	
	
//	while (GetConversionStatus() == CURRENTLY_PERFORMING_CONVERSION_E)
//	{
//		printf("CURRENTLY_PERFORMING_CONVERSION_E...\r\n");
//	}
	
	if (HAL_OK != ads111x_write_pointer(ADS1015_REG_POINTER_CONVERT))
	{
		printf("ads111x_write_pointer(1) failed, restarting i2c\r\n");
		
		//HAL_I2C_DeInit(&hi2c1);
		//HAL_I2C_Init(&hi2c1);
		return -1;
	}
	
	return ads111x_read();
}
// -1 on error
int GetMeasure_(int channel)
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
	        ADS1015_REG_CONFIG_DR_3300SPS   | // 1600 samples per second (default)
	        ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

	                    // Set PGA/voltage range
	config |= ADS1015_REG_CONFIG_PGA_4_096V;	

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
	
	config |= ADS1015_REG_CONFIG_OS_SINGLE;
  

	ads111x_write_rr(config, 1);
	
	
	//HAL_Delay(8);

	while (GetConversionStatus() == CURRENTLY_PERFORMING_CONVERSION_E)
		;   
  
	ads111x_write_pointer(ADS1015_REG_POINTER_CONVERT);
	return ads111x_read();
}

void init()
{
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
	
	if (HAL_I2C_IsDeviceReady(&hi2c1, ADS1115_REMOTE_ADR, 10, 100) != HAL_OK)
	{
		printf("HAL_I2C_IsDeviceReady: false\r\n");
		/* Start Error */
		Error_Handler();
	}
	
	  /*##-4- Put I2C peripheral in reception process ###########################*/  
	while (HAL_I2C_Master_Receive_DMA(&hi2c1, (uint16_t)ADS1115_REMOTE_ADR, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
	{
	  /* Error_Handler() function is called when Timeout error occurs.
	     When Acknowledge failure occurs (Slave don't acknowledge its address)
	     Master restarts communication */
		if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
		{
			Error_Handler();
		}
	}


	//ConfiugureReadyMode();
//	ConfigureAdsForX();
//	ConfigureAdsForY();
//	ConfigureAdsForZ();
//	ConfigureAdsForTh();
//	while (HAL_OK != ads111x_write_pointer(ADS1015_REG_POINTER_CONFIG))
//	{
//		printf("ads111x_write_pointer(ADS1015_REG_POINTER_CONFIG) failed, retry...\r\n");
//		HAL_Delay(1000);
//	}
	
	HAL_Delay(10);
	if (HAL_OK != ads111x_write_pointer(ADS1015_REG_POINTER_CONFIG))
	{
		printf("channel %d: ads111x_write_pointer(1) failed\r\n", 0);
		//continue;
	}
		
	//SetInputMultiplexer(UNIPOLAR_AIN1_E + channel);
	SetProgrammableGain(FULL_SCALE_4096_MV_E);
	SetDataRate(DATA_RATE_860_SPS_E);
	SetMode(CONTINUOUS_CONVERSION_MODE_E);
	//SetMode(POWER_DOWN_SINGLE_SHOT_MODE_E);
	SetComparatorQueue(DISABLE_COMPARATOR_E);
	SetComparatorPolarity(ACTIVE_LOW_E);

#if 0
	for (int channel = 0; channel < _num_axis; ++channel)
	{


		
		continue;

	// Start with default values
		uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
		                  ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
		                  ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
		                  ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
		                  ADS1015_REG_CONFIG_DR_3300SPS   | 
		                  ADS1015_REG_CONFIG_MODE_SINGLE;  

		                 
		// Set PGA/voltage range
		config |= ADS1015_REG_CONFIG_PGA_4_096V;

			  // Set single-ended input channel
		switch (channel)
		{
		case (0):
			config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
			break;
		case (1):
			config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
			break;
		case (2):
			config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
			break;
		case (3):
			config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
			break;
		}

			  // Set 'start single-conversion' bit
		//config |= ADS1015_REG_CONFIG_OS_SINGLE;

			  // Write config register to the ADC
		if (HAL_OK != ads111x_write_rr(config, ADS1015_REG_POINTER_CONFIG))
		{
			printf("channel %d: ads111x_write_rr(config, 1) failed\r\n", channel);
			continue;
		}
		//writeRegister(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
			//int r = GetMeasure(a);
			//if (r == -1)
			//	printf("GetMeasure(%d) failed\r\n", a);
	}
#endif
	//SetProgrammableGain(FULL_SCALE_4096_MV_E);
	//SetDataRate(DATA_RATE_64_SPS_E);
	//SetMode(CONTINUOUS_CONVERSION_MODE_E);
	//for (int i = 0; i < 4; ++i)
	//{		
		//SetInputMultiplexer(UNIPOLAR_AIN1_E + i);
	//}
	
	HAL_Delay(10);

	
	for (int i = 0; i < MIDDLE_POINT_ITERATIONS; ++i)
	{		
		printf("X/Y/R/Th: ");
		for (int a = 0; a < _num_axis; ++a)
		{
			initial_array[i][a] = GetMeasure(a);
		
			printf("%6d ", initial_array[i][a]);	
		}
		printf("\r\n");
		HAL_Delay(10);
	}
	
	printf("X/Y/R/Th: ");
	for (int a = 0; a < _num_axis; ++a)
	{
		initial[a] = min[a] = max[a] = norm[a] = 0;
		for (int i = 0; i < MIDDLE_POINT_ITERATIONS; ++i)
		{
			initial[a] += initial_array[i][a];
		}
		initial[a] /= MIDDLE_POINT_ITERATIONS;
		//min[a] = max[a] = norm[a] = initial[a];
		printf("%6d ", initial[a]);
	}
	printf(" AVERAGE \r\n");
	
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
}

//double ConvertToVolt( unsigned short x)
//{
//  return x / 1631.0;
//}

void RHIDCheckState()
{
		/* Reset the control token to inform upper layer that a transfer is ongoing */
	PrevXferComplete = 0;

	
	++step;

	//if ((debugAxis || debugGpio) && step % printOn == 0)
	//	printf("[%4d] ", step / printOn);

	if (debugAxis && step % printOn == 0)
		printf("v/m/M/n: ");
		
	for (int a = 0; a < _num_axis; ++a)
	{
		if (debugAxis && step % printOn == 0 && a > 0)
			printf(" | ");
		
		int iter = GetMeasure(a);
			
		if (iter < 0)
		{
			printf("\r\nGetMeasure(%d) failed\r\n", a);
			continue;
		}		
		
		// regular axis
		if (a != Th)
		{		
			// bind to zero axis
			iter -= initial[a];
		}
		
		// calculate min
		if (iter < min[a])
			min[a] = iter;
		
		// calculate max
		if (iter > max[a])
			max[a] = iter;	
		
		// throttle
		if (a == Th)
		{
			iter -= (max[a] + min[a]) / 2;
		}

		// 1. display current value			
		if (debugAxis && step % printOn == 0)
			printf("%5d", iter);			
		
		// 2. display min value
		if (debugAxis && step % printOn == 0)
			printf("%6d", min[a]);			
		
		// 3. display max value
		if (debugAxis && step % printOn == 0)
			printf("%6d", max[a]);		
		
		// regular axis
		if (a != Th)
		{		
			if (iter > 0)
				iter = iter * 32767 / max[a];
			else if (iter < 0)
				iter = iter * 32768 / -min[a];
			
			if (INVERT_X && a == X)
				iter = -iter;
			
			if (INVERT_Y && a == Y)
				iter = -iter;
			
			if (INVERT_Z && a == R)
				iter = -iter;

			// 4. relative to zero point
			//if (debugAxis && step % printOn == 0)
			//	printf("%6d", iter);			
			
		}
		// throttle axis without vertical aligning spring
		else 
		{
				
			int M = (max[a] - min[a]) / 2;
			int m = -(max[a] - min[a]) / 2;
			if (iter > 0)
				iter = iter * 32767 / M;
			else if (iter < 0)
				iter = -iter * 32768 / m;
			
			if (INVERT_Th && a == Th)
				iter = -iter;
		}
		
		if (iter > 32767)
			iter = 32767;
		if (iter < -32768)
			iter = -32768;
		
		// normalized
		if (debugAxis && step % printOn == 0)
			printf("%6d", iter);
					
		norm[a] = iter;
	}
//	if (debugAxis && step % printOn == 0)
//	{
//		printf("\r\n");
//	}
		
		
	// GPIO


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	GPIO_PinState s7_h_r = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	GPIO_PinState s8_h_d = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	GPIO_PinState s9_h_l = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		
		
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); 
	GPIO_PinState s2_05 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	GPIO_PinState s1_fire = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	GPIO_PinState s6_h_u = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); 
	GPIO_PinState s5_04 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	GPIO_PinState s4_02 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	GPIO_PinState s3_03 = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		
	//		if (debugGpio && step % printOn == 0)
	//			printf("1, 4: %d, %d", 
	//				HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1), 
	//				HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4));		
	if (debugGpio && step % printOn == 0)
		printf("hat %d,%d,%d,%d buttons: %d,%d,%d,%d,%d", 
			s6_h_u,
			s9_h_l,
			s8_h_d,
			s7_h_r,
			s1_fire,
			s2_05,
			s3_03,
			s4_02,
			s5_04);
		
			
	// h0 h1 h2 h3 b1 b2 b3 b4 b5 
	int16_t bit = 0;
	report.hat_and_buttons = 0;

	if (s6_h_u)
	{
		report.hat_and_buttons |= 0;
	}
	else if (s9_h_l)
	{
		report.hat_and_buttons |= 1;
	}
	else if (s8_h_d)
	{
		report.hat_and_buttons |= 2;
	}
	else if (s7_h_r)
	{
		report.hat_and_buttons |= 3;
	}
	else
	{
		report.hat_and_buttons |= 4; // neutral
			
	}
	bit += 4;
		
	report.hat_and_buttons |= s1_fire << bit++;
	report.hat_and_buttons |= s2_05 << bit++;
	report.hat_and_buttons |= s3_03 << bit++;
	report.hat_and_buttons |= s4_02 << bit++;
	report.hat_and_buttons |= s5_04 << bit++;
	

	
	report.throttle = (int16_t)norm[Th];
	report.x = (int16_t)norm[X]; 
	report.y = (int16_t)norm[Y]; 
	report.z = (int16_t)norm[R];
		
	//HAL_ADC_ConvCpltCallback(&hadc1);
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&report, sizeof(report));
	
	if ((debugAxis || debugGpio) && step % printOn == 0)
		printf("\r\n");

}
