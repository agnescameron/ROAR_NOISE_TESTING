/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * MORE ROARx3 - 15/7/24 JR_AC
 *
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "math.h"
#define NS 256 // number of samples in wavetable 0 - 255
#define MODES 1 // number of modes
#define DECAY_RANGE 20 // rates of decay -> bell curve

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

uint32_t DstAddress = (uint32_t) &(TIM1->CCR1);

/* tables --------------------------------------------------------------------*/

// sine wave 8 bit resolution scaled to 94% max val
uint32_t sine[NS] = { 120, 123, 126, 129, 132, 134, 137, 140, 143, 146, 149,
		152, 155, 157, 160, 163, 165, 168, 171, 174, 177, 179, 181, 184, 186,
		189, 191, 194, 196, 198, 200, 202, 205, 207, 209, 211, 212, 214, 216,
		218, 220, 221, 223, 224, 226, 227, 228, 229, 230, 231, 233, 234, 235,
		235, 236, 237, 238, 238, 239, 239, 239, 240, 240, 240, 240, 240, 240,
		240, 239, 239, 239, 238, 238, 237, 236, 235, 235, 234, 233, 231, 230,
		229, 228, 227, 226, 224, 223, 221, 220, 218, 216, 214, 212, 211, 209,
		207, 205, 202, 200, 198, 196, 194, 191, 189, 186, 184, 181, 179, 177,
		174, 171, 168, 165, 163, 160, 157, 155, 152, 149, 146, 143, 140, 137,
		134, 132, 129, 126, 123, 120, 117, 114, 111, 108, 105, 102, 100, 97, 94,
		91, 87, 85, 83, 80, 77, 74, 71, 69, 66, 63, 61, 58, 55, 54, 51, 49, 46,
		44, 41, 39, 38, 35, 33, 31, 29, 27, 25, 24, 22, 20, 19, 17, 16, 14, 13,
		11, 10, 9, 8, 7, 6, 5, 5, 4, 3, 2, 2, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
		1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 8, 9, 10, 11, 13, 14, 16, 17, 19, 20, 22,
		24, 25, 27, 29, 31, 33, 35, 38, 39, 41, 44, 46, 49, 51, 54, 55, 58, 61,
		63, 66, 69, 71, 74, 77, 80, 83, 85, 87, 91, 94, 97, 100, 102, 105, 108,
		111, 114, 117 };

// noise
uint32_t noise[NS];

// wave_LUT
uint32_t wave_LUT[NS];

// tmp
uint32_t tmp_wav_1[NS];
uint32_t tmp_wav[NS];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// core/mode etc. select
uint8_t mode_sel = 1;
uint8_t core_sel = 1;
uint8_t buzz_sel = 1;
uint8_t XOR_sel = 1;
uint8_t swap_sel = 1;
uint8_t decay_multiplier = 0;

// debounce
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

// ADC setup
volatile uint16_t adcResultsDMA[2];
const int adcChannelCount = sizeof(adcResultsDMA) / sizeof(adcResultsDMA[0]);
volatile int adcConversionComplete = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {

	currentMillis = HAL_GetTick(); // debounce
//
	if (GPIO_Pin == GPIO_PIN_9 && (currentMillis - previousMillis > 20)) { // 20 ms debounce
		decay_multiplier += 1;
	}

		//		mode_sel = mode_sel + 1;
//		buzz_sel = buzz_sel + 9;
//		swap_sel = swap_sel + 3;
//		XOR_sel = XOR_sel + 1; // wraps around > 256
//		previousMillis = currentMillis;
//
//		if (mode_sel % MODES == 1) {
//			decay_multiplier += 1;
//			core_sel = core_sel + 1;
//		}
//
//	}
////
//	if (mode_sel > MODES) { // see defines
//		mode_sel = 1;
//
//	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

//	// Map functions
//	uint32_t take_log(uint32_t num) {
//		double num_log = sqrt(num);
//		num_log = num_log * 10;
//		uint32_t int_log = (int) num_log;
//		return int_log;
//	}
//
//	uint32_t MAP(uint32_t au32_IN, uint32_t au32_INmin, uint32_t au32_INmax,
//			uint32_t au32_OUTmin, uint32_t au32_OUTmax) {
//		return ((((au32_IN - au32_INmin) * (au32_OUTmax - au32_OUTmin))
//				/ (au32_INmax - au32_INmin)) + au32_OUTmin);
//	}
//
//	uint32_t map_counter_scale(uint32_t num, uint32_t divisor,
//			uint8_t divisor_bitshift, uint32_t offset) {
//		num = num - offset;
//		if (num < 0)
//			num = 0;
//		float num_div = (float) num / (divisor >> divisor_bitshift); // get a number between 1 and 20
//		return (uint32_t) num_div;
//	}
	// Noise function
	/* USER CODE BEGIN 1 */
//	// Functions

	void noise_func(uint32_t dest[], uint32_t src[], uint32_t length, int decay) {
		int decay_adj = round(decay*0.4);
		for (int i = 0; i < length; i++) {
			dest[i] = src[i];
		}

		for (int i = 0; i < decay_adj; i++)
			dest[rand() % length] = (dest[rand() % length] + rand() % 128) % 256;
	}


	void rot(uint32_t dest[], uint32_t src[], uint32_t length, int decay) {
		int decay_adj = round(decay*0.6);
		for (int i = 0; i < length; i++) {
			dest[i] = src[i];
		}

		if(decay_adj > length) decay_adj = length;

		for (int i = 0; i < decay_adj; i++)
			dest[i] = (dest[i] + rand() % 128) % 256;
	}

	void clipper_func(uint32_t dest[], uint32_t src[], int length, int num) {
		for (int i = 0; i < length; i++) {
			dest[i] = src[i];
			dest[i] += num;
		}
	}
//
	void XOR_func(uint32_t dest[], uint32_t src[], int length, int decay) {
		for (int i = 0; i < length; i++) {
			dest[i] = src[i];
			dest[i] ^= decay;
		}
	}

	//
		void inc_XOR(uint32_t dest[], uint32_t src[], int length, int decay) {
			for (int i = 0; i < length; i++) {
				dest[i] = src[i];
				dest[i] |= decay;
			}
		}

	void shuffle_func(uint32_t dest[], uint32_t src[], int length, int decay) {
		//need to work out how to make a function of decay
		for (int i = 0; i < length; i++) {
			dest[i] = src[i];
		}

		for (int i=0; i<decay; i++) {
			int swap_index = rand() % swap_sel;
			int rand_ind = rand() % length;

			// swap whatever is at index i with whatever is at the swap_index
			uint32_t temp = dest[rand_ind];
			dest[rand_ind] = dest[swap_index];
			dest[swap_index] = temp;
		}
	}
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

////PWM test
//	TIM1->CCR1 = 128;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
	HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t) sine, DstAddress, NS);
	__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
	// Calibrate The ADC On Power-Up For Better Accuracy
	HAL_ADCEx_Calibration_Start(&hadc1);

	//set up variables
	memcpy(wave_LUT, sine, sizeof(sine));
	srand(time(NULL));


	uint32_t random_number = rand();
	uint32_t decay_rate = 1 + random_number%DECAY_RANGE;
	uint32_t ctr = 0;
	uint32_t phase = 0;
	uint32_t phase_add = 15;
	uint32_t freq = 1000;
	uint32_t ctr_scale = 0;
	uint32_t table_lookup = 1000; // renamed
	uint32_t ad0_bitshift = 1000;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		if(decay_multiplier > 0){
			int rate = decay_rate*decay_multiplier;


			// xor then rot -- goes to noise
//			inc_XOR(tmp_wav_1, sine, NS, rate);
//			rot(tmp_wav, tmp_wav_1, NS, rate);

			// rot then xor -- goes to 0
			rot(tmp_wav_1, sine, NS, rate);
			inc_XOR(tmp_wav, tmp_wav_1, NS, rate);

			//			XOR_func(tmp_wav_1, sine, NS, decay_rate*decay_multiplier);
	//		clipper_func(tmp_wav, sine, NS, decay_rate*decay_multiplier);
//			noise_func(tmp_wav, tmp_wav_1, NS, decay_rate*decay_multiplier);

	//		shuffle_func(tmp_wav, sine, NS, decay_rate*decay_multiplier);
			memcpy(wave_LUT, tmp_wav, sizeof(tmp_wav)); // dest, src, size(src) - acts as a buffer for passing to DMA1
			DMA1_Channel1->CMAR = (uint32_t) wave_LUT; // SrcAddress

			if (rate > NS-1) decay_multiplier = 0;
		}

		else DMA1_Channel1->CMAR = (uint32_t) sine; // SrcAddress

		freq = (adcResultsDMA[0] >> 3);


		if (freq <= 0) freq = 1;
		TIM2 -> ARR = freq;

		/* USER CODE BEGIN 3 */
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, adcChannelCount);
		while (adcConversionComplete == 0) {

		}
		adcConversionComplete = 0;
	} // end of while loop!
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = ENABLE;
	hadc1.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_256;
	hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
	hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_16;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_17;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 256 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin : PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// Conversion Complete & DMA Transfer Complete As Well
	adcConversionComplete = 1;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
